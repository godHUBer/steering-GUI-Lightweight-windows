"""Windows Raw Input relative-mouse source for Mouse Steering.

This module intentionally imports on every platform without touching Win32.
`RawInputDeltaSource.start()` is the first operation that accesses ``ctypes.windll``
and it rejects non-Windows hosts explicitly.  That keeps ``--help`` and the
pure deterministic suite independent of Windows APIs.

The source owns a message-only window on a dedicated thread. It registers the
generic-desktop mouse collection with ``RIDEV_INPUTSINK``, reads each
``WM_INPUT`` packet with ``GetRawInputData``, and queues only relative
``RAWMOUSE.lLastX`` counts. It never reads or writes cursor position.
"""

from __future__ import annotations

import ctypes
import math
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable


# ---------------------------------------------------------------------------
# Public source/queue policy
# ---------------------------------------------------------------------------


DEFAULT_RAW_INPUT_QUEUE_CAPACITY = 2048
"""Maximum pending relative-delta events before the source drops the oldest."""

RAW_INPUT_OVERFLOW_POLICY = "drop_oldest"
"""Retain the newest physical input under overload instead of adding latency."""


class RawInputError(RuntimeError):
    """Base error for Raw Input setup or packet-read failures."""


class RawInputUnavailable(RawInputError):
    """Raised when the Windows-only source cannot be used in this runtime."""


class TimestampedDeltaQueue:
    """Thread-safe bounded queue of monotonic-timestamped horizontal deltas.

    The queue deliberately drops its *oldest* event at capacity. A steering
    control loop is real-time: retaining old movement would grow latency and
    replay stale steering after a stall. ``dropped_event_count`` makes that
    unavoidable loss observable to the runtime/UI instead of hiding it.
    """

    def __init__(self, capacity: int = DEFAULT_RAW_INPUT_QUEUE_CAPACITY) -> None:
        if isinstance(capacity, bool) or not isinstance(capacity, int) or capacity <= 0:
            raise ValueError("Raw Input queue capacity must be a positive integer.")
        self._capacity = capacity
        self._lock = threading.Lock()
        self._events: deque[tuple[float, float]] = deque()
        self._dropped_event_count = 0

    @property
    def capacity(self) -> int:
        return self._capacity

    @property
    def pending_event_count(self) -> int:
        with self._lock:
            return len(self._events)

    @property
    def dropped_event_count(self) -> int:
        with self._lock:
            return self._dropped_event_count

    def put(self, timestamp: float, dx: float) -> bool:
        """Queue one nonzero delta, returning whether an event was accepted.

        The message thread is serial in normal use, but stable timestamp order
        is retained if a test/integration producer arrives out of order.
        """

        timestamp = float(timestamp)
        dx = float(dx)
        if not math.isfinite(timestamp) or not math.isfinite(dx):
            raise ValueError("Raw Input timestamp and delta must be finite.")
        if dx == 0.0:
            return False

        event = (timestamp, dx)
        with self._lock:
            if len(self._events) >= self._capacity:
                self._events.popleft()
                self._dropped_event_count += 1

            if not self._events or timestamp >= self._events[-1][0]:
                self._events.append(event)
            else:
                insert_at = len(self._events)
                while insert_at > 0 and self._events[insert_at - 1][0] > timestamp:
                    insert_at -= 1
                self._events.insert(insert_at, event)
        return True

    def take_through(self, timestamp: float) -> float:
        """Consume input owned by the control timeline through ``timestamp``."""

        timestamp = float(timestamp)
        if not math.isfinite(timestamp):
            raise ValueError("Raw Input timestamp must be finite.")
        with self._lock:
            total = 0.0
            while self._events and self._events[0][0] <= timestamp:
                _, dx = self._events.popleft()
                total += dx
            return total

    def take_all(self) -> float:
        """Destructively drain all queued events for legacy-compatible callers."""

        with self._lock:
            total = sum(dx for _, dx in self._events)
            self._events.clear()
            return total

    def clear(self) -> int:
        """Discard pending events (for example while the engine is paused)."""

        with self._lock:
            count = len(self._events)
            self._events.clear()
            return count


# ---------------------------------------------------------------------------
# Exact-width Win32 data structures and packet decoding
# ---------------------------------------------------------------------------


# Use exact-width types rather than ctypes.c_long: Windows LONG is always a
# signed 32-bit value, whereas c_long differs on some non-Windows test hosts.
_WORD = ctypes.c_uint16
_DWORD = ctypes.c_uint32
_LONG = ctypes.c_int32
_UINT = ctypes.c_uint32
_BOOL = ctypes.c_int
_HWND = ctypes.c_void_p
_HINSTANCE = ctypes.c_void_p
_HRAWINPUT = ctypes.c_void_p
_WPARAM = ctypes.c_size_t
_LPARAM = ctypes.c_ssize_t
_LRESULT = ctypes.c_ssize_t

RIM_TYPEMOUSE = 0
RID_INPUT = 0x10000003
RIDEV_REMOVE = 0x00000001
RIDEV_INPUTSINK = 0x00000100
WM_INPUT = 0x00FF
WM_CLOSE = 0x0010
WM_DESTROY = 0x0002
WM_QUIT = 0x0012
MOUSE_MOVE_ABSOLUTE = 0x0001
HWND_MESSAGE = -3
_INVALID_UINT = _UINT(-1).value


class _RAWINPUTDEVICE(ctypes.Structure):
    _fields_ = [
        ("usUsagePage", _WORD),
        ("usUsage", _WORD),
        ("dwFlags", _DWORD),
        ("hwndTarget", _HWND),
    ]


class _RAWINPUTHEADER(ctypes.Structure):
    _fields_ = [
        ("dwType", _DWORD),
        ("dwSize", _DWORD),
        ("hDevice", _HWND),
        ("wParam", _WPARAM),
    ]


class _RAWBUTTONS(ctypes.Union):
    _fields_ = [
        ("ulButtons", _DWORD),
        ("button_words", _WORD * 2),
    ]


class _RAWMOUSE(ctypes.Structure):
    _fields_ = [
        ("usFlags", _WORD),
        ("buttons", _RAWBUTTONS),
        ("ulRawButtons", _DWORD),
        ("lLastX", _LONG),
        ("lLastY", _LONG),
        ("ulExtraInformation", _DWORD),
    ]


class _RAWINPUT(ctypes.Structure):
    """The mouse-sized prefix of a native ``RAWINPUT`` packet.

    The real C structure ends in a union whose HID member is variable-sized.
    We only cast to this 48-byte prefix *after* confirming the header says
    ``RIM_TYPEMOUSE`` and the buffer contains a full ``RAWMOUSE`` payload.
    Non-mouse packets are decoded from their header alone, so no oversized
    ctypes view is ever created over a shorter keyboard/HID buffer.
    """

    _fields_ = [("header", _RAWINPUTHEADER), ("mouse", _RAWMOUSE)]


@dataclass(frozen=True)
class RawMousePacket:
    """Decoded fields relevant to the steering source, independent of Win32 I/O."""

    is_mouse: bool
    is_relative: bool
    dx: int


def decode_raw_mouse_packet(raw_input: _RAWINPUT) -> RawMousePacket:
    """Decode a ``RAWINPUT`` structure without accessing Windows APIs.

    Absolute devices (including common RDP/touch paths) are deliberately not
    reinterpreted as relative steering motion. Step 14's contract is relative
    HID counts only.
    """

    if int(raw_input.header.dwType) != RIM_TYPEMOUSE:
        return RawMousePacket(is_mouse=False, is_relative=False, dx=0)
    mouse = raw_input.mouse
    is_relative = not bool(int(mouse.usFlags) & MOUSE_MOVE_ABSOLUTE)
    return RawMousePacket(
        is_mouse=True,
        is_relative=is_relative,
        dx=int(mouse.lLastX) if is_relative else 0,
    )


# ---------------------------------------------------------------------------
# Lazy Win32 bindings
# ---------------------------------------------------------------------------


_WINFUNCTYPE = getattr(ctypes, "WINFUNCTYPE", ctypes.CFUNCTYPE)
_WNDPROC = _WINFUNCTYPE(_LRESULT, _HWND, _UINT, _WPARAM, _LPARAM)


class _POINT(ctypes.Structure):
    _fields_ = [("x", _LONG), ("y", _LONG)]


class _MSG(ctypes.Structure):
    _fields_ = [
        ("hwnd", _HWND),
        ("message", _UINT),
        ("wParam", _WPARAM),
        ("lParam", _LPARAM),
        ("time", _DWORD),
        ("pt", _POINT),
        ("lPrivate", _DWORD),
    ]


class _WNDCLASSEXW(ctypes.Structure):
    _fields_ = [
        ("cbSize", _UINT),
        ("style", _UINT),
        ("lpfnWndProc", _WNDPROC),
        ("cbClsExtra", ctypes.c_int),
        ("cbWndExtra", ctypes.c_int),
        ("hInstance", _HINSTANCE),
        ("hIcon", _HWND),
        ("hCursor", _HWND),
        ("hbrBackground", _HWND),
        ("lpszMenuName", ctypes.c_wchar_p),
        ("lpszClassName", ctypes.c_wchar_p),
        ("hIconSm", _HWND),
    ]


def _set_signature(function, argtypes, restype) -> None:
    """Assign ctypes metadata while tolerating minimal deterministic fakes."""

    try:
        function.argtypes = argtypes
        function.restype = restype
    except (AttributeError, TypeError):
        pass


class _WinRawInputApi:
    """Late-bound Win32 functions used only by a running Raw Input source."""

    def __init__(self) -> None:
        try:
            windll = ctypes.windll
            self.user32 = windll.user32
            self.kernel32 = windll.kernel32
        except (AttributeError, OSError) as exc:
            raise RawInputUnavailable("Windows user32/kernel32 APIs are unavailable.") from exc

        self.register_raw_input_devices = self.user32.RegisterRawInputDevices
        _set_signature(
            self.register_raw_input_devices,
            [ctypes.POINTER(_RAWINPUTDEVICE), _UINT, _UINT],
            _BOOL,
        )
        self.get_raw_input_data = self.user32.GetRawInputData
        _set_signature(
            self.get_raw_input_data,
            [_HRAWINPUT, _UINT, ctypes.c_void_p, ctypes.POINTER(_UINT), _UINT],
            _UINT,
        )
        self.register_class_ex = self.user32.RegisterClassExW
        _set_signature(self.register_class_ex, [ctypes.POINTER(_WNDCLASSEXW)], _WORD)
        self.unregister_class = self.user32.UnregisterClassW
        _set_signature(self.unregister_class, [ctypes.c_wchar_p, _HINSTANCE], _BOOL)
        self.create_window_ex = self.user32.CreateWindowExW
        _set_signature(
            self.create_window_ex,
            [
                _DWORD,
                ctypes.c_wchar_p,
                ctypes.c_wchar_p,
                _DWORD,
                ctypes.c_int,
                ctypes.c_int,
                ctypes.c_int,
                ctypes.c_int,
                _HWND,
                _HWND,
                _HINSTANCE,
                ctypes.c_void_p,
            ],
            _HWND,
        )
        self.destroy_window = self.user32.DestroyWindow
        _set_signature(self.destroy_window, [_HWND], _BOOL)
        self.def_window_proc = self.user32.DefWindowProcW
        _set_signature(self.def_window_proc, [_HWND, _UINT, _WPARAM, _LPARAM], _LRESULT)
        self.get_message = self.user32.GetMessageW
        _set_signature(self.get_message, [ctypes.POINTER(_MSG), _HWND, _UINT, _UINT], ctypes.c_int)
        self.translate_message = self.user32.TranslateMessage
        _set_signature(self.translate_message, [ctypes.POINTER(_MSG)], _BOOL)
        self.dispatch_message = self.user32.DispatchMessageW
        _set_signature(self.dispatch_message, [ctypes.POINTER(_MSG)], _LRESULT)
        self.post_message = self.user32.PostMessageW
        _set_signature(self.post_message, [_HWND, _UINT, _WPARAM, _LPARAM], _BOOL)
        self.post_thread_message = self.user32.PostThreadMessageW
        _set_signature(self.post_thread_message, [_DWORD, _UINT, _WPARAM, _LPARAM], _BOOL)
        self.post_quit_message = self.user32.PostQuitMessage
        _set_signature(self.post_quit_message, [ctypes.c_int], None)
        self.get_module_handle = self.kernel32.GetModuleHandleW
        _set_signature(self.get_module_handle, [ctypes.c_wchar_p], _HINSTANCE)
        self.get_current_thread_id = self.kernel32.GetCurrentThreadId
        _set_signature(self.get_current_thread_id, [], _DWORD)
        self.get_last_error = self.kernel32.GetLastError
        _set_signature(self.get_last_error, [], _DWORD)

    def error(self, operation: str) -> RawInputError:
        return RawInputError(f"{operation} failed (Win32 error {int(self.get_last_error())}).")

    def read_raw_mouse_packet(self, handle: int) -> RawMousePacket:
        """Read one ``HRAWINPUT`` handle into a safely sized temporary buffer."""

        size = _UINT(0)
        result = int(
            self.get_raw_input_data(
                _HRAWINPUT(handle),
                RID_INPUT,
                None,
                ctypes.byref(size),
                ctypes.sizeof(_RAWINPUTHEADER),
            )
        )
        if result == _INVALID_UINT or size.value == 0:
            raise self.error("GetRawInputData(size)")
        if size.value < ctypes.sizeof(_RAWINPUTHEADER):
            raise RawInputError("GetRawInputData returned a truncated RAWINPUT header.")

        buffer = ctypes.create_string_buffer(size.value)
        copied = int(
            self.get_raw_input_data(
                _HRAWINPUT(handle),
                RID_INPUT,
                ctypes.cast(buffer, ctypes.c_void_p),
                ctypes.byref(size),
                ctypes.sizeof(_RAWINPUTHEADER),
            )
        )
        if copied == _INVALID_UINT:
            raise self.error("GetRawInputData(data)")
        if copied < ctypes.sizeof(_RAWINPUTHEADER):
            raise RawInputError("GetRawInputData returned a truncated RAWINPUT packet.")

        header = ctypes.cast(buffer, ctypes.POINTER(_RAWINPUTHEADER)).contents
        if int(header.dwType) != RIM_TYPEMOUSE:
            return RawMousePacket(is_mouse=False, is_relative=False, dx=0)

        minimum_mouse_size = ctypes.sizeof(_RAWINPUTHEADER) + ctypes.sizeof(_RAWMOUSE)
        if copied < minimum_mouse_size:
            raise RawInputError("GetRawInputData returned a truncated RAWMOUSE packet.")
        raw_input = ctypes.cast(buffer, ctypes.POINTER(_RAWINPUT)).contents
        return decode_raw_mouse_packet(raw_input)


# Win32 permits only one registered target window per top-level collection in a
# process. Do not silently let a second source steal the mouse registration.
_active_source_lock = threading.Lock()
_active_source: "RawInputDeltaSource | None" = None


class RawInputDeltaSource:
    """A Windows-only, timestamped relative-mouse source.

    The public queue methods intentionally match ``RelativeMouseTracker``'s
    Step-12 handoff interface so the bounded control loop can preserve source
    time ownership without knowing whether deltas came from cursor fallback or
    Raw Input.
    """

    backend_name = "raw_input"
    allows_cursor_warp = False
    recentering = False

    def __init__(
        self,
        *,
        queue_capacity: int = DEFAULT_RAW_INPUT_QUEUE_CAPACITY,
        clock: Callable[[], float] | None = None,
    ) -> None:
        self._queue = TimestampedDeltaQueue(queue_capacity)
        self._clock = clock or time.perf_counter
        self._state_lock = threading.RLock()
        self._paused = False
        self._status = "stopped"
        self._last_error: str | None = None
        self._packet_error_count = 0
        self._ignored_absolute_event_count = 0
        self._ignored_non_mouse_event_count = 0
        self._paused_discarded_event_count = 0
        self._received_event_count = 0
        self._api: _WinRawInputApi | None = None
        self._thread: threading.Thread | None = None
        self._thread_id = 0
        self._window = None
        self._window_class_name: str | None = None
        self._window_class_atom = 0
        self._wndproc = None  # Strong reference required while native code calls it.
        self._ready = threading.Event()
        self._stopped = threading.Event()
        self._stopped.set()
        self._stop_event = threading.Event()
        self._startup_error: RawInputError | None = None

    # ----- queue-compatible input interface ---------------------------
    @property
    def pending_event_count(self) -> int:
        return self._queue.pending_event_count

    @property
    def dropped_event_count(self) -> int:
        return self._queue.dropped_event_count

    @property
    def queue_capacity(self) -> int:
        return self._queue.capacity

    @property
    def paused(self) -> bool:
        with self._state_lock:
            return self._paused

    @property
    def status(self) -> str:
        with self._state_lock:
            return self._status

    @property
    def last_error(self) -> str | None:
        with self._state_lock:
            return self._last_error

    @property
    def packet_error_count(self) -> int:
        with self._state_lock:
            return self._packet_error_count

    @property
    def received_event_count(self) -> int:
        with self._state_lock:
            return self._received_event_count

    @property
    def ignored_absolute_event_count(self) -> int:
        with self._state_lock:
            return self._ignored_absolute_event_count

    @property
    def ignored_non_mouse_event_count(self) -> int:
        with self._state_lock:
            return self._ignored_non_mouse_event_count

    @property
    def paused_discarded_event_count(self) -> int:
        with self._state_lock:
            return self._paused_discarded_event_count

    def take_through(self, timestamp: float) -> float:
        return self._queue.take_through(timestamp)

    def take_dx(self) -> float:
        """Legacy destructive drain for narrow third-party engine shims."""

        return self._queue.take_all()

    def set_paused(self, paused: bool) -> None:
        with self._state_lock:
            self._paused = bool(paused)
            if self._paused:
                self._paused_discarded_event_count += self._queue.clear()

    def enqueue_relative_delta(
        self, dx: float, *, timestamp: float | None = None
    ) -> bool:
        """Queue one relative HID count delta (also used by deterministic tests)."""

        if timestamp is None:
            timestamp = self._clock()
        with self._state_lock:
            if self._paused:
                if float(dx) != 0.0:
                    self._paused_discarded_event_count += 1
                return False
            accepted = self._queue.put(timestamp, dx)
            if accepted:
                self._received_event_count += 1
            return accepted

    def process_packet(self, packet: RawMousePacket, *, timestamp: float | None = None) -> bool:
        """Apply the Step-14 relative-only policy to a decoded packet."""

        if not packet.is_mouse:
            with self._state_lock:
                self._ignored_non_mouse_event_count += 1
            return False
        if not packet.is_relative:
            with self._state_lock:
                self._ignored_absolute_event_count += 1
            return False
        return self.enqueue_relative_delta(packet.dx, timestamp=timestamp)

    # ----- lifecycle ---------------------------------------------------
    @property
    def running(self) -> bool:
        thread = self._thread
        return thread is not None and thread.is_alive()

    def start(self, timeout: float = 3.0) -> None:
        """Create the message-only window and begin receiving ``WM_INPUT``."""

        if sys.platform != "win32":
            raise RawInputUnavailable("Windows Raw Input is available only on Windows.")
        if timeout <= 0.0:
            raise ValueError("Raw Input startup timeout must be positive.")

        with self._state_lock:
            if self.running:
                return
            self._reserve_process_slot()
            self._ready.clear()
            self._stopped.clear()
            self._stop_event.clear()
            self._startup_error = None
            self._last_error = None
            self._status = "starting"
            self._thread = threading.Thread(
                target=self._run_message_loop,
                daemon=True,
                name="raw-input-message-loop",
            )
            try:
                self._thread.start()
            except Exception:
                self._thread = None
                self._status = "faulted"
                self._release_process_slot()
                raise

        if not self._ready.wait(timeout):
            self.stop(timeout=min(timeout, 1.0))
            raise RawInputUnavailable("Timed out while creating the Raw Input message window.")

        with self._state_lock:
            startup_error = self._startup_error
        if startup_error is not None:
            self.stop(timeout=min(timeout, 1.0))
            raise startup_error

    def stop(self, timeout: float = 2.0) -> None:
        """Request message-loop shutdown and wait boundedly when safe."""

        with self._state_lock:
            thread = self._thread
            api = self._api
            window = self._window
            thread_id = self._thread_id
            self._stop_event.set()

        if api is not None:
            # WM_CLOSE gives the window procedure a normal destruction path;
            # WM_QUIT is a fallback for a window that was not fully created.
            posted = False
            if window:
                try:
                    posted = bool(api.post_message(window, WM_CLOSE, 0, 0))
                except Exception:
                    posted = False
            if not posted and thread_id:
                try:
                    api.post_thread_message(thread_id, WM_QUIT, 0, 0)
                except Exception:
                    pass

        if (
            thread is not None
            and thread is not threading.current_thread()
            and timeout > 0.0
        ):
            thread.join(timeout)

    # ----- native thread ----------------------------------------------
    def _reserve_process_slot(self) -> None:
        global _active_source
        with _active_source_lock:
            if _active_source is not None and _active_source is not self:
                raise RawInputUnavailable(
                    "Another Raw Input mouse source already owns this process."
                )
            _active_source = self

    def _release_process_slot(self) -> None:
        global _active_source
        with _active_source_lock:
            if _active_source is self:
                _active_source = None

    def _record_packet_error(self, exc: Exception) -> None:
        with self._state_lock:
            self._packet_error_count += 1
            self._last_error = f"Raw Input packet ignored: {type(exc).__name__}: {exc}"

    def _handle_wm_input(self, handle: int) -> None:
        api = self._api
        if api is None:
            return
        try:
            packet = api.read_raw_mouse_packet(handle)
            self.process_packet(packet)
        except Exception as exc:
            # One malformed/failed native packet must not kill the message loop
            # and strand an otherwise running steering session.
            self._record_packet_error(exc)

    def _register_raw_mouse(self, *, remove: bool = False) -> None:
        api = self._api
        if api is None:
            raise RawInputUnavailable("Raw Input Win32 bindings were not initialised.")
        device = _RAWINPUTDEVICE(
            usUsagePage=0x01,  # HID_USAGE_PAGE_GENERIC
            usUsage=0x02,  # HID_USAGE_GENERIC_MOUSE
            dwFlags=RIDEV_REMOVE if remove else RIDEV_INPUTSINK,
            hwndTarget=None if remove else self._window,
        )
        if not bool(
            api.register_raw_input_devices(
                ctypes.byref(device), 1, ctypes.sizeof(_RAWINPUTDEVICE)
            )
        ):
            raise api.error("RegisterRawInputDevices")

    def _run_message_loop(self) -> None:
        api: _WinRawInputApi | None = None
        try:
            api = _WinRawInputApi()
            with self._state_lock:
                self._api = api
                self._thread_id = int(api.get_current_thread_id())

            # Include a per-start nonce so a best-effort UnregisterClassW
            # failure cannot prevent this source object from being restarted.
            class_name = (
                f"MouseSteeringRawInput_{id(self):x}_{time.perf_counter_ns():x}"
            )

            @_WNDPROC
            def window_proc(hwnd, message, wparam, lparam):
                try:
                    if message == WM_INPUT:
                        self._handle_wm_input(int(lparam))
                        # Per WM_INPUT's foreground contract, route to the
                        # default procedure after reading so Windows can clean
                        # up the packet. It is harmless for RIM_INPUTSINK too.
                        return api.def_window_proc(hwnd, message, wparam, lparam)
                    if message == WM_CLOSE:
                        api.destroy_window(hwnd)
                        return 0
                    if message == WM_DESTROY:
                        api.post_quit_message(0)
                        return 0
                    return api.def_window_proc(hwnd, message, wparam, lparam)
                except Exception as exc:  # never unwind through a Win32 callback
                    self._record_packet_error(exc)
                    try:
                        return api.def_window_proc(hwnd, message, wparam, lparam)
                    except Exception:
                        return 0

            hinstance = api.get_module_handle(None)
            window_class = _WNDCLASSEXW()
            window_class.cbSize = ctypes.sizeof(_WNDCLASSEXW)
            window_class.lpfnWndProc = window_proc
            window_class.hInstance = hinstance
            window_class.lpszClassName = class_name
            atom = int(api.register_class_ex(ctypes.byref(window_class)))
            if atom == 0:
                raise api.error("RegisterClassExW")

            window = api.create_window_ex(
                0,
                class_name,
                class_name,
                0,
                0,
                0,
                0,
                0,
                _HWND(HWND_MESSAGE),
                None,
                hinstance,
                None,
            )
            if not window:
                raise api.error("CreateWindowExW(HWND_MESSAGE)")

            with self._state_lock:
                self._window_class_name = class_name
                self._window_class_atom = atom
                self._wndproc = window_proc
                self._window = window

            self._register_raw_mouse()
            with self._state_lock:
                self._status = "running"
            self._ready.set()

            message = _MSG()
            while not self._stop_event.is_set():
                result = int(api.get_message(ctypes.byref(message), None, 0, 0))
                if result == -1:
                    raise api.error("GetMessageW")
                if result == 0:
                    break
                api.translate_message(ctypes.byref(message))
                api.dispatch_message(ctypes.byref(message))

        except Exception as exc:
            error = exc if isinstance(exc, RawInputError) else RawInputError(
                f"Raw Input message loop failed: {type(exc).__name__}: {exc}"
            )
            with self._state_lock:
                self._startup_error = error
                self._last_error = str(error)
                self._status = "faulted"
            self._ready.set()
        finally:
            # Unregister before dropping/destroying the target window. Cleanup
            # is best effort because a partial native startup may not have a
            # valid registration/window/class yet.
            if api is not None and self._window:
                try:
                    self._register_raw_mouse(remove=True)
                except Exception:
                    pass
                try:
                    api.destroy_window(self._window)
                except Exception:
                    pass
            if api is not None and self._window_class_name:
                try:
                    api.unregister_class(
                        self._window_class_name,
                        api.get_module_handle(None),
                    )
                except Exception:
                    pass

            with self._state_lock:
                if self._status != "faulted":
                    self._status = "stopped"
                self._window = None
                self._window_class_name = None
                self._window_class_atom = 0
                self._wndproc = None
                self._thread_id = 0
                self._api = None
            self._release_process_slot()
            self._stopped.set()
