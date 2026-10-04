#!/usr/bin/env python3
"""
mouse_steering.py -- analog steering from a mouse, through a virtual Xbox 360
controller, with a tuning GUI.

Fixed version:
- lazy-loads vgamepad so --help works without ViGEm installed;
- uses the current vgamepad VX360Gamepad lifecycle (destroying the object
  disconnects it; there is no gamepad.delete() call);
- makes the Windows process DPI-aware before input/GUI setup;
- defaults to a Windows Raw Input relative-mouse source without cursor warps;
- gives every run an explicit session ID and lifecycle state, so stale workers
  cannot mutate a replacement run and stop() reports confirmed/pending cleanup;
- gives the control loop a fail-safe exception boundary plus owner-side fault
  cleanup, without claiming forced-process termination is safe;
- makes cursor-warp handling explicit and prevents synthetic warp events from
  being counted as steering input;
- describes its own GUI through a Tk-free contract: unit-bearing labels,
  persistent input/mode/profile/safety context, a Diagnostics tab that explains
  why steering is centring or ignoring input and whether the pad is connected,
  a safe-live-apply policy for settings that would otherwise retarget the sent
  axis abruptly, and a window that stays open until cleanup is confirmed;
- adds operational safety controls: configurable pause/stop hotkeys, an optional
  hold-to-enable deadman key, an opt-in foreground-window allowlist that
  auto-pauses steering when an unlisted window has focus, and a highly visible
  indicator banner that says whether the virtual pad is actively steering,
  held by a guard, or idle. These controls are opt-in and exist so a driver can
  kill or gate mouse-to-pad steering on purpose; none of them hide from, defeat,
  or bypass any anti-cheat, and each game's and platform's rules still apply
  (see docs/GAME_POLICY.md);
- validates/sanitises JSON presets;
- parses launcher options with argparse, including explicit profile/input-mode
  choices and dependency-free help; and
- reports runtime failures to the GUI/CLI instead of silently leaving a false
  "running" state.

Requirements (Windows -- ViGEm is Windows-only)
------------------------------------------------
    pip install vgamepad pynput

Usage
-----
    python src/mouse_steering.py                              # tuning GUI
    python src/mouse_steering.py --cli                        # console-only mode
    python src/mouse_steering.py --profile profiles/rally.json
    python src/mouse_steering.py --input-mode raw_input
    python src/mouse_steering.py --input-mode cursor_fallback # degraded mode
    python src/mouse_steering.py --log-level info
    python src/mouse_steering.py --dpi-diagnostics            # support output
    python src/mouse_steering.py --help

``--raw-input`` and ``--cursor-fallback`` remain compatibility aliases for the
corresponding ``--input-mode`` values.

F8  pause/resume   (configurable via hotkey_pause)
F9  emergency stop  (configurable via hotkey_stop)
Ctrl+C  quit in CLI mode

On validated Windows targets, the default path is Windows Raw Input relative
HID counts and never repositions the system cursor. ``--cursor-fallback``
selects the retained, visibly DEGRADED pynput cursor-coordinate compatibility
mode; ``--raw-input`` is accepted as an explicit spelling of the default.
Cursor fallback consumes only a fresh coordinate-specific pending warp target,
never a generic next callback.
"""

from __future__ import annotations

import argparse
import atexit
import ctypes
import logging
import math
import signal
import sys
import tempfile
import threading
import time
import traceback
import weakref
from collections import deque
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any, Callable

# Phase 3 / Step 18 writes concise exception/traceback records only when a
# runtime failure occurs.  A temp-directory location avoids a writable-source
# tree assumption for installed applications; its concrete path is surfaced in
# telemetry and CLI/GUI guidance.
DIAGNOSTIC_LOG_PATH = Path(tempfile.gettempdir()) / "mouse_steering_diagnostics.log"
_diagnostic_log_lock = threading.Lock()

# Phase 3 / Step 17 process-exit safety net.  The registry is weak so merely
# importing this module or constructing a short-lived engine cannot keep it
# alive.  These handlers are intentionally best effort: Python-level cleanup
# cannot make a guarantee after a forced process termination.
_BEST_EFFORT_STOP_TIMEOUT_S = 0.5
_active_engines: weakref.WeakSet = weakref.WeakSet()
_active_engines_lock = threading.RLock()
_signal_handlers: dict[int, Any] = {}
_signal_handlers_installed = False
_signal_cleanup_in_progress = False


def _register_active_engine(engine: Any) -> None:
    with _active_engines_lock:
        _active_engines.add(engine)


def _best_effort_shutdown_all(timeout: float = _BEST_EFFORT_STOP_TIMEOUT_S) -> None:
    """Request bounded cleanup for live engines during normal interpreter exit.

    Exceptions are deliberately swallowed here: this is a last-chance cleanup
    hook, not a claim that a process-exit path has confirmed device release.
    Ordinary CLI/GUI shutdown still reports the structured result from stop().
    """
    with _active_engines_lock:
        engines = tuple(_active_engines)
    for engine in engines:
        try:
            engine.stop(timeout=timeout)
        except BaseException:
            pass


def _best_effort_signal_handler(signum, frame) -> None:  # pragma: no cover - OS signal timing
    """Run bounded cleanup, then preserve the previously installed signal policy."""
    global _signal_cleanup_in_progress
    if not _signal_cleanup_in_progress:
        _signal_cleanup_in_progress = True
        try:
            _best_effort_shutdown_all()
        finally:
            _signal_cleanup_in_progress = False

    previous = _signal_handlers.get(signum, signal.SIG_DFL)
    if previous in (None, signal.SIG_IGN, _best_effort_signal_handler):
        return
    if previous is signal.SIG_DFL:
        if signum == getattr(signal, "SIGINT", None):
            raise KeyboardInterrupt
        raise SystemExit(128 + int(signum))
    previous(signum, frame)


def install_best_effort_shutdown_handlers() -> None:
    """Install normal Python-exit/SIGINT/SIGTERM cleanup once when safe.

    Signal registration is only legal in the main thread.  We keep the prior
    handler and chain to it after requesting a bounded stop, so Ctrl+C retains
    its usual KeyboardInterrupt behavior and host applications keep custom
    signal policy.
    """
    global _signal_handlers_installed
    if _signal_handlers_installed or threading.current_thread() is not threading.main_thread():
        return
    for name in ("SIGINT", "SIGTERM"):
        signum = getattr(signal, name, None)
        if signum is None:
            continue
        try:
            _signal_handlers[signum] = signal.getsignal(signum)
            signal.signal(signum, _best_effort_signal_handler)
        except (OSError, RuntimeError, ValueError):
            # Sandboxed/embedded interpreters may reject handler changes; the
            # atexit hook still remains available where normal exit occurs.
            continue
    _signal_handlers_installed = True


atexit.register(_best_effort_shutdown_all)


# Support both direct script execution (python src/mouse_steering.py) and
# package-style imports used by tests/tools.
if __package__:
    from .steering_core import (
        ACCEL_V_REF,
        CENTER_FLOOR,
        CENTERING_SEMANTICS_AXIS_TARGET,
        CURVE_PRESETS,
        FILTER_SEMANTICS_V2,
        LEGACY_FILTER_REFERENCE_HZ,
        INPUT_MODE_RAW_INPUT,
        MAX_CONTROL_INTEGRATION_S,
        MAX_LATE_AXIS_STEP,
        PRESET_PATH,
        PRESET_SCHEMA_V1,
        PRESET_SCHEMA_V2,
        PRESET_STATUS_CORRUPT,
        PRESET_STATUS_IO_ERROR,
        PRESET_STATUS_LOADED,
        PRESET_STATUS_MIGRATED,
        PRESET_STATUS_MISSING,
        PRESET_STATUS_UNSUPPORTED_VERSION,
        PRESET_STATUS_VALIDATION_ERROR,
        PROFILE_SEMANTICS_V2,
        PresetLoadResult,
        PresetSaveResult,
        PRECISION_BAND,
        RAIL_EPS,
        STEERING_MODES,
        Settings,
        SteerState,
        centering_factor,
        clamp,
        format_preset_load_result,
        format_status,
        load_preset,
        load_preset_result,
        migrate_preset,
        new_v2_settings,
        precision_factor,
        plan_control_interval,
        remap_output_deadzone,
        resolve_steering_mode,
        response_curve,
        sanitise_settings,
        save_preset,
        smoothstep,
        step_steering,
        to_axis,
        travel_px,
    )
else:
    from steering_core import (
        ACCEL_V_REF,
        CENTER_FLOOR,
        CENTERING_SEMANTICS_AXIS_TARGET,
        CURVE_PRESETS,
        FILTER_SEMANTICS_V2,
        LEGACY_FILTER_REFERENCE_HZ,
        INPUT_MODE_RAW_INPUT,
        MAX_CONTROL_INTEGRATION_S,
        MAX_LATE_AXIS_STEP,
        PRESET_PATH,
        PRESET_SCHEMA_V1,
        PRESET_SCHEMA_V2,
        PRESET_STATUS_CORRUPT,
        PRESET_STATUS_IO_ERROR,
        PRESET_STATUS_LOADED,
        PRESET_STATUS_MIGRATED,
        PRESET_STATUS_MISSING,
        PRESET_STATUS_UNSUPPORTED_VERSION,
        PRESET_STATUS_VALIDATION_ERROR,
        PROFILE_SEMANTICS_V2,
        PresetLoadResult,
        PresetSaveResult,
        PRECISION_BAND,
        RAIL_EPS,
        STEERING_MODES,
        Settings,
        SteerState,
        centering_factor,
        clamp,
        format_preset_load_result,
        format_status,
        load_preset,
        load_preset_result,
        migrate_preset,
        new_v2_settings,
        precision_factor,
        plan_control_interval,
        remap_output_deadzone,
        resolve_steering_mode,
        response_curve,
        sanitise_settings,
        save_preset,
        smoothstep,
        step_steering,
        to_axis,
        travel_px,
    )

if __package__:
    from . import safety as safety_contract
else:
    import safety as safety_contract

# Phase 4 / Step 23: the calibration flow and the layered profile workflow. The
# module is pure (no Tk, no Win32, no pynput), so importing it costs nothing and
# changes no platform behaviour.
if __package__:
    from . import calibration as calibration_contract
else:
    import calibration as calibration_contract


# ===========================================================================
# PLATFORM / DEPENDENCIES
# ===========================================================================


# Windows DPI-awareness constants. Per-Monitor V2 is represented by the
# documented negative DPI_AWARENESS_CONTEXT handle, while the older shcore API
# uses PROCESS_PER_MONITOR_DPI_AWARE = 2.
DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2 = -4
PROCESS_PER_MONITOR_DPI_AWARE = 2
_ERROR_ACCESS_DENIED = 5
_HRESULT_E_ACCESSDENIED = 0x80070005

# Runtime selections deliberately stay outside schema-v1 settings. Windows
# Raw Input is the accepted default; Step 15 retains the legacy path only as an
# explicit degraded fallback, and Step 19 owns persisted input_mode.
INPUT_BACKEND_CURSOR_FALLBACK = "cursor_fallback"
INPUT_BACKEND_RAW_INPUT = "raw_input"
INPUT_BACKENDS = (INPUT_BACKEND_CURSOR_FALLBACK, INPUT_BACKEND_RAW_INPUT)

# Step 20 keeps logging deliberately local to this application rather than
# reconfiguring a host application's root logger. The option is useful to
# launchers because it controls genuine startup/profile diagnostic records;
# it does not claim that ordinary GUI/CLI status text is a logging API.
LOG_LEVEL_CHOICES = ("debug", "info", "warning", "error", "critical")
_LOGGER = logging.getLogger("mouse_steering")
_LOGGER_HANDLER: logging.Handler | None = None


def configure_log_level(level: str) -> str:
    """Configure the application's stderr logger and return its canonical level.

    This function has no Windows, Tk, input-listener, or gamepad dependency so
    parser/help paths remain safe. A dedicated handler avoids changing a
    launcher's root logging policy and is installed at most once.
    """
    if not isinstance(level, str):
        raise ValueError("log level must be a string")
    canonical = level.lower()
    if canonical not in LOG_LEVEL_CHOICES:
        raise ValueError(
            "log level must be one of: " + ", ".join(LOG_LEVEL_CHOICES)
        )
    global _LOGGER_HANDLER
    if _LOGGER_HANDLER is None:
        _LOGGER_HANDLER = logging.StreamHandler()
        _LOGGER_HANDLER.setFormatter(
            logging.Formatter("%(levelname)s mouse_steering: %(message)s")
        )
        _LOGGER.addHandler(_LOGGER_HANDLER)
        _LOGGER.propagate = False
    else:
        # Tests, embedded launchers, and GUI redirects can replace sys.stderr
        # between launches; keep this application-local handler truthful.
        _LOGGER_HANDLER.setStream(sys.stderr)
    _LOGGER.setLevel(getattr(logging, canonical.upper()))
    return canonical


# Cursor fallback is deliberately retained only as an explicit degraded
# compatibility backend after Raw Input acceptance. A short, target-based
# pending state replaces the old unsafe "ignore the next arbitrary callback"
# mechanism. The TTL limits how long a coordinate may be recognized as our
# synthetic recenter event before it is treated as ordinary physical input.
CURSOR_WARP_PENDING_TTL_S = 0.250
CURSOR_WARP_TARGET_TOLERANCE_PX = 1.0


@dataclass
class _PendingCursorWarp:
    token: int
    target: tuple[float, float]
    expires_at: float
    saw_real_callback: bool = False


@dataclass(frozen=True)
class DpiAwarenessStatus:
    """Result of the one-time process DPI-awareness negotiation."""

    mode: str
    detail: str
    configured: bool

    @property
    def summary(self) -> str:
        labels = {
            "not_attempted": "not attempted",
            "not_applicable": "not applicable (non-Windows)",
            "per_monitor_v2": "Per-Monitor V2",
            "per_monitor": "Per-Monitor",
            "system_aware": "System-aware",
            "already_configured": "already configured by host/manifest",
            "unaware": "unaware (no supported API succeeded)",
        }
        label = labels.get(self.mode, self.mode)
        return f"{label}: {self.detail}" if self.detail else label


_dpi_status_lock = threading.Lock()
_dpi_status: DpiAwarenessStatus | None = None


def _set_ctypes_signature(function, argtypes, restype) -> None:
    """Set ctypes metadata when this is a real ctypes function, not a test fake."""
    try:
        function.argtypes = argtypes
        function.restype = restype
    except (AttributeError, TypeError):
        pass


def _clear_last_error() -> None:
    # ctypes' private last-error slot is not guaranteed to mirror calls loaded
    # through ``windll``. Clear the real Windows thread-local value as well.
    try:
        set_last_error = ctypes.windll.kernel32.SetLastError
        _set_ctypes_signature(set_last_error, [ctypes.c_ulong], None)
        set_last_error(0)
    except (AttributeError, OSError):
        pass
    try:
        ctypes.set_last_error(0)
    except (AttributeError, OSError):
        pass


def _last_error() -> int:
    try:
        get_last_error = ctypes.windll.kernel32.GetLastError
        _set_ctypes_signature(get_last_error, [], ctypes.c_ulong)
        return int(get_last_error())
    except (AttributeError, OSError):
        pass
    try:
        return int(ctypes.get_last_error())
    except (AttributeError, OSError):
        return 0


def _configure_windows_dpi_awareness() -> DpiAwarenessStatus:
    """Negotiate the newest supported Windows process DPI-awareness API.

    The first successful process-wide call wins. ERROR_ACCESS_DENIED commonly
    means an application manifest or host already selected a process mode; it
    is reported honestly rather than treated as a successful mode we chose.
    """

    attempts: list[str] = []

    # Windows 10 1703+: DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2.
    try:
        user32 = ctypes.windll.user32
        set_context = user32.SetProcessDpiAwarenessContext
        _set_ctypes_signature(set_context, [ctypes.c_void_p], ctypes.c_bool)
        _clear_last_error()
        if bool(
            set_context(
                ctypes.c_void_p(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2)
            )
        ):
            return DpiAwarenessStatus(
                "per_monitor_v2",
                "SetProcessDpiAwarenessContext(PER_MONITOR_AWARE_V2)",
                True,
            )
        error = _last_error()
        if error == _ERROR_ACCESS_DENIED:
            return DpiAwarenessStatus(
                "already_configured",
                "Per-Monitor V2 request was denied because process awareness "
                "was already selected",
                True,
            )
        attempts.append(f"Per-Monitor V2 failed (Win32 error {error})")
    except (AttributeError, OSError) as exc:
        attempts.append(f"Per-Monitor V2 unavailable ({type(exc).__name__})")
    except Exception as exc:  # pragma: no cover - defensive platform boundary
        attempts.append(f"Per-Monitor V2 raised {type(exc).__name__}")

    # Windows 8.1+: shcore PROCESS_PER_MONITOR_DPI_AWARE.
    try:
        shcore = ctypes.windll.shcore
        set_awareness = shcore.SetProcessDpiAwareness
        _set_ctypes_signature(set_awareness, [ctypes.c_int], ctypes.c_long)
        result = int(set_awareness(PROCESS_PER_MONITOR_DPI_AWARE))
        hresult = result & 0xFFFFFFFF
        if hresult == 0:
            return DpiAwarenessStatus(
                "per_monitor",
                "SetProcessDpiAwareness(PROCESS_PER_MONITOR_DPI_AWARE)",
                True,
            )
        if hresult == _HRESULT_E_ACCESSDENIED:
            return DpiAwarenessStatus(
                "already_configured",
                "Per-Monitor request was denied because process awareness was "
                "already selected",
                True,
            )
        attempts.append(f"Per-Monitor failed (HRESULT 0x{hresult:08X})")
    except (AttributeError, OSError) as exc:
        attempts.append(f"Per-Monitor unavailable ({type(exc).__name__})")
    except Exception as exc:  # pragma: no cover - defensive platform boundary
        attempts.append(f"Per-Monitor raised {type(exc).__name__}")

    # Vista+: last-resort system-DPI-aware mode.
    try:
        user32 = ctypes.windll.user32
        set_system_aware = user32.SetProcessDPIAware
        _set_ctypes_signature(set_system_aware, [], ctypes.c_bool)
        _clear_last_error()
        if bool(set_system_aware()):
            return DpiAwarenessStatus(
                "system_aware",
                "SetProcessDPIAware() legacy fallback",
                True,
            )
        error = _last_error()
        if error == _ERROR_ACCESS_DENIED:
            return DpiAwarenessStatus(
                "already_configured",
                "System-DPI request was denied because process awareness was "
                "already selected",
                True,
            )
        attempts.append(f"System-aware fallback failed (Win32 error {error})")
    except (AttributeError, OSError) as exc:
        attempts.append(f"System-aware fallback unavailable ({type(exc).__name__})")
    except Exception as exc:  # pragma: no cover - defensive platform boundary
        attempts.append(f"System-aware fallback raised {type(exc).__name__}")

    return DpiAwarenessStatus(
        "unaware",
        "; ".join(attempts) or "No supported DPI-awareness API was available",
        False,
    )


def configure_dpi_awareness() -> DpiAwarenessStatus:
    """Configure process DPI awareness once, before Tk or input backends load."""
    global _dpi_status
    with _dpi_status_lock:
        if _dpi_status is not None:
            return _dpi_status
        if sys.platform != "win32":
            _dpi_status = DpiAwarenessStatus(
                "not_applicable",
                "DPI process awareness is Windows-only",
                False,
            )
        else:
            _dpi_status = _configure_windows_dpi_awareness()
        return _dpi_status


def current_dpi_awareness() -> DpiAwarenessStatus:
    """Return the known status without causing a late platform-side effect."""
    with _dpi_status_lock:
        return _dpi_status or DpiAwarenessStatus(
            "not_attempted",
            "Call configure_dpi_awareness() before Tk or pynput initialization",
            False,
        )


def make_dpi_aware() -> DpiAwarenessStatus:
    """Backward-compatible name for the explicit DPI configuration boundary."""
    return configure_dpi_awareness()


def run_dpi_diagnostics() -> None:
    """Print a no-gamepad diagnostic snapshot for Windows DPI validation.

    This deliberately performs no pynput/vgamepad import. On Windows it creates
    a short-lived hidden Tk root *after* the process configuration to expose the
    coordinate-space values a target-PC tester needs to compare.
    """

    status = configure_dpi_awareness()
    print("=" * 62)
    print("  Mouse Steering — DPI diagnostics")
    print("=" * 62)
    print(f"  awareness mode : {status.mode}")
    print(f"  configured     : {'yes' if status.configured else 'no'}")
    print(f"  detail         : {status.detail}")

    if sys.platform != "win32":
        print("  platform       : non-Windows; Windows metrics unavailable")
        print("=" * 62)
        return

    try:
        user32 = ctypes.windll.user32
        get_metric = user32.GetSystemMetrics
        _set_ctypes_signature(get_metric, [ctypes.c_int], ctypes.c_int)
        metrics = {
            "primary width": 0,
            "primary height": 1,
            "virtual x": 76,
            "virtual y": 77,
            "virtual width": 78,
            "virtual height": 79,
        }
        values = {name: int(get_metric(code)) for name, code in metrics.items()}
        print(
            "  virtual desktop: "
            f"x={values['virtual x']} y={values['virtual y']} "
            f"{values['virtual width']}×{values['virtual height']} px"
        )
        print(
            "  primary desktop: "
            f"{values['primary width']}×{values['primary height']} px"
        )

        try:
            get_dpi_for_system = user32.GetDpiForSystem
            _set_ctypes_signature(get_dpi_for_system, [], ctypes.c_uint)
            print(f"  system DPI     : {int(get_dpi_for_system())}")
        except (AttributeError, OSError):
            print("  system DPI     : API unavailable")
    except Exception as exc:  # pragma: no cover - target-PC platform boundary
        print(f"  Windows metrics: unavailable ({type(exc).__name__}: {exc})")

    root = None
    try:
        import tkinter as tk

        root = tk.Tk()
        root.withdraw()
        tk_scaling = float(root.tk.call("tk", "scaling"))
        print(
            "  Tk screen      : "
            f"{root.winfo_screenwidth()}×{root.winfo_screenheight()} px; "
            f"scaling={tk_scaling:.4f}; fpixels/in={root.winfo_fpixels('1i'):.2f}"
        )

        try:
            get_dpi_for_window = ctypes.windll.user32.GetDpiForWindow
            _set_ctypes_signature(
                get_dpi_for_window, [ctypes.c_void_p], ctypes.c_uint
            )
            print(f"  Tk window DPI  : {int(get_dpi_for_window(root.winfo_id()))}")
        except (AttributeError, OSError):
            print("  Tk window DPI  : API unavailable")
    except Exception as exc:  # pragma: no cover - target-PC platform boundary
        print(f"  Tk diagnostics : unavailable ({type(exc).__name__}: {exc})")
    finally:
        if root is not None:
            try:
                root.destroy()
            except Exception:
                pass

    print("=" * 62)


def import_mouse_keyboard():
    try:
        from pynput import keyboard, mouse
        return keyboard, mouse
    except ImportError as exc:  # pragma: no cover
        raise RuntimeError(
            "Missing dependency 'pynput'.\n"
            "    pip install vgamepad pynput"
        ) from exc


def import_keyboard():
    """Lazy keyboard-only import used by the Raw Input backend's hotkeys."""
    try:
        from pynput import keyboard
        return keyboard
    except ImportError as exc:  # pragma: no cover
        raise RuntimeError(
            "Missing dependency 'pynput'.\n"
            "    pip install vgamepad pynput"
        ) from exc


def create_raw_input_source(*, queue_capacity: int | None = None):
    """Lazily import the Windows-only source only when Raw Input is selected."""
    if __package__:
        from .raw_input import RawInputDeltaSource
    else:
        from raw_input import RawInputDeltaSource

    kwargs = {} if queue_capacity is None else {"queue_capacity": queue_capacity}
    return RawInputDeltaSource(**kwargs)


def import_vgamepad():
    try:
        import vgamepad
        return vgamepad
    except ImportError as exc:  # pragma: no cover
        raise RuntimeError(
            "Missing dependency 'vgamepad'.\n"
            "    pip install vgamepad pynput\n"
            "Installing vgamepad also installs the ViGEmBus driver; accept "
            "the driver installer licence."
        ) from exc
    except Exception as exc:  # pragma: no cover
        raise RuntimeError(
            "Could not initialise vgamepad/ViGEmBus.\n"
            "Make sure the ViGEmBus driver is installed and working, then "
            "retry.\n"
            f"Underlying error: {exc}"
        ) from exc


# ===========================================================================
# STEP 23 -- CALIBRATION RUNNER (INPUT ONLY, NEVER A VIRTUAL PAD)
# ===========================================================================
#
# The calibration flow measures the *mouse*. It deliberately owns no gamepad:
# the source-creation path below builds only an input source and a listener, so
# "calibration cannot steer anything" is structural rather than a promise. The
# flow also refuses to start while a steering session is running, because the
# same movement would be steering the pad at the same time.
#
# The class is injectable in three places -- ``source_factory``,
# ``sample_source``, and ``output_probe`` -- which is what lets the whole flow,
# including its refusals, be driven deterministically on a machine with no
# display, no mouse, and no Windows.


class CalibrationRunner:
    """Drive :class:`calibration.CalibrationSession` against a live input source."""

    def __init__(
        self,
        settings: Settings,
        *,
        input_backend: str = INPUT_BACKEND_RAW_INPUT,
        session: Any = None,
        source_factory: Callable[[], Any] | None = None,
        sample_source: Callable[[float], float] | None = None,
        output_probe: Callable[[], tuple[float, float] | None] | None = None,
        allow_running_session: bool = False,
        clock: Callable[[], float] = time.perf_counter,
        lifecycle_provider: Callable[[], str] | None = None,
        poll_hz: float = 60.0,
    ) -> None:
        if input_backend not in INPUT_BACKENDS:
            raise ValueError(f"Unknown input backend: {input_backend!r}")
        self._settings = sanitise_settings(settings)
        self._input_backend = input_backend
        self._session = session
        self._source_factory = source_factory
        self._sample_source = sample_source
        self._output_probe = output_probe
        self._allow_running_session = bool(allow_running_session)
        self._clock = clock
        self._poll_hz = max(min(poll_hz, 240.0), 5.0)
        self._lifecycle_provider = lifecycle_provider
        self._tracker: Any = None
        self._listener: Any = None
        self._mouse: Any = None
        self._started = False
        self._output_samples: list[calibration_contract.OutputSample] = []
        self._session_state: calibration_contract.CalibrationSession = (
            calibration_contract.CalibrationSession(self._settings)
        )

    # ----- lifecycle -----------------------------------------------------
    @property
    def calibration(self) -> Any:
        return self._session_state

    @property
    def lifecycle_state(self) -> str:
        if self._session is not None:
            return str(self._session.state)
        if self._lifecycle_provider is not None:
            return str(self._lifecycle_provider() or RUNTIME_STOPPED)
        return RUNTIME_STOPPED

    def precondition(self) -> calibration_contract.CalibrationGuard:
        """Whether this run may start.

        The default refuses while a session is driving the pad. A caller that has
        taken the explicit opt-in (``allow_running_session``) re-evaluates the
        same guard with the running states removed, so STARTING/STOPPING/FAULTED
        are still refused: only a settled, live session is acceptable.
        """
        state = self.lifecycle_state
        if self._allow_running_session:
            return calibration_contract.calibration_precondition(
                state, engine_running_states=()
            )
        return calibration_contract.calibration_precondition(state)

    def start(self) -> calibration_contract.StepOutcome:
        """Create the input source (never a gamepad) and open the first step."""
        guard = self.precondition()
        guard.require()
        if self._started:
            return self._session_state.feed(self._clock(), [])
        if self._sample_source is None:
            self._build_source()
        self._started = True
        return self._session_state.feed(self._clock(), [])

    def _build_source(self) -> None:
        """Create the input-only source and its listener.

        No virtual gamepad is created here, and none is ever created by this
        class: the operator's mouse stays free of steering for the whole flow.
        """
        if self._input_backend == INPUT_BACKEND_RAW_INPUT:
            factory = self._source_factory or create_raw_input_source
            tracker = factory()
            self._tracker = tracker
            start_source = getattr(tracker, "start", None)
            if not callable(start_source):
                raise RuntimeError(
                    "the Raw Input source could not be created: it has no start()"
                )
            start_source()
        else:
            mouse = None
            try:
                _, mouse = import_mouse_keyboard()
            except Exception:  # pragma: no cover - dependency boundary
                mouse = None
            self._mouse = mouse
            tracker = RelativeMouseTracker(mouse, recenter=False)
            self._tracker = tracker
            self._listener = mouse.Listener(on_move=tracker.on_move)
            self._listener.start()
        self._sample_source = self._drain_source

    def _drain_source(self, now_s: float) -> float:
        tracker = self._tracker
        if tracker is None:
            return 0.0
        take_through = getattr(tracker, "take_through", None)
        if callable(take_through):
            return float(take_through(now_s))
        return float(tracker.take_dx())

    def stop(self) -> None:
        """Release the input source. Called on every exit path."""
        listener = self._listener
        if listener is not None:
            try:
                listener.stop()
            except Exception:  # pragma: no cover - listener boundary
                pass
            self._listener = None
        tracker = self._tracker
        stop_source = getattr(tracker, "stop", None)
        if callable(stop_source):
            try:
                stop_source()
            except Exception:  # pragma: no cover - source boundary
                pass
        self._tracker = None
        self._started = False

    # ----- driving -------------------------------------------------------
    def poll(self) -> calibration_contract.StepOutcome:
        """Drain one poll's input, feed the flow, and sample the output trace.

        The verification step is answered by the operator, not measured, so it
        receives no samples: the flow treats samples arriving there as a caller
        bug, and inventing them would make the trace mean something it did not
        measure.
        """
        now = self._clock()
        step = self._session_state.current_step
        if step is not None and step.kind == calibration_contract.STEP_VERIFY:
            return self._session_state.feed(now, ())
        samples: list[calibration_contract.SourceSample] = []
        if self._started and self._sample_source is not None:
            delta = float(self._sample_source(now))
            if delta != 0.0:
                samples.append(calibration_contract.SourceSample(now, delta))
        elif self._started:
            samples.append(calibration_contract.SourceSample(now, 0.0))
        outcome = self._session_state.feed(now, samples)
        self._record_output(now, outcome)
        return outcome

    def _record_output(
        self, now_s: float, outcome: calibration_contract.StepOutcome
    ) -> None:
        """Record our own sent output while a measured step is running.

        This is the "actual output trace" the journal asks for. It is sampled
        from published telemetry only: this class never touches the engine's
        internals, and it has no pad of its own to sample.
        """
        if self._output_probe is None:
            return
        if outcome.complete or outcome.aborted or outcome.needs_observation:
            return
        try:
            reading = self._output_probe()
        except Exception:  # pragma: no cover - probe boundary
            return
        if reading is None:
            return
        pos, axis = reading
        self._output_samples.append(
            calibration_contract.OutputSample(float(now_s), float(pos), float(axis))
        )

    def observe(self, observation: str) -> calibration_contract.StepOutcome:
        return self._session_state.observe(observation)

    def retry(self) -> calibration_contract.StepOutcome:
        return self._session_state.retry()

    def skip(self) -> calibration_contract.StepOutcome:
        return self._session_state.skip()

    def abort(self, reason: str) -> calibration_contract.StepOutcome:
        return self._session_state.abort(reason)

    # ----- results -------------------------------------------------------
    def report(self) -> calibration_contract.CalibrationReport:
        report = self._session_state.report()
        if self._output_samples:
            return report.with_output_trace(
                calibration_contract.OutputTrace(tuple(self._output_samples))
            )
        return report

    def suggestion(self) -> calibration_contract.ProfileSuggestion:
        return calibration_contract.suggest_settings(self._settings, self.report())


def engine_output_probe(
    engine: Any, *, require_active: bool = True
) -> Callable[[], tuple[float, float] | None]:
    """Return a probe reading ``(pos, sent axis)`` from published telemetry.

    By default a reading is only produced while the engine is driving a live
    session. A stopped engine still publishes its last telemetry frame, and a
    frame that nothing is updating is not a measurement of anything: recording
    it would put a flat line into the calibration report and label it
    machine-observed. Pass ``require_active=False`` only where the caller has
    already established that the session is live.
    """

    def probe() -> tuple[float, float] | None:
        try:
            telemetry = engine.telemetry()
        except Exception:  # pragma: no cover - engine boundary
            return None
        if not isinstance(telemetry, dict):
            return None
        if require_active and str(
            telemetry.get("lifecycle_state", "")
        ) not in CALIBRATION_LIVE_STATES:
            return None
        pos = telemetry.get("pos")
        axis = telemetry.get("axis_output", telemetry.get("out"))
        if pos is None or axis is None:
            return None
        return float(pos), float(axis)

    return probe


#: What the calibration tab promises while no session is running. Kept as a
#: constant so the live-measurement checkbox can restore the exact wording.
CALIBRATION_PROMISE_INPUT_ONLY = (
    "Calibration measures how far the mouse actually travels, then asks what the "
    "game did with it. It never creates a virtual pad, so nothing can steer while "
    "you measure."
)
#: The same promise under the explicit live-measurement opt-in.
CALIBRATION_PROMISE_LIVE_OUTPUT = (
    "Live output measurement is on: your steering session keeps running, so mouse "
    "movement steers the virtual pad while you measure, and the profile records "
    "the axis the pad actually received. This flow still creates no pad of its own."
)


def _run_live_calibration(
    settings: Settings,
    *,
    input_backend: str,
    write_path: Path | None,
    echo: Callable[[str], None] = print,
    engine_factory: Callable[..., Any] | None = None,
    runner: "CalibrationRunner | None" = None,
) -> int:
    """Calibrate while a real session runs, so the output trace is measured.

    The opt-in is the operator's, and it is theirs alone: this flow still builds
    no pad. The session below is the ordinary product path -- the same engine,
    the same settings, the same backend -- started deliberately so that the
    published axis is a genuine trace rather than a stale telemetry frame. The
    session is stopped on every exit path and its ``StopResult`` is reported,
    because a calibration run must not leave a pad behind.
    """
    factory = engine_factory or (lambda s, b: SteeringEngine(s, input_backend=b))
    engine = factory(settings, input_backend)
    try:
        engine.start()
    except Exception as exc:
        echo(f"Live calibration could not start a session: {exc}")
        try:
            echo(format_telemetry_diagnostic(engine.telemetry()))
        except Exception:  # pragma: no cover - engine boundary
            pass
        return 2
    try:
        return run_calibration_console(
            settings,
            input_backend=input_backend,
            write_path=write_path,
            engine=engine,
            allow_running_session=True,
            echo=echo,
            runner=runner,
        )
    finally:
        try:
            result = engine.stop()
        except Exception as exc:  # pragma: no cover - engine boundary
            echo(f"Live session stop raised: {type(exc).__name__}: {exc}")
        else:
            echo("Live session: " + format_stop_result(result))


# Command words accepted at a calibration prompt. They are single letters so the
# flow can be driven without leaving the keyboard position a driver is using.
#: A console run stops itself after this many polls (about a minute at 60 Hz).
#: A source that has silently stopped delivering input must not turn the flow
#: into an open-ended wait.
DEFAULT_CALIBRATION_MAX_POLLS = 3600

CALIBRATION_ABORT_WORDS = ("q", "quit", "abort", "escape", "esc")
CALIBRATION_RETRY_WORDS = ("r", "retry", "again")
CALIBRATION_SKIP_WORDS = ("s", "skip", "later")


def format_calibration_prompt(
    outcome: calibration_contract.StepOutcome,
) -> str:
    """Return the operator-facing block for one flow state."""
    if outcome.aborted:
        return outcome.status_line()
    if outcome.complete:
        return outcome.status_line()
    lines = [
        "",
        "-" * 62,
        f"Step {outcome.step_index + 1} of {outcome.step_count}: {outcome.title}",
        f"  {outcome.prompt}",
    ]
    if outcome.guidance:
        lines.append(f"  ({outcome.guidance})")
    if outcome.needs_observation:
        if outcome.step == calibration_contract.STEP_LOCK:
            lines.append(
                "  Report the in-game result: [f]ull lock, [p]artial, "
                "[s]aturated"
            )
        else:
            lines.append(
                "  Report how it felt: [a]bout right, [t]oo sensitive, "
                "[n]ot enough"
            )
    else:
        lines.append(f"  progress: {outcome.progress:.0%}")
    if outcome.failure:
        lines.append(f"  ATTENTION: {outcome.failure}")
    lines.append(
        "  commands: r = retry this step, s = skip (optional steps), q = abort"
    )
    lines.append("-" * 62)
    return "\n".join(lines)


def format_calibration_suggestion(
    suggestion: calibration_contract.ProfileSuggestion,
) -> str:
    """Return the proposal table shown before anything is written."""
    lines = ["", "=" * 62, "Calibration result", "=" * 62]
    if not suggestion.allowed:
        return "\n".join(lines + [f"No profile is offered: {suggestion.blocked_reason}", "=" * 62])
    if not suggestion.changed_fields:
        lines.append(
            "The measurement agrees with the current profile; nothing needs to "
            "change."
        )
    for item in suggestion.changed_fields:
        lines.append("  " + item.describe())
    for warning in suggestion.warnings:
        lines.append("  note: " + warning)
    lines.append("=" * 62)
    return "\n".join(lines)


def run_calibration_console(
    settings: Settings,
    *,
    input_backend: str = INPUT_BACKEND_RAW_INPUT,
    write_path: Path | None = None,
    runner: "CalibrationRunner | None" = None,
    echo: Callable[[str], None] = print,
    prompt: Callable[[str], str] = input,
    engine: Any = None,
    allow_running_session: bool = False,
    max_polls: int | None = None,
    poll_interval_s: float = 1.0 / 60.0,
) -> int:
    """Run the console calibration flow and return a process status.

    ``runner``, ``echo``, and ``prompt`` are injectable so the whole interaction
    -- including every refusal and the final confirmations -- is testable without
    a mouse, a display, or ViGEm.
    """
    printer = echo
    printer("=" * 62)
    printer("Mouse calibration — input only")
    printer("=" * 62)
    if allow_running_session:
        printer(
            "Live output measurement is ON: a steering session keeps running, so "
            "your mouse is steering the virtual pad while you measure."
        )
        printer(
            "The pad's published axis is recorded as the output trace. This flow "
            "still creates no pad of its own."
        )
    else:
        printer(
            "This flow never creates a virtual pad, so nothing can steer while you "
            "measure."
        )
    printer(
        "It measures the mouse, then asks what the GAME did, because only you can "
        "see inside the game."
    )
    printer(safety_contract.safety_policy_notice())

    live_probe = (
        engine_output_probe(engine)
        if (engine is not None and allow_running_session)
        else None
    )
    active_runner = runner or CalibrationRunner(
        settings,
        input_backend=input_backend,
        output_probe=live_probe,
        allow_running_session=allow_running_session,
    )
    try:
        outcome = active_runner.start()
    except RuntimeError as exc:
        printer(f"Calibration refused: {exc}")
        return 2

    poll_limit = (
        DEFAULT_CALIBRATION_MAX_POLLS if max_polls is None else int(max_polls)
    )
    last_line = ""
    polls = 0
    try:
        while not outcome.complete and not outcome.aborted:
            if polls >= poll_limit:
                # Bounded on purpose: a source that stopped delivering input, or
                # an operator who walked away, must not hold the process open.
                printer(
                    f"Calibration stopped after {poll_limit} polls without "
                    "finishing. Nothing was written."
                )
                active_runner.abort("stopped after the polling limit")
                outcome = active_runner.calibration.report().complete and outcome or outcome
                report = active_runner.report()
                for line in report.summary_lines():
                    printer("  measured: " + line)
                suggestion = calibration_contract.suggest_settings(
                    settings, report
                )
                printer(format_calibration_suggestion(suggestion))
                active_runner.stop()
                return 2
            line = format_calibration_prompt(outcome)
            if line != last_line:
                printer(line)
                last_line = line
            if outcome.needs_observation:
                answer = prompt("  report> ").strip().lower()
                if answer in CALIBRATION_ABORT_WORDS:
                    outcome = active_runner.abort("aborted at the report prompt")
                    break
                if answer in CALIBRATION_RETRY_WORDS:
                    outcome = active_runner.retry()
                    continue
                if answer in CALIBRATION_SKIP_WORDS:
                    outcome = active_runner.skip()
                    last_line = ""
                    continue
                mapped = _calibration_observation(outcome, answer)
                if mapped is None:
                    printer(
                        "  unknown report. Use f/p/s for the lock step or "
                        "a/t/n for the verification step."
                    )
                    continue
                outcome = active_runner.observe(mapped)
                last_line = ""
                continue
            polls += 1
            if poll_interval_s > 0.0:
                time.sleep(poll_interval_s)
            outcome = active_runner.poll()
    except KeyboardInterrupt:
        outcome = active_runner.abort("interrupted (Ctrl+C)")
    except (RuntimeError, ValueError) as exc:
        # A source or flow that refuses mid-run is reported, not traced back at
        # the operator: nothing is written on this path either.
        printer(f"Calibration stopped: {exc}")
        active_runner.abort(str(exc))
    finally:
        report = active_runner.report()
        active_runner.stop()

    for line in report.summary_lines():
        printer("  measured: " + line)

    suggestion = calibration_contract.suggest_settings(settings, report)
    printer(format_calibration_suggestion(suggestion))
    if not suggestion.allowed:
        return 2
    if not suggestion.changed_fields:
        return 0

    answer = prompt("Apply these changes to the profile? [y/N] ").strip().lower()
    if answer not in ("y", "yes"):
        printer("Nothing was written.")
        return 0

    applied = calibration_contract.apply_suggestion(settings, suggestion)
    provenance = dict(suggestion.provenance)
    provenance["measured_at"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    applied = replace(applied, calibration=provenance)
    if write_path is None:
        printer(
            "Changes are in memory only; no --calibrate-write path was given, so "
            "nothing was saved."
        )
        return 0
    try:
        saved = save_preset(applied, Path(write_path))
    except (OSError, ValueError) as exc:
        printer(f"Could not save the calibrated profile: {exc}")
        return 2
    printer(f"Saved the calibrated profile to {saved.path}.")
    for notice in saved.notices:
        printer(f"  note: {notice}")
    return 0


def _calibration_observation(
    outcome: calibration_contract.StepOutcome, answer: str
) -> str | None:
    """Map a one-letter console answer to an observation value."""
    if outcome.step == calibration_contract.STEP_LOCK:
        return {
            "f": calibration_contract.OBSERVED_FULL,
            "full": calibration_contract.OBSERVED_FULL,
            "p": calibration_contract.OBSERVED_PARTIAL,
            "partial": calibration_contract.OBSERVED_PARTIAL,
            "s": calibration_contract.OBSERVED_SATURATED,
            "saturated": calibration_contract.OBSERVED_SATURATED,
        }.get(answer)
    if outcome.step == calibration_contract.STEP_VERIFY:
        return {
            "a": calibration_contract.OBSERVED_ABOUT_RIGHT,
            "about": calibration_contract.OBSERVED_ABOUT_RIGHT,
            "about right": calibration_contract.OBSERVED_ABOUT_RIGHT,
            "t": calibration_contract.OBSERVED_TOO_SENSITIVE,
            "too sensitive": calibration_contract.OBSERVED_TOO_SENSITIVE,
            "n": calibration_contract.OBSERVED_NOT_ENOUGH,
            "not enough": calibration_contract.OBSERVED_NOT_ENOUGH,
        }.get(answer)
    return None


# ===========================================================================
# SCREEN / MOUSE
# ===========================================================================


def get_screen_center() -> tuple[int, int] | None:
    """Centre of the desktop, spanning all monitors, when available."""
    if sys.platform == "win32":
        import ctypes

        user32 = ctypes.windll.user32
        try:
            x = user32.GetSystemMetrics(76)  # SM_XVIRTUALSCREEN
            y = user32.GetSystemMetrics(77)  # SM_YVIRTUALSCREEN
            w = user32.GetSystemMetrics(78)  # SM_CXVIRTUALSCREEN
            h = user32.GetSystemMetrics(79)  # SM_CYVIRTUALSCREEN
            return (x + w // 2, y + h // 2)
        except Exception:
            return (
                user32.GetSystemMetrics(0) // 2,
                user32.GetSystemMetrics(1) // 2,
            )

    try:
        import tkinter

        root = tkinter.Tk()
        root.withdraw()
        center = (root.winfo_screenwidth() // 2, root.winfo_screenheight() // 2)
        root.destroy()
        return center
    except Exception:
        return None


class RelativeMouseTracker:
    """Timestamped, explicitly degraded cursor-coordinate fallback source.

    Raw Input is the accepted default steering backend. This class remains only
    as a visibly degraded, explicitly selected compatibility path. Its
    recentering state is deliberately target-based: it may consume
    a callback only when it
    matches one *known* warp target before a short expiry. It never discards an
    arbitrary next event and never clears genuine queued input for a warp.
    """

    backend_name = INPUT_BACKEND_CURSOR_FALLBACK
    allows_cursor_warp = True
    input_degraded = True

    def __init__(
        self,
        mouse_module,
        recenter: bool = True,
        clock: Callable[[], float] | None = None,
    ) -> None:
        self._lock = threading.Lock()
        # Events are stamped at callback arrival instead of being reduced to a
        # scalar immediately. The scheduler can therefore consume exactly the
        # slice of movement that belongs to its bounded control timeline after
        # a stall, leaving newer physical input queued for later reports.
        self._events: deque[tuple[float, float]] = deque()
        self._clock = clock or time.perf_counter
        self._last_xy: tuple[float, float] | None = None
        self._pending_warp: _PendingCursorWarp | None = None
        self._next_warp_token = 0
        self._paused = False
        self._rebase_required = False
        self._warp_attempt_count = 0
        self._warp_failure_count = 0
        self._synthetic_warp_event_count = 0
        self._expired_warp_count = 0
        self._rebase_count = 0
        self._paused_discarded_event_count = 0
        self._last_error: str | None = None
        self._controller = mouse_module.Controller()
        self._screen_center = get_screen_center() if recenter else None

    @staticmethod
    def _matches_target(
        point: tuple[float, float], target: tuple[float, float]
    ) -> bool:
        return (
            abs(point[0] - target[0]) <= CURSOR_WARP_TARGET_TOLERANCE_PX
            and abs(point[1] - target[1]) <= CURSOR_WARP_TARGET_TOLERANCE_PX
        )

    def _expire_pending_warp_locked(self, now: float) -> None:
        pending = self._pending_warp
        if pending is not None and now >= pending.expires_at:
            self._pending_warp = None
            self._expired_warp_count += 1

    def _append_event_locked(self, event_t: float, dx: float) -> None:
        """Append one nonzero callback delta while retaining timestamp order."""
        if not dx:
            return
        event = (event_t, dx)
        if not self._events or event_t >= self._events[-1][0]:
            self._events.append(event)
            return
        insert_at = len(self._events)
        while insert_at > 0 and self._events[insert_at - 1][0] > event_t:
            insert_at -= 1
        self._events.insert(insert_at, event)

    def _read_controller_position(self) -> tuple[float, float] | None:
        """Best-effort actual-coordinate read for failure/pause rebasing."""
        try:
            position = self._controller.position
            if position is None or len(position) != 2:
                return None
            return (float(position[0]), float(position[1]))
        except Exception:
            return None

    def _rebase_from_controller(self, *, require_callback_if_unavailable: bool) -> None:
        actual = self._read_controller_position()
        with self._lock:
            if actual is not None:
                self._last_xy = actual
                self._rebase_required = False
                self._rebase_count += 1
            elif require_callback_if_unavailable:
                # Avoid manufacturing a large delta if the cursor moved while
                # paused and this controller cannot report its current position.
                # The next callback establishes a safe baseline instead.
                self._rebase_required = True

    def on_move(self, x: float, y: float) -> None:
        event_t = float(self._clock())
        point = (float(x), float(y))
        with self._lock:
            self._expire_pending_warp_locked(event_t)

            if self._paused:
                # Track a fresh baseline while paused but never queue steering
                # input. Resume separately rebases in case callbacks were lost.
                self._last_xy = point
                return

            if self._rebase_required:
                self._last_xy = point
                self._rebase_required = False
                self._rebase_count += 1
                return

            pending = self._pending_warp
            if pending is not None and self._matches_target(point, pending.target):
                # Consume only the known synthetic target while it is fresh.
                # If a genuine callback was already accepted after the warp,
                # retain that newer baseline: a delayed synthetic event must
                # not rewind it and create a later false delta.
                self._pending_warp = None
                self._synthetic_warp_event_count += 1
                if not pending.saw_real_callback:
                    self._last_xy = point
                return

            if self._last_xy is not None:
                self._append_event_locked(event_t, point[0] - self._last_xy[0])
            self._last_xy = point
            if pending is not None:
                pending.saw_real_callback = True

    def take_through(self, timestamp: float) -> float:
        """Consume movement no newer than ``timestamp`` in callback-time order."""
        with self._lock:
            dx = 0.0
            while self._events and self._events[0][0] <= timestamp:
                _, event_dx = self._events.popleft()
                dx += event_dx
            return dx

    def take_dx(self) -> float:
        """Legacy destructive drain used by old callers/tests.

        The Step-12 control loop uses :meth:`take_through` so timestamped input
        can remain pending across a capped scheduler catch-up.
        """
        with self._lock:
            dx = sum(event_dx for _, event_dx in self._events)
            self._events.clear()
            return dx

    @property
    def pending_event_count(self) -> int:
        with self._lock:
            return len(self._events)

    @property
    def dropped_event_count(self) -> int:
        """Cursor fallback has no bounded queue; only Raw Input can overflow."""
        return 0

    @property
    def queue_capacity(self) -> int:
        return 0

    @property
    def status(self) -> str:
        return "degraded_cursor_fallback"

    @property
    def last_error(self) -> str | None:
        with self._lock:
            return self._last_error

    @property
    def paused(self) -> bool:
        with self._lock:
            return self._paused

    @property
    def recentering(self) -> bool:
        return self._screen_center is not None

    @property
    def needs_cursor_warp(self) -> bool:
        center = self._screen_center
        if center is None:
            return False
        now = float(self._clock())
        with self._lock:
            self._expire_pending_warp_locked(now)
            return bool(
                not self._paused
                and not self._rebase_required
                and self._pending_warp is None
                and self._last_xy is not None
                and not self._matches_target(self._last_xy, center)
            )

    @property
    def cursor_warp_pending(self) -> bool:
        now = float(self._clock())
        with self._lock:
            self._expire_pending_warp_locked(now)
            return self._pending_warp is not None

    @property
    def cursor_warp_attempt_count(self) -> int:
        with self._lock:
            return self._warp_attempt_count

    @property
    def cursor_warp_failure_count(self) -> int:
        with self._lock:
            return self._warp_failure_count

    @property
    def cursor_synthetic_warp_event_count(self) -> int:
        with self._lock:
            return self._synthetic_warp_event_count

    @property
    def cursor_expired_warp_count(self) -> int:
        with self._lock:
            return self._expired_warp_count

    @property
    def cursor_rebase_count(self) -> int:
        with self._lock:
            return self._rebase_count

    @property
    def cursor_paused_discarded_event_count(self) -> int:
        with self._lock:
            return self._paused_discarded_event_count

    @property
    def cursor_rebase_required(self) -> bool:
        with self._lock:
            return self._rebase_required

    def warp_to_center(self) -> bool:
        """Request one safe recenter only when movement made it necessary.

        This method never clears ``_events``. A successful setter establishes
        the target as the no-callback baseline, while a pending target remains
        briefly available to consume its matching synthetic callback.
        """
        center = self._screen_center
        if center is None:
            return False

        now = float(self._clock())
        with self._lock:
            self._expire_pending_warp_locked(now)
            if (
                self._paused
                or self._rebase_required
                or self._pending_warp is not None
                or self._last_xy is None
                or self._matches_target(self._last_xy, center)
            ):
                return False

            self._next_warp_token += 1
            token = self._next_warp_token
            self._pending_warp = _PendingCursorWarp(
                token=token,
                target=center,
                expires_at=now + CURSOR_WARP_PENDING_TTL_S,
            )
            self._warp_attempt_count += 1

        try:
            self._controller.position = center
        except Exception as exc:
            with self._lock:
                pending = self._pending_warp
                if pending is not None and pending.token == token:
                    self._pending_warp = None
                self._warp_failure_count += 1
                self._last_error = (
                    f"Cursor recenter failed: {type(exc).__name__}: {exc}"
                )
            # A readable controller gives the actual post-failure coordinate.
            # A setter may have partially moved the cursor before raising, so
            # an unreadable controller cannot safely retain the old baseline:
            # the next callback becomes a baseline rather than a potentially
            # huge invented steering delta.
            self._rebase_from_controller(require_callback_if_unavailable=True)
            return False

        with self._lock:
            pending = self._pending_warp
            if pending is not None and pending.token == token:
                if not pending.saw_real_callback:
                    self._last_xy = center
            self._last_error = None
        return True

    def set_paused(self, paused: bool) -> None:
        paused = bool(paused)
        with self._lock:
            changed = self._paused != paused
            self._paused = paused
            self._pending_warp = None
            if changed:
                self._paused_discarded_event_count += len(self._events)
                self._events.clear()

        # Cursor motion may have occurred while callbacks were paused. Query
        # the controller on both transition edges; if it cannot report a point,
        # the next callback is made a baseline rather than a steering delta.
        if changed:
            self._rebase_from_controller(require_callback_if_unavailable=True)


def source_diagnostics(source, fallback_backend: str) -> dict[str, Any]:
    """Return a stable telemetry subset for either input-source implementation."""

    return {
        "input_backend": getattr(source, "backend_name", fallback_backend),
        "input_source_status": getattr(source, "status", "stopped"),
        "input_queue_capacity": getattr(source, "queue_capacity", 0),
        "input_dropped_events": getattr(source, "dropped_event_count", 0),
        "input_packet_errors": getattr(source, "packet_error_count", 0),
        "input_source_error": getattr(source, "last_error", None),
        "input_degraded": bool(getattr(source, "input_degraded", False)),
        "cursor_warp_pending": bool(
            getattr(source, "cursor_warp_pending", False)
        ),
        "cursor_warp_attempt_count": getattr(
            source, "cursor_warp_attempt_count", 0
        ),
        "cursor_warp_failure_count": getattr(
            source, "cursor_warp_failure_count", 0
        ),
        "cursor_synthetic_warp_event_count": getattr(
            source, "cursor_synthetic_warp_event_count", 0
        ),
        "cursor_expired_warp_count": getattr(
            source, "cursor_expired_warp_count", 0
        ),
        "cursor_rebase_count": getattr(source, "cursor_rebase_count", 0),
        "cursor_rebase_required": bool(
            getattr(source, "cursor_rebase_required", False)
        ),
        "cursor_paused_discarded_event_count": getattr(
            source, "cursor_paused_discarded_event_count", 0
        ),
    }


# ===========================================================================
# GAMEPAD
# ===========================================================================


def create_gamepad():
    vg = import_vgamepad()
    cls = getattr(vg, "VX360Gamepad", None)
    if cls is None:
        raise RuntimeError(
            "This vgamepad build does not expose VX360Gamepad."
        )
    try:
        return cls()
    except Exception as exc:
        raise RuntimeError(
            "Could not create the virtual Xbox 360 gamepad.\n"
            "Is the ViGEmBus driver installed and working?\n"
            f"Underlying error: {exc}"
        ) from exc


def shutdown_gamepad(gamepad) -> str | None:
    """Neutralise/reset a virtual pad and return a truthful failure detail.

    Current vgamepad keeps the device connected for the lifetime of the
    ``VX360Gamepad`` object and disconnects when that object is released, so
    there is deliberately no nonexistent ``delete()`` call.  Callers that need
    confirmed cleanup must retain the reference when this returns an error;
    silently dropping a pad that may not have been neutralised is unsafe.
    """
    if gamepad is None:
        return None
    try:
        gamepad.left_joystick_float(x_value_float=0.0, y_value_float=0.0)
        gamepad.update()
        gamepad.reset()
        gamepad.update()
    except Exception as exc:
        return f"could not neutralise virtual gamepad: {exc}"
    return None


# ===========================================================================
# STEERING ENGINE
# ===========================================================================


# Phase 3 / Step 16 lifecycle states.  These strings deliberately appear in
# telemetry/UI output, so they are stable public diagnostics rather than an
# implementation-only boolean.
RUNTIME_STOPPED = "STOPPED"
RUNTIME_STARTING = "STARTING"
RUNTIME_RUNNING = "RUNNING"
RUNTIME_PAUSED = "PAUSED"
RUNTIME_STOPPING = "STOPPING"
RUNTIME_FAULTED = "FAULTED"
RUNTIME_STATES = (
    RUNTIME_STOPPED,
    RUNTIME_STARTING,
    RUNTIME_RUNNING,
    RUNTIME_PAUSED,
    RUNTIME_STOPPING,
    RUNTIME_FAULTED,
)
_RUNTIME_ACTIVE_STATES = frozenset(
    (RUNTIME_STARTING, RUNTIME_RUNNING, RUNTIME_PAUSED, RUNTIME_STOPPING)
)

#: The lifecycle states in which a session is really driving the pad. The
#: calibration guard refuses these by default, and the live output probe trusts
#: exactly this tuple, so the two cannot disagree about what "live" means.
CALIBRATION_LIVE_STATES = (RUNTIME_RUNNING, RUNTIME_PAUSED)
_RUNTIME_TRANSITIONS = {
    RUNTIME_STOPPED: frozenset((RUNTIME_STARTING, RUNTIME_FAULTED)),
    RUNTIME_STARTING: frozenset((RUNTIME_RUNNING, RUNTIME_STOPPING, RUNTIME_FAULTED)),
    RUNTIME_RUNNING: frozenset((RUNTIME_PAUSED, RUNTIME_STOPPING, RUNTIME_FAULTED)),
    RUNTIME_PAUSED: frozenset((RUNTIME_RUNNING, RUNTIME_STOPPING, RUNTIME_FAULTED)),
    RUNTIME_STOPPING: frozenset((RUNTIME_STOPPED, RUNTIME_FAULTED)),
    RUNTIME_FAULTED: frozenset((RUNTIME_STOPPING, RUNTIME_STOPPED)),
}

# Stable public Step-17 stop-result strings.  Keep result state separate from
# lifecycle state: a faulted session can have confirmed resource cleanup, and a
# STOPPING session can have a stop request that has not yet been confirmed.
STOP_RESULT_NOT_REQUESTED = "not_requested"
STOP_RESULT_STOPPED = "stopped"
STOP_RESULT_STOPPING_TIMEOUT = "stopping_timeout"
STOP_RESULT_ALREADY_STOPPED = "already_stopped"
STOP_RESULT_CALLED_FROM_WORKER = "called_from_worker"
STOP_RESULT_FAULTED = "faulted"
_STOP_RESULT_VALUES = frozenset(
    (
        STOP_RESULT_NOT_REQUESTED,
        STOP_RESULT_STOPPED,
        STOP_RESULT_STOPPING_TIMEOUT,
        STOP_RESULT_ALREADY_STOPPED,
        STOP_RESULT_CALLED_FROM_WORKER,
        STOP_RESULT_FAULTED,
    )
)
_DEFAULT_STOP_TIMEOUT_S = 2.0


@dataclass(frozen=True)
class RuntimeDiagnostic:
    """One actionable runtime exception record retained by a session."""

    stage: str
    exception_type: str
    message: str
    cause_type: str | None
    cause_message: str | None
    action: str
    traceback_text: str
    created_at: str
    log_path: str | None = None

    @property
    def summary(self) -> str:
        return (
            f"{_humanise_diagnostic_stage(self.stage)} failed "
            f"({self.exception_type}: {_brief_diagnostic_message(self.message)})"
        )


def _brief_diagnostic_message(message: Any) -> str:
    """Keep interactive surfaces readable while telemetry/logs retain the original."""
    return " ".join(str(message).split()) or "(no exception message)"


def _humanise_diagnostic_stage(stage: str) -> str:
    labels = {
        "startup.raw_input.create_source": "Creating the Raw Input source",
        "startup.raw_input.start_source": "Starting the Raw Input source",
        "startup.keyboard_hotkeys.import": "Loading keyboard hotkeys",
        "startup.cursor_fallback.import_pynput": "Loading cursor fallback input",
        "startup.cursor_fallback.create_tracker": "Creating the cursor fallback tracker",
        "startup.virtual_gamepad.create": "Creating the virtual gamepad",
        "startup.mouse_listener.start": "Starting the mouse listener",
        "startup.hotkeys.create": "Creating global hotkeys",
        "startup.hotkeys.start": "Starting global hotkeys",
        "startup.safety.probe": "Creating the foreground probe",
        "startup.deadman_listener.start": "Starting the deadman key listener",
        "startup.focus_watcher.start": "Starting the foreground watcher",
        "startup.control_worker.start": "Starting the control worker",
        "startup.previous_cleanup": "Starting after unconfirmed cleanup",
        "runtime.input": "Reading steering input",
        "runtime.virtual_gamepad_output": "Writing virtual gamepad output",
        "runtime.cursor_recenter": "Recentering the cursor fallback",
        "runtime.control_loop": "Running the steering control loop",
        "shutdown.control_loop_join": "Joining the control worker",
        "shutdown.control_loop_timeout": "Waiting for the control worker to stop",
        "shutdown.virtual_gamepad_neutralise": "Neutralising the virtual gamepad",
        "shutdown.listener_stop": "Stopping an input listener",
        "shutdown.focus_watcher_join": "Joining the foreground watcher",
        "shutdown.focus_watcher_timeout": "Waiting for the foreground watcher to stop",
        "shutdown.listener_join": "Joining an input listener",
        "shutdown.listener_timeout": "Waiting for an input listener to stop",
        "shutdown.input_source_stop": "Stopping the input source",
        "shutdown.input_source_timeout": "Waiting for the input source to stop",
        "shutdown.resource_cleanup": "Cleaning up runtime resources",
    }
    return labels.get(stage, stage.replace("_", " ").replace(".", " / "))


def _recommended_failure_action(
    stage: str,
    exc: BaseException,
    input_backend: str | None,
) -> str:
    """Give a bounded, truthful next action without hiding the root cause."""
    details = f"{type(exc).__name__}: {exc}".lower()
    if "pynput" in details:
        return (
            "Install or repair pynput, then retry. If this is an intentionally "
            "degraded compatibility launch, confirm the selected input mode first."
        )
    if "vgamepad" in details or "vigem" in details:
        return (
            "Check vgamepad and the ViGEmBus driver, then retry. Do not assume "
            "a virtual pad was created."
        )
    if "listener" in stage or "hotkey" in stage:
        return (
            "Close conflicting hook/overlay software and retry. If shutdown is "
            "pending, close only after cleanup is confirmed."
        )
    if stage == "startup.previous_cleanup":
        return (
            "Retry Stop/cleanup for the existing session; do not start another "
            "session or close as though the virtual pad were disconnected."
        )
    if stage.startswith("startup.raw_input"):
        return (
            "Confirm Windows Raw Input support and retry. If compatibility capture "
            "is deliberately required, restart with --cursor-fallback (DEGRADED)."
        )
    if stage == "runtime.virtual_gamepad_output":
        return (
            "Retry after checking the virtual-controller driver. Use Stop/Retry "
            "cleanup and do not close as though the pad were disconnected."
        )
    if stage == "startup.safety.probe":
        return (
            "The foreground-window probe could not be created. Disarm the focus "
            "guard, or run on a supported Windows desktop, then retry."
        )
    if stage == "startup.deadman_listener.start":
        return (
            "The deadman key listener could not start. Check for conflicting "
            "keyboard-hook software, or disarm the deadman in Safety settings."
        )
    if stage.startswith("shutdown") or isinstance(exc, TimeoutError):
        return (
            "Wait or retry stop; do not assume the virtual pad is disconnected. "
            "If the problem persists, review the retained root-cause details before "
            "force-closing."
        )
    if input_backend == INPUT_BACKEND_RAW_INPUT:
        return (
            "Retry after checking the input source and virtual-controller driver. "
            "Review the reported root cause before changing the input mode."
        )
    return (
        "Retry after correcting the reported problem. Review the retained root-cause "
        "details before closing or starting another session."
    )


def _write_diagnostic_log(diagnostic: RuntimeDiagnostic) -> str | None:
    """Append one traceback record without allowing logging failure to mask it."""
    try:
        DIAGNOSTIC_LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
        with _diagnostic_log_lock:
            with DIAGNOSTIC_LOG_PATH.open("a", encoding="utf-8") as handle:
                handle.write("=" * 78 + "\n")
                handle.write(f"{diagnostic.created_at}  {diagnostic.summary}\n")
                if diagnostic.cause_type:
                    handle.write(
                        "Cause: "
                        f"{diagnostic.cause_type}: {diagnostic.cause_message}\n"
                    )
                handle.write(f"Recommended action: {diagnostic.action}\n")
                handle.write(diagnostic.traceback_text.rstrip() + "\n")
        return str(DIAGNOSTIC_LOG_PATH)
    except Exception:
        # Diagnostics must never replace the original startup/runtime fault.
        return None


def capture_runtime_diagnostic(
    stage: str,
    exc: BaseException,
    input_backend: str | None = None,
) -> RuntimeDiagnostic:
    """Capture exception identity/cause/traceback and write a best-effort log."""
    cause = exc.__cause__
    if cause is None and not getattr(exc, "__suppress_context__", False):
        cause = exc.__context__
    diagnostic = RuntimeDiagnostic(
        stage=stage,
        exception_type=type(exc).__name__,
        message=str(exc).strip() or repr(exc),
        cause_type=type(cause).__name__ if cause is not None else None,
        cause_message=(str(cause).strip() or repr(cause)) if cause is not None else None,
        action=_recommended_failure_action(stage, exc, input_backend),
        traceback_text="".join(
            traceback.format_exception(type(exc), exc, exc.__traceback__)
        ),
        created_at=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    )
    return replace(diagnostic, log_path=_write_diagnostic_log(diagnostic))


def format_runtime_diagnostic(diagnostic: RuntimeDiagnostic) -> str:
    """Format concise GUI/CLI guidance while retaining full trace in the log."""
    message = f"{diagnostic.summary}. {diagnostic.action}"
    if diagnostic.log_path:
        message += f" Inspect diagnostic log: {diagnostic.log_path}"
    return message


def format_telemetry_diagnostic(telemetry: dict[str, Any]) -> str:
    """Render telemetry diagnostics for a UI surface without exposing a trace."""
    error_type = telemetry.get("error_type")
    error_message = telemetry.get("error_message")
    if not error_type or not error_message:
        return str(telemetry.get("error") or "Unknown runtime error.")
    stage = str(telemetry.get("error_stage") or "runtime")
    action = str(telemetry.get("error_action") or "Retry after inspection.")
    message = (
        f"{_humanise_diagnostic_stage(stage)} failed "
        f"({error_type}: {_brief_diagnostic_message(error_message)}). {action}"
    )
    log_path = telemetry.get("diagnostic_log_path")
    if log_path:
        message += f" Inspect diagnostic log: {log_path}"
    return message


class EngineStartupError(RuntimeError):
    """A start failure with a session telemetry/log diagnostic attached."""

    def __init__(self, diagnostic: RuntimeDiagnostic) -> None:
        self.diagnostic = diagnostic
        super().__init__(format_runtime_diagnostic(diagnostic))


@dataclass(frozen=True)
class StopResult:
    """Truthful, immutable outcome from one :meth:`SteeringEngine.stop` call."""

    status: str
    session_id: int | None
    lifecycle_state: str
    cleanup_complete: bool
    detail: str | None = None

    @property
    def cleanup_confirmed(self) -> bool:
        """Whether resource release was confirmed for this result."""
        return self.cleanup_complete and self.status in (
            STOP_RESULT_STOPPED,
            STOP_RESULT_ALREADY_STOPPED,
            STOP_RESULT_FAULTED,
        )


@dataclass
class RuntimeSession:
    """All resources and lifecycle facts for exactly one engine run.

    A worker receives this object directly and must never fetch mutable runtime
    resources back from the engine.  This makes stale callbacks/threads unable
    to steer, clean up, or publish telemetry for a replacement session.
    """

    session_id: int
    stop_event: Any
    state: str = RUNTIME_STARTING
    worker: threading.Thread | None = None
    tracker: Any = None
    mouse_listener: Any = None
    hotkeys: Any = None
    # Phase 4 / Step 22 operational safety: one foreground probe and (when the
    # deadman is armed) one extra keyboard listener per session.
    focus_probe: Any = None
    focus_reading: Any = None
    focus_watcher: Any = None
    safety_listener: Any = None
    deadman_spec: Any = None
    deadman_held: bool = False
    pause_cause: str | None = None
    pause_detail: str | None = None
    # Operator-facing feedback about the pause control itself (for example a
    # refused resume). Kept separate from ``pause_detail`` so the control loop's
    # per-report guard reason cannot erase it.
    pause_notice: str | None = None
    keyboard: Any = None
    mouse: Any = None
    gamepad: Any = None
    cleanup_started: bool = False
    cleanup_complete: bool = False
    cleanup_error: str | None = None
    cleanup_failure_stage: str | None = None
    cleanup_exception: BaseException | None = field(
        default=None,
        repr=False,
        compare=False,
    )
    fault_reason: str | None = None
    diagnostic: RuntimeDiagnostic | None = None
    stop_status: str = STOP_RESULT_NOT_REQUESTED
    stop_detail: str | None = None
    cleanup_owner: Any = None
    cleanup_owner_running: bool = False
    cleanup_lock: Any = field(
        default_factory=threading.Lock,
        repr=False,
        compare=False,
    )
    worker_exited: bool = False


class SteeringEngine:
    """Own one explicit :class:`RuntimeSession` at a time.

    Step 16 makes the session, rather than a collection of engine attributes,
    authoritative for lifecycle state and runtime-resource ownership.  A stale
    worker can finish its own session safely but cannot mutate the replacement
    session's telemetry or resources.
    """

    def __init__(
        self,
        settings: Settings | None = None,
        recenter: bool = True,
        show_status: bool = False,
        input_backend: str | None = None,
        raw_input_factory: Callable[[], Any] | None = None,
        focus_probe_factory: Callable[[], Any] | None = None,
    ) -> None:
        self._settings = sanitise_settings(settings or Settings())
        selected_backend = (
            self._settings.input_mode if input_backend is None else input_backend
        )
        if selected_backend not in INPUT_BACKENDS:
            raise ValueError(
                f"Unknown input backend {selected_backend!r}; expected one of {INPUT_BACKENDS}."
            )
        self._slock = threading.Lock()
        self._tlock = threading.Lock()
        # Lifecycle operations are serial, but worker telemetry must be able to
        # re-enter the same lock through a transition/publish helper.
        self._ops = threading.RLock()
        self._session: RuntimeSession | None = None
        self._next_session_id = 0
        self._recenter = recenter
        self._show_status = show_status
        # Schema-v2 persists input_mode. An explicit caller/CLI backend still
        # wins over the profile, while a migrated v1 profile intentionally
        # selects its retained cursor-coordinate compatibility source.
        self._input_backend = selected_backend
        self._raw_input_factory = raw_input_factory
        # Injectable so the deterministic suite can drive the focus guard with a
        # scripted reading instead of a real desktop.
        self._focus_probe_factory = focus_probe_factory

        dpi_status = current_dpi_awareness()
        self._telemetry = {
            "pos": 0.0,
            # ``out`` remains the legacy sent-axis alias. Step 9 exposes both
            # pipeline stages explicitly for telemetry/diagnostics.
            "out": 0.0,
            "axis_target": 0.0,
            "axis_output": 0.0,
            "steering_mode": self._settings.steering_mode,
            "centering_state": "idle",
            "centering_active": False,
            "holding": False,
            "saturated": False,
            # Step-21 signal-pipeline diagnostics. These remain explicitly in
            # backend source units/s (Raw Input counts or degraded cursor
            # coordinates), while ``input_age_s`` measures accepted effective
            # input rather than raw-position movement.
            "raw_velocity_units_per_s": 0.0,
            "filtered_velocity_units_per_s": 0.0,
            "input_age_s": 0.0,
            # Step-12 scheduler diagnostics. ``loop_dt_s`` is measured wall
            # time; ``integration_dt_s`` is the bounded control slice used for
            # this report. Counts persist for the active engine session.
            "loop_dt_s": 0.0,
            "integration_dt_s": 0.0,
            "max_integration_dt_s": MAX_CONTROL_INTEGRATION_S,
            "integration_lag_s": 0.0,
            "late_tick": False,
            "late_tick_count": 0,
            "worst_jitter_s": 0.0,
            "pending_input_events": 0,
            "late_output_guard": False,
            # Phase-2 input-source diagnostics. Raw Input's queue is bounded
            # and deliberately drop-oldest on overload; the cursor fallback is
            # intentionally unbounded but visibly degraded/quarantined.
            "input_backend": self._input_backend,
            "input_source_status": "not_started",
            "input_queue_capacity": 0,
            "input_dropped_events": 0,
            "input_packet_errors": 0,
            "input_source_error": None,
            # Step-15 degraded-cursor diagnostics. Raw Input leaves all of
            # these false/zero and never possesses a warp method.
            "input_degraded": (
                self._input_backend == INPUT_BACKEND_CURSOR_FALLBACK
            ),
            "cursor_warp_pending": False,
            "cursor_warp_attempt_count": 0,
            "cursor_warp_failure_count": 0,
            "cursor_synthetic_warp_event_count": 0,
            "cursor_expired_warp_count": 0,
            "cursor_rebase_count": 0,
            "cursor_rebase_required": False,
            "cursor_paused_discarded_event_count": 0,
            "cursor_warp_enabled": (
                self._input_backend == INPUT_BACKEND_CURSOR_FALLBACK
                and self._recenter
            ),
            # Phase-2 Step-13 platform diagnostics. ``not_attempted`` is
            # possible only for a programmatic engine created before the entry
            # boundary; start() updates these before importing pynput.
            "dpi_awareness_mode": dpi_status.mode,
            "dpi_awareness_detail": dpi_status.detail,
            "dpi_awareness_configured": dpi_status.configured,
            # Phase-4 Step-22 operational-safety diagnostics. Defaults describe
            # a disarmed, stopped engine; the control loop republishes them from
            # the live gate decision every report.
            "safety_guard_configured": False,
            "safety_fail_mode": self._settings.safety_fail_mode,
            "hotkey_pause": self._settings.hotkey_pause,
            "hotkey_stop": self._settings.hotkey_stop,
            "deadman_enabled": self._settings.deadman_enabled,
            "deadman_held": False,
            "deadman_key": self._settings.deadman_key,
            "focus_guard_enabled": self._settings.focus_guard_enabled,
            "focus_allowlist_size": len(self._settings.focus_allowlist),
            "focus_gate_state": safety_contract.GATE_OPEN,
            "focus_allowed": None,
            "focus_target": None,
            "focus_probe_detail": None,
            "focus_probe_error_count": 0,
            "safety_gate_state": safety_contract.GATE_OPEN,
            "safety_gate_reason": (
                "No armed session; the safety gate is not being evaluated."
            ),
            "safety_gate_warning": None,
            "safety_gate_close_count": 0,
            "safety_blocked_s": 0.0,
            "pause_cause": None,
            "pause_detail": None,
            "pause_notice": None,
            "indicator_label": "ENGINE STOPPED",
            "indicator_severity": safety_contract.INDICATOR_INACTIVE,
            # Phase-3 Step-16 authoritative session lifecycle diagnostics.
            "session_id": None,
            "lifecycle_state": RUNTIME_STOPPED,
            "session_cleanup_complete": True,
            "session_cleanup_error": None,
            "shutdown_status": STOP_RESULT_ALREADY_STOPPED,
            "shutdown_detail": None,
            # Phase-3 Step-18 actionable diagnostic record. Full trace text is
            # retained in telemetry/log for support, while GUI/CLI use the
            # concise type/message/action formatter.
            "error_type": None,
            "error_message": None,
            "error_cause_type": None,
            "error_cause_message": None,
            "error_stage": None,
            "error_action": None,
            "error_traceback": None,
            "diagnostic_log_path": None,
            "running": False,
            "paused": False,
            "error": None,
        }

    # ----- settings ----------------------------------------------------
    def input_backend(self) -> str:
        """The selected input backend for this engine instance."""
        return self._input_backend

    def get_settings(self) -> Settings:
        with self._slock:
            return replace(self._settings)

    def set_field(self, key: str, value) -> None:
        with self._slock:
            if not hasattr(self._settings, key):
                raise AttributeError(f"Unknown setting: {key}")
            candidate = replace(self._settings)
            setattr(candidate, key, value)

            # The current GUI still carries the v1-compatible tuning widgets
            # until Step 21 redesigns it around the explicit v2 names. Do not
            # let an edit to one of those visible controls become a no-op for
            # a native-v2 profile: translate it immediately into the active
            # physical field while retaining the GUI value as a compatibility
            # mirror. The mapping is the documented 83 Hz bridge, not an
            # implicit conversion performed while loading an old profile.
            if candidate.profile_semantics == "v2" and key in {
                "smoothing",
                "noise_gate_px",
                "hysteresis_px",
                "center_time_s",
            }:
                if isinstance(value, bool) or not isinstance(value, (int, float)):
                    raise ValueError(f"{key} must be numeric.")
                numeric_value = float(value)
                if not math.isfinite(numeric_value):
                    raise ValueError(f"{key} must be finite.")
                if key == "smoothing":
                    if not 0.0 <= numeric_value < 1.0:
                        raise ValueError("smoothing must be in [0, 1).")
                    candidate.smoothing_tau_ms = (
                        0.0
                        if numeric_value == 0.0
                        else -1000.0
                        / (LEGACY_FILTER_REFERENCE_HZ * math.log(numeric_value))
                    )
                elif key == "noise_gate_px":
                    candidate.noise_gate_units_per_s = (
                        numeric_value * LEGACY_FILTER_REFERENCE_HZ
                    )
                elif key == "hysteresis_px":
                    candidate.centering_hysteresis_units_per_s = (
                        numeric_value * LEGACY_FILTER_REFERENCE_HZ
                    )
                else:  # center_time_s
                    candidate.center_return_time_ms = numeric_value * 1000.0

            # Retain the legacy boolean as a compatibility shortcut: turning
            # it on selects the legacy release mode, turning it off selects
            # manual. The explicit mode selector remains authoritative.
            if key == "auto_return_enabled" and isinstance(value, bool):
                candidate.steering_mode = (
                    "release_to_center" if value else "manual"
                )
            elif key == "steering_mode" and value in STEERING_MODES:
                candidate.auto_return_enabled = value != "manual"
            self._settings = sanitise_settings(candidate)

    def replace_settings(self, settings: Settings) -> None:
        validated = sanitise_settings(settings)
        with self._slock:
            self._settings = replace(validated)

    # ----- telemetry ---------------------------------------------------
    def telemetry(self) -> dict[str, Any]:
        with self._tlock:
            return dict(self._telemetry)

    def _set_telemetry(self, **kw) -> None:
        """Update static/pre-session telemetry only.

        Worker and lifecycle paths use ``_publish_session_telemetry`` instead,
        which checks session identity while holding the lifecycle lock.
        """
        with self._tlock:
            self._telemetry.update(kw)

    # ----- lifecycle helpers ------------------------------------------
    @staticmethod
    def _worker_alive(worker: Any) -> bool:
        try:
            return worker is not None and bool(worker.is_alive())
        except Exception:
            return False

    @staticmethod
    def _state_is_active(state: str) -> bool:
        return state in _RUNTIME_ACTIVE_STATES

    @property
    def lifecycle_state(self) -> str:
        with self._ops:
            return self._session.state if self._session is not None else RUNTIME_STOPPED

    @property
    def session_id(self) -> int | None:
        with self._ops:
            return self._session.session_id if self._session is not None else None

    @property
    def running(self) -> bool:
        """Whether the authoritative current session is active.

        ``STOPPING`` intentionally remains active: a new run must not replace
        a session until its worker has actually exited and owner cleanup has
        completed.
        """
        with self._ops:
            return bool(
                self._session is not None
                and self._state_is_active(self._session.state)
            )

    def _new_session_locked(self) -> RuntimeSession:
        self._next_session_id += 1
        return RuntimeSession(
            session_id=self._next_session_id,
            stop_event=threading.Event(),
        )

    def _is_current_session_locked(self, session: RuntimeSession) -> bool:
        return self._session is session

    def _is_current_session(self, session: RuntimeSession) -> bool:
        with self._ops:
            return self._is_current_session_locked(session)

    def _publish_session_telemetry_locked(
        self, session: RuntimeSession, **updates: Any
    ) -> bool:
        """Publish only if ``session`` still owns the engine.

        The lifecycle lock remains held through the telemetry update, closing
        the race where an old loop passes an identity check and then overwrites
        telemetry after a replacement session starts.
        """
        if not self._is_current_session_locked(session):
            return False
        diagnostic = session.diagnostic
        payload = {
            "session_id": session.session_id,
            "lifecycle_state": session.state,
            "session_cleanup_complete": session.cleanup_complete,
            "session_cleanup_error": session.cleanup_error,
            "shutdown_status": session.stop_status,
            "shutdown_detail": session.stop_detail,
            "error_type": diagnostic.exception_type if diagnostic else None,
            "error_message": diagnostic.message if diagnostic else None,
            "error_cause_type": diagnostic.cause_type if diagnostic else None,
            "error_cause_message": diagnostic.cause_message if diagnostic else None,
            "error_stage": diagnostic.stage if diagnostic else None,
            "error_action": diagnostic.action if diagnostic else None,
            "error_traceback": diagnostic.traceback_text if diagnostic else None,
            "diagnostic_log_path": diagnostic.log_path if diagnostic else None,
            "running": self._state_is_active(session.state),
            "paused": session.state == RUNTIME_PAUSED,
        }
        payload.update(updates)
        with self._tlock:
            self._telemetry.update(payload)
        return True

    def _publish_session_telemetry(
        self, session: RuntimeSession, **updates: Any
    ) -> bool:
        with self._ops:
            return self._publish_session_telemetry_locked(session, **updates)

    def _transition_locked(
        self,
        session: RuntimeSession,
        target: str,
        **updates: Any,
    ) -> bool:
        """Perform one checked transition for the current session."""
        if target not in RUNTIME_STATES:
            raise ValueError(f"Unknown runtime state: {target!r}")
        if not self._is_current_session_locked(session):
            return False
        previous = session.state
        if previous != target and target not in _RUNTIME_TRANSITIONS[previous]:
            raise RuntimeError(
                f"Invalid runtime transition {previous} -> {target} "
                f"for session {session.session_id}."
            )
        session.state = target
        return self._publish_session_telemetry_locked(session, **updates)

    def _transition(
        self, session: RuntimeSession, target: str, **updates: Any
    ) -> bool:
        with self._ops:
            return self._transition_locked(session, target, **updates)

    @staticmethod
    def _remaining_timeout(deadline: float) -> float:
        return max(0.0, deadline - time.monotonic())

    @staticmethod
    def _resource_alive(resource: Any) -> bool | None:
        """Return known liveness, or ``None`` when the resource exposes none."""
        if resource is None:
            return False
        probe = getattr(resource, "is_alive", None)
        if callable(probe):
            try:
                return bool(probe())
            except Exception:
                return None
        try:
            running = getattr(resource, "running")
        except Exception:
            return None
        if isinstance(running, bool):
            return running
        return None

    @staticmethod
    def _append_unique_resource(resources: list[tuple[str, Any]], name: str, value: Any) -> None:
        if value is not None and not any(value is existing for _, existing in resources):
            resources.append((name, value))

    def _record_exception_locked(
        self,
        session: RuntimeSession,
        stage: str,
        exc: BaseException,
        *,
        fatal: bool = False,
    ) -> RuntimeDiagnostic:
        """Store/log one exception without allowing diagnostics to hide it.

        The first fatal diagnostic is the session's authoritative root cause.
        Later cleanup diagnostics are still appended to the log but do not
        overwrite that root cause in telemetry.
        """
        diagnostic = capture_runtime_diagnostic(stage, exc, self._input_backend)
        if fatal or session.diagnostic is None:
            session.diagnostic = diagnostic
        self._publish_session_telemetry_locked(session)
        return diagnostic

    def _fault_exception_locked(
        self,
        session: RuntimeSession,
        stage: str,
        exc: BaseException,
    ) -> RuntimeDiagnostic:
        diagnostic = self._record_exception_locked(
            session, stage, exc, fatal=True
        )
        self._fault_session_locked(session, diagnostic.summary)
        return diagnostic

    def _fault_session_locked(self, session: RuntimeSession, reason: str) -> bool:
        """Preserve the first actionable fault reason and publish ``FAULTED``."""
        if reason and session.fault_reason is None:
            session.fault_reason = reason
        fault_reason = session.fault_reason or reason or "Runtime cleanup fault."
        if session.state != RUNTIME_FAULTED:
            return self._transition_locked(
                session,
                RUNTIME_FAULTED,
                error=fault_reason,
            )
        return self._publish_session_telemetry_locked(session, error=fault_reason)

    def _make_stop_result_locked(
        self,
        session: RuntimeSession,
        status: str,
        detail: str | None = None,
    ) -> StopResult:
        if status not in _STOP_RESULT_VALUES:
            raise ValueError(f"Unknown stop result: {status!r}")
        if status == STOP_RESULT_FAULTED:
            detail = detail or session.fault_reason or session.cleanup_error
        session.stop_status = status
        session.stop_detail = detail
        self._publish_session_telemetry_locked(session)
        return StopResult(
            status=status,
            session_id=session.session_id,
            lifecycle_state=session.state,
            cleanup_complete=session.cleanup_complete,
            detail=detail,
        )

    def _cleanup_session_resources(
        self,
        session: RuntimeSession,
        timeout: float,
    ) -> str | None:
        """Confirm cleanup for one dead session without holding ``_ops`` in joins.

        The session keeps every reference until all observable work succeeds:
        gamepad neutralisation comes first, listeners/source are stopped and
        joined where supported, and only then are references dropped.  A
        failure leaves the resources attached to this session for a safe retry.
        """
        deadline = time.monotonic() + max(float(timeout), 0.0)
        with session.cleanup_lock:
            with self._ops:
                if not self._is_current_session_locked(session):
                    return "session is no longer current; cleanup was not applied"
                if session.cleanup_complete:
                    return None
                session.cleanup_started = True
                gamepad = session.gamepad
                tracker = session.tracker
                listeners: list[tuple[str, Any]] = []
                self._append_unique_resource(
                    listeners, "mouse listener", session.mouse_listener
                )
                self._append_unique_resource(listeners, "hotkey listener", session.hotkeys)
                self._append_unique_resource(
                    listeners, "deadman listener", session.safety_listener
                )
                focus_watcher = session.focus_watcher

            failures: list[str] = []
            first_failure_stage: str | None = None
            first_failure_exception: BaseException | None = None

            def remember_cleanup_failure(stage: str, exc: BaseException) -> None:
                nonlocal first_failure_stage, first_failure_exception
                if first_failure_exception is None:
                    first_failure_stage = stage
                    first_failure_exception = exc

            # The pad must be driven neutral before any reference can be
            # released. Continue best-effort listener/source shutdown even if
            # this fails, but do not claim final cleanup or drop the pad.
            pad_error = shutdown_gamepad(gamepad)
            if pad_error:
                failures.append(pad_error)
                remember_cleanup_failure(
                    "shutdown.virtual_gamepad_neutralise",
                    RuntimeError(pad_error),
                )

            # Stop every listener before joining any of them so a paired
            # mouse/hotkey listener cannot keep the other one alive.
            for name, listener in listeners:
                try:
                    listener.stop()
                except Exception as exc:
                    failures.append(f"could not stop {name}: {exc}")
                    remember_cleanup_failure("shutdown.listener_stop", exc)

            # Raw Input owns a message-loop thread and exposes a bounded
            # ``stop(timeout=...)``.  Cursor tracker instances have no stop.
            stop_source = getattr(tracker, "stop", None)
            if callable(stop_source):
                try:
                    try:
                        stop_source(timeout=self._remaining_timeout(deadline))
                    except TypeError:
                        # Lightweight test doubles and pynput-like shims often
                        # expose stop() without the optional timeout argument.
                        stop_source()
                except Exception as exc:
                    failures.append(f"could not stop input source: {exc}")
                    remember_cleanup_failure("shutdown.input_source_stop", exc)

            current = threading.current_thread()
            for name, listener in listeners:
                join = getattr(listener, "join", None)
                if listener is current:
                    detail = f"cannot join {name} from its own callback"
                    failures.append(detail)
                    remember_cleanup_failure("shutdown.listener_join", RuntimeError(detail))
                    continue
                if callable(join):
                    try:
                        join(self._remaining_timeout(deadline))
                    except Exception as exc:
                        failures.append(f"could not join {name}: {exc}")
                        remember_cleanup_failure("shutdown.listener_join", exc)
                        continue
                    alive = self._resource_alive(listener)
                    if alive is True:
                        detail = f"{name} did not stop within the timeout"
                        failures.append(detail)
                        remember_cleanup_failure("shutdown.listener_timeout", TimeoutError(detail))

            source_alive = self._resource_alive(tracker)
            if source_alive is True:
                detail = "input source did not stop within the timeout"
                failures.append(detail)
                remember_cleanup_failure("shutdown.input_source_timeout", TimeoutError(detail))

            # The focus watcher is an ownerless daemon thread signalled through
            # the session's stop event. It is joined here rather than through the
            # listener path because it exposes join() but no stop().
            if (
                focus_watcher is not None
                and focus_watcher is not threading.current_thread()
            ):
                watcher_join = getattr(focus_watcher, "join", None)
                if callable(watcher_join):
                    try:
                        watcher_join(self._remaining_timeout(deadline))
                    except Exception as exc:
                        failures.append(f"could not join focus watcher: {exc}")
                        remember_cleanup_failure("shutdown.focus_watcher_join", exc)
                    else:
                        if self._resource_alive(focus_watcher) is True:
                            detail = "focus watcher did not stop within the timeout"
                            failures.append(detail)
                            remember_cleanup_failure(
                                "shutdown.focus_watcher_timeout", TimeoutError(detail)
                            )

            with self._ops:
                if not self._is_current_session_locked(session):
                    return "session is no longer current; cleanup was not applied"
                if failures:
                    session.cleanup_error = "; ".join(failures)
                    session.cleanup_failure_stage = first_failure_stage
                    session.cleanup_exception = first_failure_exception
                    self._publish_session_telemetry_locked(session)
                    return session.cleanup_error

                # All shutdown work above was confirmed before relinquishing
                # ownership.  In particular, the gamepad reference is dropped
                # only after its neutral/reset sequence succeeded.
                session.mouse_listener = None
                session.hotkeys = None
                session.safety_listener = None
                session.focus_watcher = None
                session.focus_probe = None
                session.focus_reading = None
                session.deadman_held = False
                session.keyboard = None
                session.mouse = None
                session.tracker = None
                session.gamepad = None
                session.cleanup_error = None
                session.cleanup_failure_stage = None
                session.cleanup_exception = None
                session.cleanup_complete = True
                self._publish_session_telemetry_locked(session)
                return None

    def _finish_stopped_locked(self, session: RuntimeSession) -> bool:
        """Publish the one normal, confirmed terminal state for a session."""
        updates = {
            "pos": 0.0,
            "out": 0.0,
            "axis_target": 0.0,
            "axis_output": 0.0,
            "steering_mode": self.get_settings().steering_mode,
            "centering_state": "idle",
            "centering_active": False,
            "holding": False,
            "saturated": False,
            "input_source_status": "stopped",
            "cursor_warp_enabled": False,
            "deadman_held": False,
            "safety_gate_state": safety_contract.GATE_OPEN,
            "safety_gate_reason": (
                "No armed session; the safety gate is not being evaluated."
            ),
            "safety_gate_warning": None,
            "pause_cause": None,
            "pause_detail": None,
            "pause_notice": None,
            "focus_allowed": None,
            "focus_target": None,
            "indicator_label": "ENGINE STOPPED",
            "indicator_severity": safety_contract.INDICATOR_INACTIVE,
            "error": None,
        }
        if session.state == RUNTIME_STOPPED:
            return self._publish_session_telemetry_locked(session, **updates)
        return self._transition_locked(session, RUNTIME_STOPPED, **updates)

    def _is_runtime_callback_thread_locked(self, session: RuntimeSession) -> bool:
        current = threading.current_thread()
        return any(
            current is resource
            for resource in (
                session.worker,
                session.mouse_listener,
                session.hotkeys,
                session.safety_listener,
            )
        )

    def _schedule_owner_cleanup_locked(self, session: RuntimeSession) -> None:
        """Arrange owner-thread finalisation once, never from a callback itself."""
        if (
            not self._is_current_session_locked(session)
            or session.cleanup_complete
            or session.cleanup_owner_running
        ):
            return
        session.cleanup_owner_running = True
        owner = threading.Thread(
            target=self._owner_cleanup,
            args=(session,),
            daemon=True,
            name=f"steering-cleanup-owner-{session.session_id}",
        )
        session.cleanup_owner = owner
        try:
            owner.start()
        except Exception as exc:
            session.cleanup_owner = None
            session.cleanup_owner_running = False
            session.cleanup_error = f"could not schedule owner cleanup: {exc}"
            if session.state == RUNTIME_FAULTED:
                self._record_exception_locked(
                    session, "shutdown.owner_cleanup_schedule", exc
                )
                self._publish_session_telemetry_locked(
                    session,
                    error=session.fault_reason or session.cleanup_error,
                )
            else:
                self._fault_exception_locked(
                    session, "shutdown.owner_cleanup_schedule", exc
                )

    def _owner_cleanup(self, session: RuntimeSession) -> None:
        """Bounded cleanup handoff for a worker/listener-originated stop/fault."""
        try:
            self._stop_session(
                session,
                timeout=_DEFAULT_STOP_TIMEOUT_S,
                schedule_owner=False,
            )
        finally:
            with self._ops:
                if self._is_current_session_locked(session):
                    session.cleanup_owner = None
                    session.cleanup_owner_running = False

    def _set_paused_locked(
        self,
        session: RuntimeSession,
        paused: bool,
        cause: str | None = None,
        detail: str | None = None,
    ) -> bool:
        """Pause/resume one session and record *why* it is paused.

        The cause matters: a pause imposed by the operational safety guard must
        not be silently cleared by the operator's pause control, or arming a
        guard would be pointless.
        """
        if not self._is_current_session_locked(session):
            return False
        target = RUNTIME_PAUSED if paused else RUNTIME_RUNNING
        if session.state not in (RUNTIME_RUNNING, RUNTIME_PAUSED):
            return False
        if paused:
            session.pause_cause = cause or safety_contract.PAUSE_CAUSE_USER
            session.pause_detail = detail
            session.pause_notice = None
        else:
            session.pause_cause = None
            session.pause_detail = None
            session.pause_notice = None
        if session.state == target:
            return self._publish_session_telemetry_locked(
                session,
                pause_cause=session.pause_cause,
                pause_detail=session.pause_detail,
                pause_notice=session.pause_notice,
            )

        tracker = session.tracker
        if tracker is not None:
            tracker.set_paused(paused)
        return self._transition_locked(
            session,
            target,
            pending_input_events=getattr(tracker, "pending_event_count", 0),
            pause_cause=session.pause_cause,
            pause_detail=session.pause_detail,
            pause_notice=session.pause_notice,
            **source_diagnostics(tracker, self._input_backend),
        )

    def _toggle_pause_for_session(self, session: RuntimeSession) -> bool:
        with self._ops:
            if not self._is_current_session_locked(session):
                return False
            if session.state == RUNTIME_PAUSED:
                if session.pause_cause in safety_contract.SAFETY_PAUSE_CAUSES:
                    # Clearing a guard-imposed hold from the pause control would
                    # defeat the guard the operator armed. The guard releases it.
                    session.pause_notice = (
                        "Pause is held by the safety guard; "
                        + (
                            session.pause_detail
                            or "resolve the guard condition to resume."
                        )
                    )
                    self._publish_session_telemetry_locked(
                        session, pause_notice=session.pause_notice
                    )
                    return False
                return self._set_paused_locked(session, False)
            return self._set_paused_locked(
                session, True, cause=safety_contract.PAUSE_CAUSE_USER
            )

    def _stop_if_current(self, session: RuntimeSession) -> None:
        with self._ops:
            if not self._is_current_session_locked(session):
                return
        self._stop_session(session, timeout=_DEFAULT_STOP_TIMEOUT_S)

    # ----- lifecycle ---------------------------------------------------
    def start(self) -> None:
        with self._ops:
            previous = self._session
            if previous is not None:
                if self._state_is_active(previous.state) or self._worker_alive(
                    previous.worker
                ):
                    # Transition checking is intentional: an active STOPPING
                    # session cannot be overwritten by a new run.
                    return
                if not previous.cleanup_complete:
                    # A faulted/dead session is still resource owner until
                    # bounded cleanup has confirmed reference release. Never
                    # let a new run inherit or mask that uncertain ownership.
                    refusal = RuntimeError(
                        "Previous runtime cleanup is not confirmed; retry stop "
                        "and do not assume the virtual pad is disconnected."
                    )
                    diagnostic = self._record_exception_locked(
                        previous,
                        "startup.previous_cleanup",
                        refusal,
                    )
                    raise EngineStartupError(diagnostic) from refusal

            session = self._new_session_locked()
            self._session = session
            dpi_status = configure_dpi_awareness()
            self._publish_session_telemetry_locked(
                session,
                dpi_awareness_mode=dpi_status.mode,
                dpi_awareness_detail=dpi_status.detail,
                dpi_awareness_configured=dpi_status.configured,
                pos=0.0,
                out=0.0,
                axis_target=0.0,
                axis_output=0.0,
                steering_mode=self.get_settings().steering_mode,
                centering_state="idle",
                centering_active=False,
                holding=False,
                saturated=False,
                loop_dt_s=0.0,
                integration_dt_s=0.0,
                max_integration_dt_s=MAX_CONTROL_INTEGRATION_S,
                integration_lag_s=0.0,
                late_tick=False,
                late_tick_count=0,
                worst_jitter_s=0.0,
                pending_input_events=0,
                late_output_guard=False,
                input_backend=self._input_backend,
                input_source_status="starting",
                input_queue_capacity=0,
                input_dropped_events=0,
                input_packet_errors=0,
                input_source_error=None,
                input_degraded=(
                    self._input_backend == INPUT_BACKEND_CURSOR_FALLBACK
                ),
                cursor_warp_pending=False,
                cursor_warp_attempt_count=0,
                cursor_warp_failure_count=0,
                cursor_synthetic_warp_event_count=0,
                cursor_expired_warp_count=0,
                cursor_rebase_count=0,
                cursor_rebase_required=False,
                cursor_paused_discarded_event_count=0,
                cursor_warp_enabled=False,
                error=None,
                auto_return_enabled=self.get_settings().auto_return_enabled,
            )

            # Every operation below can fail for a target-PC dependency,
            # driver, listener, or platform reason. Keep the exact stage so a
            # generic startup failure never erases the useful root cause.
            startup_stage = "startup.raw_input.create_source"
            try:
                if self._input_backend == INPUT_BACKEND_RAW_INPUT:
                    # The Win32 source is lazy-imported and creates its own
                    # message-only window/thread. It never imports pynput.mouse
                    # or asks the cursor fallback to recenter.
                    factory = self._raw_input_factory or create_raw_input_source
                    startup_stage = "startup.raw_input.create_source"
                    tracker = factory()
                    session.tracker = tracker
                    startup_stage = "startup.raw_input.start_source"
                    start_source = getattr(tracker, "start", None)
                    if not callable(start_source):
                        raise RuntimeError(
                            "Raw Input factory returned a source without start()."
                        )
                    start_source()
                    startup_stage = "startup.keyboard_hotkeys.import"
                    session.keyboard = import_keyboard()
                    session.mouse = None
                else:
                    startup_stage = "startup.cursor_fallback.import_pynput"
                    session.keyboard, session.mouse = import_mouse_keyboard()
                    startup_stage = "startup.cursor_fallback.create_tracker"
                    session.tracker = RelativeMouseTracker(
                        session.mouse,
                        recenter=self._recenter,
                    )

                startup_stage = "startup.virtual_gamepad.create"
                session.gamepad = create_gamepad()
                tracker = session.tracker
                self._publish_session_telemetry_locked(
                    session,
                    **source_diagnostics(tracker, self._input_backend),
                    cursor_warp_enabled=(
                        self._recenter
                        and bool(getattr(tracker, "allows_cursor_warp", False))
                    ),
                    auto_return_enabled=self.get_settings().auto_return_enabled,
                )

                def _on_pause() -> None:
                    self._toggle_pause_for_session(session)

                def _on_stop() -> None:
                    # Capture the session identity. A late old hotkey may not
                    # signal, stop, or clean up a replacement session.
                    threading.Thread(
                        target=self._stop_if_current,
                        args=(session,),
                        daemon=True,
                        name="steering-emergency-stop",
                    ).start()

                if self._input_backend == INPUT_BACKEND_CURSOR_FALLBACK:
                    startup_stage = "startup.mouse_listener.start"
                    session.mouse_listener = session.mouse.Listener(
                        on_move=tracker.on_move
                    )
                    session.mouse_listener.start()

                startup_stage = "startup.hotkeys.create"
                # Step 22 makes the pause/stop keys configurable rather than
                # hard-coded. Values are validated by sanitise_settings, so a bad
                # binding is reported before any listener starts.
                hotkey_settings = self.get_settings()
                session.hotkeys = session.keyboard.GlobalHotKeys(
                    {
                        hotkey_settings.hotkey_pause: _on_pause,
                        hotkey_settings.hotkey_stop: _on_stop,
                    }
                )
                startup_stage = "startup.hotkeys.start"
                session.hotkeys.start()

                # Phase 4 / Step 22: build this session's operational-safety
                # runtime. The probe is constructed eagerly (no native call, so
                # it cannot fail here) and the watcher runs for the whole session
                # so arming the focus guard live takes effect without a restart.
                startup_stage = "startup.safety.probe"
                probe_factory = (
                    self._focus_probe_factory
                    or safety_contract.create_foreground_probe
                )
                session.focus_probe = probe_factory()
                session.focus_reading = None
                session.deadman_held = False
                safety_settings = self.get_settings()

                if safety_settings.deadman_enabled:
                    startup_stage = "startup.deadman_listener.start"
                    session.deadman_spec = safety_contract.parse_single_key_spec(
                        safety_settings.deadman_key
                    )

                    def _on_deadman_press(key) -> None:
                        if safety_contract.matches_key(session.deadman_spec, key):
                            session.deadman_held = True

                    def _on_deadman_release(key) -> None:
                        if safety_contract.matches_key(session.deadman_spec, key):
                            session.deadman_held = False

                    session.safety_listener = session.keyboard.Listener(
                        on_press=_on_deadman_press,
                        on_release=_on_deadman_release,
                    )
                    session.safety_listener.start()

                startup_stage = "startup.focus_watcher.start"
                session.focus_watcher = threading.Thread(
                    target=self._focus_watch,
                    args=(session,),
                    daemon=True,
                    name=f"steering-focus-watch-{session.session_id}",
                )
                session.focus_watcher.start()

                startup_stage = "startup.control_worker.start"
                session.worker = threading.Thread(
                    target=self._loop,
                    args=(session,),
                    daemon=True,
                    name=f"steering-control-loop-{session.session_id}",
                )
                # The worker reference is installed before it starts, then the
                # session becomes RUNNING as one checked transition.
                self._transition_locked(session, RUNTIME_RUNNING)
                session.worker.start()
                _register_active_engine(self)

            except Exception as exc:
                # Transactional startup: this session alone is cleaned; an old
                # callback cannot clear resources belonging to any later run.
                # Resource references remain attached if confirmation fails,
                # which blocks replacement startup until a later owner retry.
                session.stop_event.set()
                diagnostic = self._fault_exception_locked(
                    session, startup_stage, exc
                )
                self._publish_session_telemetry_locked(
                    session,
                    input_source_status=(
                        "startup_failed"
                        if self._input_backend == INPUT_BACKEND_RAW_INPUT
                        else "not_started"
                    ),
                    input_source_error=(
                        diagnostic.summary
                        if self._input_backend == INPUT_BACKEND_RAW_INPUT
                        else None
                    ),
                    cursor_warp_enabled=False,
                    error=diagnostic.summary,
                )
                # start() serializes resource construction under _ops. Joining
                # a listener that may be in a callback waiting for that lock
                # would deadlock, so hand such cleanup to an owner after this
                # transaction releases _ops. Pre-listener failures can clean
                # synchronously and retain existing startup-test determinism.
                if (
                    session.mouse_listener is None
                    and session.hotkeys is None
                    and session.safety_listener is None
                    and session.focus_watcher is None
                ):
                    cleanup_error = self._cleanup_session_resources(
                        session, _DEFAULT_STOP_TIMEOUT_S
                    )
                    if cleanup_error:
                        self._record_exception_locked(
                            session,
                            session.cleanup_failure_stage
                            or "shutdown.resource_cleanup",
                            session.cleanup_exception or RuntimeError(cleanup_error),
                        )
                else:
                    self._schedule_owner_cleanup_locked(session)
                raise EngineStartupError(diagnostic) from exc

    def _focus_watch(self, session: RuntimeSession) -> None:
        """Sample the foreground window for the life of one session.

        The watcher only records a reading on its own session object; it never
        transitions state or touches the pad. The control loop owns enforcement,
        so a slow or failing probe can delay a hold but can never steer.
        """
        interval = 1.0 / max(safety_contract.FOCUS_POLL_HZ, 1.0)
        while not session.stop_event.is_set():
            probe = session.focus_probe
            if probe is None:
                reading = safety_contract.FocusReading(
                    ok=False,
                    detail="no foreground probe is attached to this session",
                )
            else:
                try:
                    reading = probe.read()
                except Exception as exc:  # pragma: no cover - probe boundary
                    reading = safety_contract.FocusReading(
                        ok=False,
                        detail=(
                            "foreground probe raised "
                            f"{type(exc).__name__}: {exc}"
                        ),
                    )
            session.focus_reading = reading
            # A short, stop-aware sleep keeps shutdown prompt and lets a
            # disarmed guard cost almost nothing.
            session.stop_event.wait(interval)

    def _stop_session(
        self,
        session: RuntimeSession,
        timeout: float,
        *,
        schedule_owner: bool = True,
    ) -> StopResult:
        """Stop one current session and report exactly what was confirmed.

        Joining the control worker and cleanup resources happens outside the
        lifecycle lock.  This lets callbacks finish while retaining identity
        protection, and prevents a join from freezing state/telemetry updates.
        """
        timeout = max(float(timeout), 0.0)
        with self._ops:
            if not self._is_current_session_locked(session):
                return StopResult(
                    status=STOP_RESULT_ALREADY_STOPPED,
                    session_id=session.session_id,
                    lifecycle_state=session.state,
                    cleanup_complete=session.cleanup_complete,
                    detail="Session is no longer current.",
                )

            if session.state == RUNTIME_STOPPED and session.cleanup_complete:
                return self._make_stop_result_locked(
                    session,
                    STOP_RESULT_ALREADY_STOPPED,
                )

            worker = session.worker
            session.stop_event.set()
            if session.state != RUNTIME_FAULTED and session.state != RUNTIME_STOPPING:
                self._transition_locked(session, RUNTIME_STOPPING)

            # A control/listener callback can signal its session but must never
            # join itself or release resources that may still be executing.
            if self._is_runtime_callback_thread_locked(session):
                detail = "Stop requested from a runtime callback; owner cleanup pending."
                self._publish_session_telemetry_locked(session, error=detail)
                if schedule_owner:
                    self._schedule_owner_cleanup_locked(session)
                return self._make_stop_result_locked(
                    session,
                    STOP_RESULT_CALLED_FROM_WORKER,
                    detail,
                )

        if self._worker_alive(worker):
            try:
                worker.join(timeout)
            except Exception as exc:
                with self._ops:
                    if self._is_current_session_locked(session):
                        if session.state == RUNTIME_FAULTED:
                            diagnostic = self._record_exception_locked(
                                session,
                                "shutdown.control_loop_join",
                                exc,
                            )
                            detail = session.fault_reason or diagnostic.summary
                        else:
                            diagnostic = self._fault_exception_locked(
                                session,
                                "shutdown.control_loop_join",
                                exc,
                            )
                            detail = diagnostic.summary
                        return self._make_stop_result_locked(
                            session,
                            STOP_RESULT_FAULTED,
                            detail,
                        )

        with self._ops:
            if not self._is_current_session_locked(session):
                return StopResult(
                    status=STOP_RESULT_ALREADY_STOPPED,
                    session_id=session.session_id,
                    lifecycle_state=session.state,
                    cleanup_complete=session.cleanup_complete,
                    detail="Session is no longer current.",
                )
            worker_alive = self._worker_alive(worker)
            if worker_alive:
                if session.state == RUNTIME_FAULTED:
                    detail = session.fault_reason or "Control loop fault cleanup is pending."
                    if schedule_owner:
                        self._schedule_owner_cleanup_locked(session)
                    return self._make_stop_result_locked(
                        session,
                        STOP_RESULT_FAULTED,
                        detail,
                    )

                detail = "Control loop did not stop within the timeout."
                timeout_diagnostic = self._record_exception_locked(
                    session,
                    "shutdown.control_loop_timeout",
                    TimeoutError(detail),
                )
                self._publish_session_telemetry_locked(
                    session,
                    error=detail,
                    shutdown_detail=(
                        f"{detail} {timeout_diagnostic.action}"
                    ),
                )
                if schedule_owner:
                    self._schedule_owner_cleanup_locked(session)
                return self._make_stop_result_locked(
                    session,
                    STOP_RESULT_STOPPING_TIMEOUT,
                    detail,
                )

        cleanup_error = self._cleanup_session_resources(session, timeout)
        with self._ops:
            if not self._is_current_session_locked(session):
                return StopResult(
                    status=STOP_RESULT_ALREADY_STOPPED,
                    session_id=session.session_id,
                    lifecycle_state=session.state,
                    cleanup_complete=session.cleanup_complete,
                    detail="Session is no longer current.",
                )

            if cleanup_error:
                cleanup_exception = session.cleanup_exception or RuntimeError(
                    cleanup_error
                )
                cleanup_stage = (
                    session.cleanup_failure_stage or "shutdown.resource_cleanup"
                )
                if session.state != RUNTIME_FAULTED:
                    diagnostic = self._fault_exception_locked(
                        session,
                        cleanup_stage,
                        cleanup_exception,
                    )
                    detail = diagnostic.summary
                else:
                    diagnostic = self._record_exception_locked(
                        session,
                        cleanup_stage,
                        cleanup_exception,
                    )
                    detail = session.fault_reason or diagnostic.summary
                    self._publish_session_telemetry_locked(session, error=detail)
                return self._make_stop_result_locked(
                    session,
                    STOP_RESULT_FAULTED,
                    cleanup_error,
                )

            if session.state == RUNTIME_FAULTED:
                # Cleanup succeeded, but a genuine runtime fault remains part
                # of this session's truthful terminal history.
                return self._make_stop_result_locked(
                    session,
                    STOP_RESULT_FAULTED,
                    session.fault_reason,
                )

            self._finish_stopped_locked(session)
            return self._make_stop_result_locked(session, STOP_RESULT_STOPPED)

    def stop(self, timeout: float = _DEFAULT_STOP_TIMEOUT_S) -> StopResult:
        """Request a bounded stop and return a structured, truthful outcome."""
        with self._ops:
            session = self._session
        if session is None:
            self._set_telemetry(
                shutdown_status=STOP_RESULT_ALREADY_STOPPED,
                shutdown_detail=None,
            )
            return StopResult(
                status=STOP_RESULT_ALREADY_STOPPED,
                session_id=None,
                lifecycle_state=RUNTIME_STOPPED,
                cleanup_complete=True,
            )
        return self._stop_session(session, timeout)

    def set_paused(self, paused: bool) -> bool:
        with self._ops:
            session = self._session
            if session is None:
                return False
            if session.state == RUNTIME_PAUSED and session.pause_cause in (
                safety_contract.SAFETY_PAUSE_CAUSES
            ):
                # Programmatic callers get the same protection as the hotkey:
                # only the guard may release a guard-imposed hold.
                return False
            return self._set_paused_locked(
                session,
                bool(paused),
                cause=safety_contract.PAUSE_CAUSE_USER if paused else None,
            )

    def wait(self) -> None:
        with self._ops:
            session = self._session
            worker = session.worker if session is not None else None
        if worker is not None:
            worker.join()

    # ----- loop --------------------------------------------------------
    def _loop(self, session: RuntimeSession | None = None) -> None:
        """Run one session's control loop without consulting engine resources."""
        if session is None:
            with self._ops:
                session = self._session
        if session is None:
            return
        if not self._is_current_session(session):
            # A stale worker may only neutralise its own captured gamepad and
            # signal its own event; it must not touch current-engine telemetry
            # or resources.
            session.stop_event.set()
            with self._ops:
                session.worker_exited = True
            shutdown_gamepad(session.gamepad)
            return

        initial_t = time.perf_counter()
        state = SteerState(
            last_counted_t=initial_t,
            last_effective_input_t=initial_t,
        )
        next_t = initial_t
        last_wall_t = initial_t
        # ``control_t`` advances only by bounded slices. It is intentionally a
        # separate timeline from wall time while a scheduler stall is being
        # drained; tracker callback timestamps are compared against this value.
        control_t = initial_t
        last_status = 0.0
        fault: str | None = None
        late_tick_count = 0
        worst_jitter_s = 0.0
        catching_up = False
        fault_stage = "runtime.control_loop"
        # Step-22 safety accounting. ``previous_gate_open`` lets the close
        # counter count distinct holds rather than every report inside one hold.
        safety_close_count = 0
        safety_blocked_s = 0.0
        previous_gate_open = True

        try:
            while not session.stop_event.is_set():
                with self._ops:
                    if not self._is_current_session_locked(session):
                        return
                    lifecycle_state = session.state
                if lifecycle_state not in (RUNTIME_RUNNING, RUNTIME_PAUSED):
                    return

                wall_now = time.perf_counter()
                loop_dt_s = max(wall_now - last_wall_t, 0.0)
                last_wall_t = wall_now

                # One lock-protected settings copy defines this whole control
                # tick: core math, output conversion, telemetry, and cadence.
                # A GUI/preset replacement therefore takes effect atomically
                # on the next tick rather than midway through this one.
                s = self.get_settings()
                hz = s.update_hz if s.update_hz > 0.0 else 83.0
                tick_s = 1.0 / min(max(hz, 1.0), 1000.0)
                wall_timing = plan_control_interval(loop_dt_s, tick_s)
                control_timing = plan_control_interval(
                    max(wall_now - control_t, 0.0), tick_s
                )

                # A late *wall* tick increments the event count once. A
                # following catch-up report remains flagged by ``catching_up``
                # but does not inflate the count for the same stall.
                if wall_timing.late:
                    late_tick_count += 1
                worst_jitter_s = max(worst_jitter_s, abs(loop_dt_s - tick_s))
                if control_timing.late:
                    catching_up = True

                fault_stage = "runtime.input"
                tracker = session.tracker
                gamepad = session.gamepad
                paused = lifecycle_state == RUNTIME_PAUSED

                # Phase 4 / Step 22: evaluate the deadman/focus gate before any
                # input is consumed. A closed gate is enforced *as a pause*, so
                # lifecycle, telemetry, CLI, and the GUI all report the same
                # truth: the pad is neutral because a guard is holding it.
                gate = safety_contract.evaluate_safety_gate(
                    s,
                    lifecycle_state=lifecycle_state,
                    deadman_held=bool(session.deadman_held),
                    focus_reading=session.focus_reading,
                )
                if gate.applicable:
                    if not gate.open:
                        if lifecycle_state == RUNTIME_RUNNING:
                            self._set_paused_locked(
                                session,
                                True,
                                cause=gate.cause,
                                detail=gate.reason,
                            )
                            paused = True
                            if previous_gate_open:
                                safety_close_count += 1
                        elif (
                            lifecycle_state == RUNTIME_PAUSED
                            and session.pause_cause
                            in safety_contract.SAFETY_PAUSE_CAUSES
                            and (
                                session.pause_cause != gate.cause
                                or session.pause_detail != gate.reason
                            )
                        ):
                            # The hold persists but its reason changed (for
                            # example the deadman was released while the window
                            # is also unlisted). Refresh it rather than leaving
                            # telemetry describing a condition that ended.
                            self._set_paused_locked(
                                session,
                                True,
                                cause=gate.cause,
                                detail=gate.reason,
                            )
                        safety_blocked_s += loop_dt_s
                    elif (
                        lifecycle_state == RUNTIME_PAUSED
                        and session.pause_cause
                        in safety_contract.SAFETY_PAUSE_CAUSES
                    ):
                        # Only a guard-imposed hold resumes automatically; an
                        # operator pause stays paused until the operator resumes.
                        self._set_paused_locked(session, False)
                        paused = False
                    previous_gate_open = gate.open

                pending_input_events = 0
                source_info = source_diagnostics(tracker, self._input_backend)
                cursor_warp_enabled = bool(
                    tracker is not None
                    and self._recenter
                    and getattr(tracker, "allows_cursor_warp", False)
                )
                late_guard_active = False

                if paused:
                    # Paused time deliberately does not become a deferred
                    # steering interval. Drain already-arrived input through
                    # current wall time and rebase the control timeline.
                    if tracker is not None:
                        take_through = getattr(tracker, "take_through", None)
                        if callable(take_through):
                            take_through(wall_now)
                        else:  # Compatibility for a narrow third-party shim.
                            tracker.take_dx()
                        pending_input_events = getattr(
                            tracker, "pending_event_count", 0
                        )
                    control_t = wall_now
                    integration_dt_s = 0.0
                    integration_lag_s = 0.0
                    catching_up = False
                    state = SteerState()
                    axis = 0.0
                    if gamepad is not None:
                        fault_stage = "runtime.virtual_gamepad_output"
                        gamepad.left_joystick_float(
                            x_value_float=0.0,
                            y_value_float=0.0,
                        )
                        gamepad.update()
                else:
                    integration_dt_s = control_timing.integration_dt_s
                    integration_now = control_t + integration_dt_s
                    if tracker is not None:
                        take_through = getattr(tracker, "take_through", None)
                        if callable(take_through):
                            # Do not collapse all movement received during a
                            # stall into this capped interval. Newer events
                            # stay timestamped in the tracker for later slices.
                            dx = take_through(integration_now)
                        else:  # Compatibility for a narrow third-party shim.
                            dx = tracker.take_dx()
                    else:
                        dx = 0.0

                    # A late report, and every slice while it catches up, gets
                    # a hard sent-axis bound even when the profile's normal
                    # max_slew_rate is zero/unlimited.
                    late_guard_active = catching_up
                    old_axis_output = state.axis_output
                    state = step_steering(
                        state,
                        dx,
                        integration_dt_s,
                        integration_now,
                        s,
                        max_axis_step=(
                            MAX_LATE_AXIS_STEP if late_guard_active else None
                        ),
                    )
                    axis = state.axis_output
                    control_t = integration_now
                    integration_lag_s = max(wall_now - control_t, 0.0)

                    # Keep the guard active for the final partial catch-up
                    # slice, then return to ordinary configured slew behavior
                    # only once the timeline is within one nominal report.
                    if catching_up and integration_lag_s <= tick_s:
                        catching_up = False

                    if tracker is not None:
                        pending_input_events = getattr(
                            tracker, "pending_event_count", 0
                        )

                    # ``old_axis_output`` is intentionally retained in the
                    # expression so telemetry reports a real applied clamp,
                    # rather than merely a policy that happened not to bind.
                    late_output_guard = (
                        late_guard_active
                        and abs(state.axis_target - old_axis_output)
                        > MAX_LATE_AXIS_STEP + 1e-9
                        and abs(axis - old_axis_output)
                        >= MAX_LATE_AXIS_STEP - 1e-9
                    )

                    if gamepad is not None:
                        fault_stage = "runtime.virtual_gamepad_output"
                        gamepad.left_joystick_float(
                            x_value_float=axis,
                            y_value_float=0.0,
                        )
                        gamepad.update()

                if paused:
                    late_output_guard = False

                # Cursor fallback is still skipped during a bounded catch-up:
                # newer timestamped events remain owned by later control
                # slices. Step 15's target-based warp no longer clears its
                # queue, but avoiding extra desktop operations while behind
                # keeps the degraded path conservative.
                if (
                    tracker is not None
                    and cursor_warp_enabled
                    and bool(getattr(tracker, "needs_cursor_warp", True))
                    and not paused
                    and not late_guard_active
                ):
                    # RawInputDeltaSource sets allows_cursor_warp=False and
                    # does not expose warp_to_center(), so Raw Input never
                    # reaches a cursor-position API on this path. The repaired
                    # cursor fallback reports needs_cursor_warp=False after a
                    # successful/no-op/pending recenter, avoiding per-tick
                    # Controller.position writes.
                    fault_stage = "runtime.cursor_recenter"
                    tracker.warp_to_center()
                    pending_input_events = getattr(
                        tracker, "pending_event_count", 0
                    )
                    source_info = source_diagnostics(tracker, self._input_backend)

                indicator = safety_contract.input_indicator(
                    lifecycle_state=session.state,
                    gate=gate,
                    input_age_s=state.input_age_s,
                    filtered_velocity=state.filtered_velocity,
                    pause_cause=session.pause_cause,
                    input_degraded=bool(getattr(tracker, "input_degraded", False)),
                    source_error=source_info.get("input_source_error"),
                )

                fault_stage = "runtime.control_loop"
                self._publish_session_telemetry(
                    session,
                    pos=state.raw_position,
                    out=axis,
                    axis_target=state.axis_target,
                    axis_output=state.axis_output,
                    steering_mode=s.steering_mode,
                    centering_state=state.centering_state,
                    centering_active=state.centering_active,
                    holding=state.holding,
                    saturated=state.saturated,
                    raw_velocity_units_per_s=state.source_velocity,
                    filtered_velocity_units_per_s=state.filtered_velocity,
                    input_age_s=state.input_age_s,
                    loop_dt_s=loop_dt_s,
                    integration_dt_s=integration_dt_s,
                    max_integration_dt_s=control_timing.max_integration_dt_s,
                    integration_lag_s=integration_lag_s,
                    late_tick=late_guard_active,
                    late_tick_count=late_tick_count,
                    worst_jitter_s=worst_jitter_s,
                    pending_input_events=pending_input_events,
                    late_output_guard=late_output_guard,
                    safety_guard_configured=safety_contract.safety_guard_configured(
                        s
                    ),
                    safety_fail_mode=s.safety_fail_mode,
                    hotkey_pause=s.hotkey_pause,
                    hotkey_stop=s.hotkey_stop,
                    deadman_enabled=bool(s.deadman_enabled),
                    deadman_held=bool(session.deadman_held),
                    deadman_key=s.deadman_key,
                    focus_guard_enabled=bool(s.focus_guard_enabled),
                    focus_allowlist_size=len(s.focus_allowlist),
                    focus_gate_state=gate.state,
                    focus_allowed=(gate.focus.allowed if gate.focus else None),
                    focus_target=(gate.focus.target if gate.focus else None),
                    focus_probe_detail=(gate.focus.detail if gate.focus else None),
                    focus_probe_error_count=int(
                        getattr(session.focus_probe, "probe_error_count", 0) or 0
                    ),
                    safety_gate_state=gate.state,
                    safety_gate_reason=gate.reason,
                    safety_gate_warning=gate.warning,
                    safety_gate_close_count=safety_close_count,
                    safety_blocked_s=safety_blocked_s,
                    pause_cause=session.pause_cause,
                    pause_detail=session.pause_detail,
                    pause_notice=session.pause_notice,
                    indicator_label=indicator.label,
                    indicator_severity=indicator.severity,
                    **source_info,
                    cursor_warp_enabled=cursor_warp_enabled,
                    auto_return_enabled=s.auto_return_enabled,
                )

                if (
                    self._show_status
                    and hasattr(sys.stdout, "isatty")
                    and sys.stdout.isatty()
                    and wall_now - last_status >= 0.2
                ):
                    print(
                        format_status(state.pos, axis, paused).rstrip()
                        + f"  |  {indicator.label}",
                        end="",
                        flush=True,
                    )
                    last_status = wall_now

                next_t += tick_s
                delay = next_t - time.perf_counter()
                if delay > 0.0:
                    # Wake promptly on stop instead of sleeping a full tick.
                    session.stop_event.wait(delay)
                else:
                    # Do not attempt to schedule a burst of overdue callbacks.
                    # The control timeline above still drains at most one safe
                    # slice per actual report.
                    next_t = time.perf_counter()

        except Exception as exc:
            with self._ops:
                if self._is_current_session_locked(session):
                    diagnostic = self._fault_exception_locked(
                        session, fault_stage, exc
                    )
                    fault = diagnostic.summary
                else:
                    fault = (
                        f"{_humanise_diagnostic_stage(fault_stage)} failed "
                        f"({type(exc).__name__}: {exc})"
                    )
        finally:
            # Fail-safe neutralisation even if update() or another loop
            # operation raises unexpectedly.  This is this session's pad only.
            shutdown_gamepad(session.gamepad)

            session.stop_event.set()
            with self._ops:
                session.worker_exited = True
                if not self._is_current_session_locked(session):
                    return
                if fault is not None:
                    self._fault_session_locked(session, fault)
                elif session.state in (RUNTIME_RUNNING, RUNTIME_PAUSED, RUNTIME_STARTING):
                    self._fault_session_locked(
                        session,
                        "Control loop exited without an owner stop request.",
                    )
                else:
                    # STOPPING intentionally remains STOPPING until an owner
                    # confirms worker exit and releases session resources.
                    self._publish_session_telemetry_locked(session)

                # A faulting worker must never dispose itself.  Hand cleanup to
                # an owner thread, which joins this worker after it returns and
                # keeps the session FAULTED with its root cause intact.
                if session.state == RUNTIME_FAULTED and not session.cleanup_complete:
                    self._schedule_owner_cleanup_locked(session)


def format_stop_result(result: StopResult) -> str:
    """Return human-facing shutdown wording without overstating confirmation."""
    if result.status == STOP_RESULT_STOPPED:
        return "Virtual pad neutralized and disconnected after confirmed cleanup."
    if result.status == STOP_RESULT_ALREADY_STOPPED:
        return "No active runtime session; shutdown was already confirmed."
    if result.status == STOP_RESULT_STOPPING_TIMEOUT:
        return (
            "Shutdown is still pending: the virtual pad may remain active. "
            "Retry stop or wait for cleanup; forced process termination cannot "
            "confirm device cleanup."
        )
    if result.status == STOP_RESULT_CALLED_FROM_WORKER:
        return (
            "Stop was signalled from a runtime callback; owner cleanup is still "
            "pending. Do not assume the virtual pad is disconnected."
        )
    if result.status == STOP_RESULT_FAULTED:
        detail = result.detail or "Unknown runtime fault."
        if result.cleanup_confirmed:
            return (
                "Engine faulted, but runtime cleanup was confirmed and the "
                f"virtual-pad reference was released: {detail}"
            )
        return (
            "Engine faulted and cleanup is still pending; do not assume the "
            f"virtual pad is disconnected: {detail}"
        )
    return "Shutdown outcome is unknown; do not assume the virtual pad is disconnected."


# ===========================================================================
# CLI
# ===========================================================================


def run_cli(
    settings: Settings,
    input_backend: str | None = None,
) -> None:
    install_best_effort_shutdown_handlers()
    # Keep direct CLI callers inside the same early-before-pynput guarantee as
    # main(). This has no effect after main() already configured the process.
    dpi_status = configure_dpi_awareness()
    settings = sanitise_settings(settings)
    input_backend = settings.input_mode if input_backend is None else input_backend
    if input_backend not in INPUT_BACKENDS:
        raise ValueError(f"Unknown input backend: {input_backend!r}")
    l, r = travel_px(settings)
    print("=" * 62)
    print("  Mouse -> Analog Steering  (virtual Xbox 360 controller)")
    print("=" * 62)
    print(f"  DPI awareness : {dpi_status.summary}")
    input_labels = {
        INPUT_BACKEND_CURSOR_FALLBACK: (
            "DEGRADED: cursor fallback (legacy cursor coordinates)"
        ),
        INPUT_BACKEND_RAW_INPUT: "Windows Raw Input (relative HID counts; default)",
    }
    print(f"  input backend : {input_labels.get(input_backend, input_backend)}")
    source_unit = "raw counts" if input_backend == INPUT_BACKEND_RAW_INPUT else "px"
    print(
        f"  steering range : {settings.full_lock_px:.0f} {source_unit} = 100% "
        f"(travel L≈{l:.0f} / R≈{r:.0f} {source_unit})"
    )
    print(
        f"  sensitivity    : {settings.mouse_sensitivity:.2f} "
        f"(L {settings.sens_left:.2f} / R {settings.sens_right:.2f})"
    )
    curve_extra = (
        f" (exp {settings.curve_exp:g})"
        if settings.curve_preset == "Custom power"
        else ""
    )
    print(f"  curve          : {settings.curve_preset}{curve_extra}")
    normal_rate = (
        "unlimited"
        if settings.max_steering_rate == 0.0
        else f"{settings.max_steering_rate:.1f} locks/s base"
    )
    reversal_rate = (
        "inherit normal position rate"
        if settings.max_reversal_rate == 0.0
        else f"{settings.max_reversal_rate:.1f} locks/s base"
    )
    output_slew = (
        "unlimited"
        if settings.max_slew_rate == 0.0
        else f"{settings.max_slew_rate:.1f} sticks/s"
    )
    print(f"  position rate  : {normal_rate}")
    print(f"  reversal rate  : {reversal_rate}")
    print(f"  output slew    : {output_slew}")
    mode_messages = {
        "release_to_center": "release-to-centre after idle grace",
        "continuous_spring": "continuous spring / input balance",
        "manual": "manual (no automatic centering)",
    }
    print(
        f"  centering mode : {mode_messages[settings.steering_mode]}"
    )
    return_domain = (
        "axis-target"
        if settings.centering_semantics == "axis_target"
        else "raw-position"
    )
    return_time_ms = (
        settings.center_return_time_ms
        if settings.centering_semantics == "axis_target"
        else settings.center_time_s * 1000.0
    )
    print(
        f"  {return_domain} return : strength {settings.center_strength:.2f}, "
        f"{return_time_ms:.0f} ms nominal, curve {settings.center_curve:+.2f}"
    )
    print(f"  update rate    : {settings.update_hz:.0f} Hz")
    print(
        "  stall safety   : 50 ms max integration; late sent-axis change "
        "≤ 25.0%/report"
    )
    if input_backend == INPUT_BACKEND_RAW_INPUT:
        print("  raw queue      : 2048 events; overflow drops oldest (telemetry counts it)")
    else:
        print("  WARNING        : DEGRADED cursor fallback; Raw Input is the accepted default.")
        print("                   Cursor scale/capture can differ; recentering is guarded.")
    print("-" * 62)
    mode_instruction = {
        "release_to_center": (
            "Move mouse left/right; release to centre after the configured hold time."
        ),
        "continuous_spring": (
            "Move mouse left/right; input continuously balances the return spring."
        ),
        "manual": "Move mouse left/right; manually countersteer to unwind.",
    }
    print(f"  {mode_instruction[settings.steering_mode]}")
    if input_backend == INPUT_BACKEND_RAW_INPUT:
        print("  Raw Input receives relative counts; this path never recentres the cursor.")
    print("  " + safety_contract.hotkey_summary_line(settings))
    print("  Ctrl+C = quit")
    print("  guard          : " + safety_contract.safety_summary_line(settings))
    if safety_contract.safety_guard_configured(settings):
        print(
            "                   a guard hold keeps the pad neutral and cannot be "
            "cleared with the pause key"
        )
    print("  fair play      : " + safety_contract.safety_policy_notice())
    print("=" * 62, flush=True)

    engine = SteeringEngine(
        settings,
        show_status=True,
        input_backend=input_backend,
    )
    try:
        engine.start()
    except Exception as exc:
        diagnostic = getattr(exc, "diagnostic", None)
        if isinstance(diagnostic, RuntimeDiagnostic):
            message = format_runtime_diagnostic(diagnostic)
        else:
            message = (
                f"Startup failed ({type(exc).__name__}: {exc}). Retry after "
                "correcting the problem; inspect the diagnostic log if one was created."
            )
        print(message, file=sys.stderr, flush=True)
        raise SystemExit(2)

    print("Virtual Xbox 360 pad connected.\n", flush=True)
    try:
        engine.wait()
        if sys.stdout.isatty():
            print()
    except KeyboardInterrupt:
        print("\nCtrl+C -- shutting down...")
    finally:
        stop_result = engine.stop()
        telemetry = engine.telemetry()
        if telemetry.get("error"):
            print(f"Engine status: {format_telemetry_diagnostic(telemetry)}")
        if telemetry.get("input_backend") == INPUT_BACKEND_RAW_INPUT:
            print(
                "Raw Input queue: "
                f"{int(telemetry.get('input_dropped_events', 0))} event(s) dropped; "
                f"{int(telemetry.get('input_packet_errors', 0))} packet error(s)."
            )
            if telemetry.get("input_source_error"):
                print(f"Raw Input detail: {telemetry['input_source_error']}")
        elif telemetry.get("input_degraded"):
            print(
                "DEGRADED cursor fallback: "
                f"{int(telemetry.get('cursor_warp_attempt_count', 0))} recenter request(s), "
                f"{int(telemetry.get('cursor_warp_failure_count', 0))} failure(s), "
                f"{int(telemetry.get('cursor_expired_warp_count', 0))} expired target(s)."
            )
            if telemetry.get("input_source_error"):
                print(f"Cursor fallback detail: {telemetry['input_source_error']}")
        print(format_stop_result(stop_result), flush=True)


# ===========================================================================
# GUI
# ===========================================================================


# ===========================================================================
# STEP 21 — PRESENTATION CONTRACT, DIAGNOSTICS, AND SAFE LIVE APPLY
# ===========================================================================
#
# Everything in this section is deliberately free of Tk, Win32, pynput, and
# vgamepad. The driver-facing wording, the unit-bearing field labels, the
# "why is it doing that" explanations, and the live-change safety policy are
# therefore all exercisable by the deterministic suite; the Tk surface below
# only renders what these functions decide. Keeping the interpretation here
# also prevents the widget code from quietly re-deriving control semantics.

# A retargeting change (curve, rails, inversion, output deadzone) is safe to
# apply to a live session only while the sent stick is essentially neutral:
# at that point no abrupt axis movement can be produced. Above this magnitude
# the GUI refuses by default and offers an explicit, acknowledged override.
GUI_NEUTRAL_APPLY_AXIS = 0.05

GUI_APPLY_LIVE = "live"
GUI_APPLY_NEUTRAL_ONLY = "live_when_neutral"
GUI_APPLY_REQUIRES_STOP = "requires_stopped_session"
GUI_APPLY_REQUIRES_RESTART = "requires_new_session"

# Fields whose *current output value* moves discontinuously when they change
# mid-run. They are still legal changes; they are just not safe to apply while
# the car is being steered at speed.
GUI_RETARGETING_FIELDS = frozenset(
    (
        "full_lock_px",
        "lock_left",
        "lock_right",
        "curve_preset",
        "curve_exp",
        "output_deadzone",
        "invert_axis",
        "steering_mode",
    )
)

# Fields owned by a session rather than by a control tick. Changing them in a
# running session would relabel already-captured input, so they need a new one.
GUI_RESTART_FIELDS = frozenset(
    (
        "input_mode",
        "profile_semantics",
        "filter_semantics",
        "centering_semantics",
        "preset_unknown_fields",
        # Step 22: hotkey bindings and the deadman listener are created with the
        # session, so changing them mid-run would bind nothing. The rest of the
        # safety policy (guard on/off, allowlist, fail mode) is read from the
        # live settings snapshot every tick and applies immediately.
        "hotkey_pause",
        "hotkey_stop",
        "deadman_enabled",
        "deadman_key",
    )
)


def gui_source_terminology(s: Settings) -> dict[str, str]:
    """Return the active source-unit vocabulary for one settings snapshot.

    Schema v2 stores explicit units, so the label must follow the *selected*
    backend instead of inheriting legacy pixel wording. This is the single
    place the GUI obtains words like "counts" or "px".
    """
    if s.input_mode == INPUT_MODE_RAW_INPUT:
        return {
            "backend": "Raw Input",
            "distance": "Raw Input counts",
            "distance_short": "counts",
            "velocity": "counts/s",
            "filter_reference": f"{LEGACY_FILTER_REFERENCE_HZ:.0f} Hz reference",
        }
    return {
        "backend": "DEGRADED cursor fallback",
        "distance": "cursor-coordinate units",
        "distance_short": "px",
        "velocity": "px/s",
        "filter_reference": "legacy 83 Hz reference",
    }


@dataclass(frozen=True)
class GuiFieldRow:
    """One editable settings row, independent of the widget toolkit.

    ``legacy_key``/``v2_key`` let the same conceptual row edit either the
    retained schema-v1 field or the native-v2 physical field. A retained v1
    profile therefore keeps its documented 83 Hz-calibrated control, while a
    native v2 profile edits an explicit unit-bearing value instead of a value
    that silently mirrors it.
    """

    key: str
    group: str
    lo: float
    hi: float
    res: float
    fmt: str
    legacy_key: str | None = None
    v2_key: str | None = None
    v2_lo: float | None = None
    v2_hi: float | None = None
    v2_res: float | None = None
    v2_fmt: str | None = None
    boolean: bool = False
    #: A free-text row (hotkeys, allowlist rules). Text rows commit on Return
    #: or focus-out rather than on every keystroke.
    text: bool = False
    main: bool = False


GUI_ROWS: tuple[GuiFieldRow, ...] = (
    # ----- Main tab ----------------------------------------------------
    GuiFieldRow("full_lock_px", "Main", 100.0, 2000.0, 10.0, "%.0f", main=True),
    GuiFieldRow("mouse_sensitivity", "Main", 0.05, 5.0, 0.05, "%.2f", main=True),
    GuiFieldRow("curve_exp", "Main", 0.5, 5.0, 0.1, "%.1f", main=True),
    GuiFieldRow("center_strength", "Main", 0.0, 2.0, 0.05, "%.2f", main=True),
    GuiFieldRow(
        "center_time_s",
        "Main",
        0.05,
        2.0,
        0.05,
        "%.2f",
        v2_key="center_return_time_ms",
        v2_lo=50.0,
        v2_hi=2000.0,
        v2_res=10.0,
        v2_fmt="%.0f",
        main=True,
    ),
    GuiFieldRow("center_curve", "Main", -1.0, 1.0, 0.05, "%+.2f", main=True),
    # Combo-driven rows. They are part of the same contract so their label,
    # tooltip, and safe-live-apply classification cannot drift from the
    # sliders that share their semantics.
    GuiFieldRow("curve_preset", "Main", 0.0, 0.0, 0.0, "", main=True),
    GuiFieldRow("steering_mode", "Main", 0.0, 0.0, 0.0, "", main=True),
    # ----- Input scaling -----------------------------------------------
    GuiFieldRow("raw_scale", "Input scaling", 0.05, 5.0, 0.05, "%.2f"),
    GuiFieldRow("sens_left", "Input scaling", 0.05, 5.0, 0.05, "%.2f"),
    GuiFieldRow("sens_right", "Input scaling", 0.05, 5.0, 0.05, "%.2f"),
    GuiFieldRow("lock_left", "Input scaling", 0.10, 1.0, 0.01, "%.2f"),
    GuiFieldRow("lock_right", "Input scaling", 0.10, 1.0, 0.01, "%.2f"),
    # ----- Input filtering ---------------------------------------------
    GuiFieldRow(
        "smoothing",
        "Input filtering",
        0.0,
        0.95,
        0.05,
        "%.2f",
        v2_key="smoothing_tau_ms",
        v2_lo=0.0,
        v2_hi=500.0,
        v2_res=5.0,
        v2_fmt="%.0f",
    ),
    GuiFieldRow(
        "noise_gate_px",
        "Input filtering",
        0.0,
        10.0,
        0.25,
        "%.2f",
        v2_key="noise_gate_units_per_s",
        v2_lo=0.0,
        v2_hi=2000.0,
        v2_res=10.0,
        v2_fmt="%.0f",
    ),
    GuiFieldRow(
        "hysteresis_px",
        "Input filtering",
        0.0,
        20.0,
        0.5,
        "%.2f",
        v2_key="centering_hysteresis_units_per_s",
        v2_lo=0.0,
        v2_hi=4000.0,
        v2_res=10.0,
        v2_fmt="%.0f",
    ),
    GuiFieldRow("precision_zone", "Input filtering", 0.0, 0.5, 0.01, "%.2f"),
    GuiFieldRow("precision_gain", "Input filtering", 0.05, 1.0, 0.05, "%.2f"),
    # ----- Steering dynamics -------------------------------------------
    GuiFieldRow("steering_accel", "Steering dynamics", 0.0, 2.0, 0.05, "%.2f"),
    GuiFieldRow("max_steering_rate", "Steering dynamics", 0.0, 30.0, 0.5, "%.1f"),
    GuiFieldRow("max_reversal_rate", "Steering dynamics", 0.0, 30.0, 0.5, "%.1f"),
    GuiFieldRow(
        "saturate_clean", "Steering dynamics", 0.0, 0.0, 0.0, "", boolean=True
    ),
    # ----- Centering ----------------------------------------------------
    GuiFieldRow("idle_grace_ms", "Centering", 0.0, 500.0, 10.0, "%.0f"),
    # ----- Output -------------------------------------------------------
    GuiFieldRow("max_slew_rate", "Output", 0.0, 100.0, 1.0, "%.0f"),
    GuiFieldRow("output_deadzone", "Output", 0.0, 0.2, 0.005, "%.3f"),
    GuiFieldRow("invert_axis", "Output", 0.0, 0.0, 0.0, "", boolean=True),
    # ----- Timing -------------------------------------------------------
    GuiFieldRow("update_hz", "Timing", 20.0, 500.0, 1.0, "%.0f"),
    # ----- Safety (Step 22) ---------------------------------------------
    GuiFieldRow("hotkey_pause", "Safety", 0.0, 0.0, 0.0, "", text=True),
    GuiFieldRow("hotkey_stop", "Safety", 0.0, 0.0, 0.0, "", text=True),
    GuiFieldRow(
        "deadman_enabled", "Safety", 0.0, 0.0, 0.0, "", boolean=True
    ),
    GuiFieldRow("deadman_key", "Safety", 0.0, 0.0, 0.0, "", text=True),
    GuiFieldRow(
        "focus_guard_enabled", "Safety", 0.0, 0.0, 0.0, "", boolean=True
    ),
    GuiFieldRow("focus_allowlist_text", "Safety", 0.0, 0.0, 0.0, "", text=True),
    GuiFieldRow("safety_fail_mode", "Safety", 0.0, 0.0, 0.0, ""),
)

# Rows rendered as a read-only combo box rather than a slider.
GUI_COMBO_ROW_KEYS = frozenset(
    ("curve_preset", "steering_mode", "safety_fail_mode")
)

# Values offered by each combo row.
GUI_COMBO_VALUES: dict[str, tuple[str, ...]] = {
    "curve_preset": tuple(CURVE_PRESETS),
    "steering_mode": tuple(STEERING_MODES),
    "safety_fail_mode": tuple(safety_contract.SAFETY_FAIL_MODES),
}

# The pre-Step-21 GUI built its rows from ``ADV_GROUPS``. It is retained as a
# read-only compatibility view of rows that are not on the Main tab so external
# callers/tests that only enumerate groups keep working.
ADV_GROUP_NAMES: tuple[str, ...] = tuple(
    dict.fromkeys(row.group for row in GUI_ROWS if not row.main)
)


def gui_row_for(key: str) -> GuiFieldRow | None:
    for row in GUI_ROWS:
        if row.key == key:
            return row
    return None


def gui_row_field(row: GuiFieldRow, s: Settings) -> str:
    """Return the settings attribute this row edits for this snapshot."""
    if row.key == "focus_allowlist_text":
        # The allowlist is a list; the widget edits one line-based projection of
        # it. Mapping happens here so the apply path stays uniform.
        return "focus_allowlist"
    if row.boolean:
        return row.key
    if s.profile_semantics == PROFILE_SEMANTICS_V2 and row.v2_key:
        return row.v2_key
    return row.key


def gui_row_domain(row: GuiFieldRow, s: Settings) -> tuple[float, float, float, str]:
    """Return ``(lo, hi, res, fmt)`` for the field this row currently edits."""
    if (
        not row.boolean
        and s.profile_semantics == PROFILE_SEMANTICS_V2
        and row.v2_key
    ):
        return (
            row.lo if row.v2_lo is None else row.v2_lo,
            row.hi if row.v2_hi is None else row.v2_hi,
            row.res if row.v2_res is None else row.v2_res,
            row.fmt if row.v2_fmt is None else row.v2_fmt,
        )
    return row.lo, row.hi, row.res, row.fmt


_LEGACY_HYSTERESIS_CEILING_UNITS = 20.0 * LEGACY_FILTER_REFERENCE_HZ


def gui_field_label(row: GuiFieldRow, s: Settings) -> str:
    """Return a unit-bearing label for a row without inventing new semantics."""
    terms = gui_source_terminology(s)
    native_v2 = s.profile_semantics == PROFILE_SEMANTICS_V2
    labels = {
        "full_lock_px": f"Steering range ({terms['distance_short']} for 100%)",
        "mouse_sensitivity": "Mouse sensitivity",
        "curve_exp": "Curve exponent (Custom power only)",
        "center_strength": "Centring strength",
        "center_time_s": (
            f"Return time ({terms['filter_reference']}), s"
            if not native_v2
            else "Axis-target return time, ms"
        ),
        "center_curve": "Centring curve",
        "curve_preset": "Response-curve preset",
        "steering_mode": "Centring mode",
        "raw_scale": (
            f"{terms['distance_short']} delta scale"
            if not native_v2
            else "Input scale (source units)"
        ),
        "sens_left": "Sensitivity LEFT",
        "sens_right": "Sensitivity RIGHT",
        "lock_left": "Max lock LEFT",
        "lock_right": "Max lock RIGHT",
        "smoothing": (
            "Input smoothing (83 Hz coefficient)"
            if not native_v2
            else "Input smoothing time constant, ms"
        ),
        "noise_gate_px": (
            f"Noise gate ({terms['filter_reference']})"
            if not native_v2
            else f"Noise gate, {terms['velocity']}"
        ),
        "hysteresis_px": (
            f"Centring hysteresis ({terms['filter_reference']})"
            if not native_v2
            else f"Centring hysteresis, {terms['velocity']}"
        ),
        "precision_zone": "Centre-position zone",
        "precision_gain": "Centre-position gain",
        "steering_accel": "Velocity rate boost",
        "max_steering_rate": "Base position rate, locks/s",
        "max_reversal_rate": "Base reversal rate, locks/s",
        "saturate_clean": "Clean saturation",
        "idle_grace_ms": "Release-mode hold time, ms",
        "max_slew_rate": "Max output-axis slew, sticks/s",
        "output_deadzone": "Output-axis deadzone (post-curve)",
        "invert_axis": "Invert axis",
        "update_hz": "Update frequency, Hz",
        "hotkey_pause": "Pause / resume hotkey",
        "hotkey_stop": "Emergency-stop hotkey",
        "deadman_enabled": "Require a held deadman key",
        "deadman_key": "Deadman hold key",
        "focus_guard_enabled": "Auto-pause unless an allowed window has focus",
        "focus_allowlist_text": "Allowed windows (one rule per line)",
        "safety_fail_mode": "Behaviour when the foreground window cannot be read",
    }
    return labels.get(row.key, row.key)


def gui_field_hint(row: GuiFieldRow, s: Settings) -> str:
    """Return the unit-contract tooltip for a row."""
    terms = gui_source_terminology(s)
    native_v2 = s.profile_semantics == PROFILE_SEMANTICS_V2
    hints = {
        "full_lock_px": (
            f"{terms['distance']} of accepted input for a full turn; "
            "applies before the response curve"
        ),
        "mouse_sensitivity": "Steering gain; independent of the range above",
        "curve_exp": "Power used when the curve preset is Custom power",
        "center_strength": (
            "Return force scale; 0 disables automatic centring in "
            "release/continuous modes"
        ),
        "center_time_s": (
            "Raw-position return time; a Cubed output curve unwinds the "
            "visible axis faster than this number suggests"
            if not native_v2
            else "Visible axis-target return time for a full-lock target"
        ),
        "center_curve": "Stiff at lock <-> linear <-> stiff near centre",
        "curve_preset": (
            "Shape applied between raw position and the sent axis; it also "
            "changes how quickly output starts to move"
        ),
        "steering_mode": (
            "Release-to-centre waits through the hold time; continuous spring "
            "balances input against return every tick; manual never returns"
        ),
        "raw_scale": (
            "Cursor-delta scale — not raw HID scaling; affected by Windows "
            "pointer settings and Enhance Pointer Precision"
            if not native_v2
            else "Multiplies accepted source units before gain"
        ),
        "sens_left": "Steering gain while moving left",
        "sens_right": "Steering gain while moving right",
        "lock_left": "How far toward full-left the stick may reach",
        "lock_right": "How far toward full-right the stick may reach",
        "smoothing": (
            "Legacy per-report EMA coefficient calibrated at 83 Hz"
            if not native_v2
            else "Elapsed-time EMA constant — report-rate independent"
        ),
        "noise_gate_px": (
            "Legacy per-report threshold; compared in velocity units"
            if not native_v2
            else "Input below this source velocity is rejected as noise"
        ),
        "hysteresis_px": (
            "Legacy per-report threshold; re-engagement gate for centring"
            if not native_v2
            else "Filtered input must exceed this to be accepted"
        ),
        "precision_zone": (
            "Occupied-position region near centre; a position zone, not a "
            "low-speed filter, so it also slows a full-lock flick"
        ),
        "precision_gain": "Input scale while inside the centre-position zone",
        "steering_accel": (
            "Scales both the requested position rate and its base cap; the "
            "output-axis slew limiter is separate"
        ),
        "max_steering_rate": "Normal raw-position base cap; 0 = unlimited",
        "max_reversal_rate": (
            "Countersteer base cap; 0 = inherit the normal position rate"
        ),
        "saturate_clean": (
            "Movement pushing further into an engaged rail leaves no trace"
        ),
        "idle_grace_ms": (
            "Release mode: effective input must stop for this long before "
            "returning; ignored by continuous/manual"
        ),
        "max_slew_rate": "Sent-stick rate cap; 0 = unlimited",
        "output_deadzone": (
            "Post-curve threshold; the remaining axis range is remapped "
            "continuously; 0 = disabled"
        ),
        "invert_axis": "Reverse if the game turns the wrong way",
        "update_hz": "Virtual gamepad report rate",
        "hotkey_pause": (
            "Canonical names such as <f8> or <ctrl>+p; applies to the next "
            "session"
        ),
        "hotkey_stop": "Canonical names such as <f9>; applies to the next session",
        "deadman_enabled": (
            "Steering leaves neutral only while the deadman key is held; takes "
            "effect on the next session"
        ),
        "deadman_key": (
            "A single held key such as <f10>, <shift> or x; takes effect on the "
            "next session"
        ),
        "focus_guard_enabled": (
            "Auto-pauses the session (pad neutral) unless the foreground window "
            "matches a rule below; needs at least one rule"
        ),
        "focus_allowlist_text": (
            "proc:name.exe matches the process, title:text matches the window "
            "title, and a bare rule matches either; separate rules with ';'"
        ),
        "safety_fail_mode": (
            "fail_closed holds steering when the foreground window cannot be "
            "read; fail_open keeps steering and warns"
        ),
    }
    return hints.get(row.key, "")


@dataclass(frozen=True)
class GuiApplyDecision:
    """Result of the safe-live-apply policy for one prospective edit."""

    key: str
    field: str
    kind: str
    allowed: bool
    reason: str
    requires_acknowledgement: bool = False

    @property
    def is_override(self) -> bool:
        return self.allowed and self.requires_acknowledgement


def gui_live_apply_policy(
    key: str,
    settings: Settings,
    telemetry: dict[str, Any],
    *,
    acknowledge_abrupt: bool = False,
) -> GuiApplyDecision:
    """Decide whether one setting may change live, and say why.

    The engine already applies a whole validated settings snapshot atomically
    at the next tick and clamps state to new rails, so this policy is about
    *driving feel and honesty*, not core correctness: a retargeting change at
    speed moves the sent stick immediately, and a session-owned field cannot be
    reinterpreted without restarting input capture.
    """
    row = gui_row_for(key)
    field = gui_row_field(row, settings) if row is not None else key
    state = str(telemetry.get("lifecycle_state", RUNTIME_STOPPED))
    active = state in _RUNTIME_ACTIVE_STATES

    if field in GUI_RESTART_FIELDS:
        if active:
            return GuiApplyDecision(
                key,
                field,
                GUI_APPLY_REQUIRES_RESTART,
                False,
                f"{field} is captured when a session starts; stop the session "
                "and start it again instead of changing it mid-run.",
            )
        return GuiApplyDecision(
            key, field, GUI_APPLY_REQUIRES_RESTART, True, "Applies on next start."
        )

    if field not in GUI_RETARGETING_FIELDS:
        return GuiApplyDecision(
            key, field, GUI_APPLY_LIVE, True, "Applies on the next control tick."
        )

    if not active:
        return GuiApplyDecision(
            key,
            field,
            GUI_APPLY_NEUTRAL_ONLY,
            True,
            "No armed session; applies immediately.",
        )

    # A paused session discards input and holds the pad neutral, so the change
    # cannot produce a visible steering jump.
    if state == RUNTIME_PAUSED:
        return GuiApplyDecision(
            key,
            field,
            GUI_APPLY_NEUTRAL_ONLY,
            True,
            "Session is paused with the pad held neutral; applying now is safe.",
        )

    if telemetry.get("late_tick"):
        return GuiApplyDecision(
            key,
            field,
            GUI_APPLY_NEUTRAL_ONLY,
            False,
            "The control loop is catching up after a late report; wait for the "
            "guard to clear before retargeting the sent axis.",
        )

    sent = abs(_gui_number(telemetry, "axis_output", _gui_number(telemetry, "out")))
    if sent <= GUI_NEUTRAL_APPLY_AXIS:
        return GuiApplyDecision(
            key,
            field,
            GUI_APPLY_NEUTRAL_ONLY,
            True,
            f"Sent axis is neutral ({sent:+.1%}); retargeting now cannot jerk "
            "the stick.",
        )

    if acknowledge_abrupt:
        return GuiApplyDecision(
            key,
            field,
            GUI_APPLY_NEUTRAL_ONLY,
            True,
            f"Acknowledged override at a sent axis of {sent:+.1%}; the stick "
            "may move abruptly.",
            requires_acknowledgement=True,
        )

    return GuiApplyDecision(
        key,
        field,
        GUI_APPLY_NEUTRAL_ONLY,
        False,
        f"{field} changes the desired output immediately and the sent axis is "
        f"{sent:+.1%}. Return to centre (or stop the session) and apply again.",
    )


def gui_close_window_decision(
    stop_result: StopResult | None,
    telemetry: dict[str, Any],
) -> tuple[bool, str]:
    """Say whether closing the window is allowed to destroy the retry surface.

    Destroying the window on an unconfirmed stop would present an unverified
    cleanup as a normal, safe disconnect and would remove the only visible
    place to retry it. This stays Tk-free so the policy is testable.
    """
    if stop_result is not None:
        if stop_result.cleanup_confirmed:
            return True, format_stop_result(stop_result)
        return (
            False,
            f"{format_stop_result(stop_result)} The window stays open so "
            "Stop/cleanup can be retried; closing now would not confirm that "
            "the virtual pad was released.",
        )

    state = str(telemetry.get("lifecycle_state", RUNTIME_STOPPED))
    cleanup_complete = bool(telemetry.get("session_cleanup_complete", True))
    if state == RUNTIME_STOPPED and cleanup_complete:
        return True, "No active session; cleanup was already confirmed."
    return (
        False,
        "Runtime cleanup is not confirmed "
        f"(state {state}). Use Stop/Retry cleanup before closing the window.",
    )


def _gui_mode_label(mode: str) -> str:
    return {
        "release_to_center": "Release-to-centre",
        "continuous_spring": "Continuous spring",
        "manual": "Manual",
    }.get(mode, mode)


def gui_centering_explanation(
    settings: Settings, telemetry: dict[str, Any]
) -> str:
    """Explain *why* the car is (or is not) centring right now.

    A safety-guard hold is reported before the ordinary pause wording, because
    "paused" alone would hide the reason the driver most needs to see.
    """
    state = str(telemetry.get("lifecycle_state", RUNTIME_STOPPED))
    mode = resolve_steering_mode(settings)
    if state in (RUNTIME_STOPPED, RUNTIME_STARTING):
        return "No running session, so nothing is being sent to the pad."
    if telemetry.get("pause_cause") in safety_contract.SAFETY_PAUSE_CAUSES:
        return (
            "Held by the operational safety guard, so centring is not being "
            f"evaluated: {telemetry.get('safety_gate_reason')}"
        )
    if state == RUNTIME_PAUSED:
        return "Paused: the pad is held neutral and centring is not being evaluated."
    if mode == "manual":
        return (
            "Manual mode: no automatic return exists. The car keeps its "
            "position until countersteering unwinds it."
        )

    centring = str(telemetry.get("centering_state", "idle"))
    position = _gui_number(telemetry, "pos")
    age_ms = max(0.0, _gui_number(telemetry, "input_age_s")) * 1000.0
    strength = settings.center_strength
    return_time_ms = (
        settings.center_return_time_ms
        if settings.centering_semantics == CENTERING_SEMANTICS_AXIS_TARGET
        else settings.center_time_s * 1000.0
    )
    if strength <= 0.0 or return_time_ms <= 0.0:
        return (
            "Centring is disabled by its own settings (strength or return "
            "time is zero) in the active mode."
        )
    if centring == "centering":
        return (
            f"Returning to centre now ({_gui_mode_label(mode)}): raw position "
            f"{position:+.3f}, nominal return {return_time_ms:.0f} ms."
        )
    if centring == "waiting":
        grace = settings.idle_grace_ms
        return (
            f"Waiting: accepted input is only {age_ms:.0f} ms old and release "
            f"mode holds steering for {grace:.0f} ms before returning."
        )
    if centring == "input" and mode == "continuous_spring":
        return (
            "Continuous spring is running, but accepted input currently "
            "balances it, so the position is holding rather than returning."
        )
    if centring == "disabled":
        return "Centring is disabled in this mode."
    if position == 0.0:
        return "At centre; there is nothing to return."
    return (
        f"No return force is being applied in the active {_gui_mode_label(mode)} "
        "state."
    )


def gui_input_gate_explanation(
    settings: Settings, telemetry: dict[str, Any]
) -> str:
    """Explain *why* input is accepted, gated, or ignored right now."""
    state = str(telemetry.get("lifecycle_state", RUNTIME_STOPPED))
    if state == RUNTIME_STOPPED:
        return "The engine is stopped, so no input is being read."
    if state == RUNTIME_STARTING:
        return "Starting: input capture is being created."
    if telemetry.get("pause_cause") in safety_contract.SAFETY_PAUSE_CAUSES:
        return (
            "Held by the operational safety guard, so movement is discarded and "
            f"the pad stays neutral: {telemetry.get('safety_gate_reason')}"
        )
    if state == RUNTIME_PAUSED:
        return "Paused: movement is discarded and the pad is held neutral."
    if state in (RUNTIME_STOPPING,):
        return "Stopping: input is no longer being applied to the pad."

    source = _gui_number(telemetry, "raw_velocity_units_per_s")
    filtered = _gui_number(telemetry, "filtered_velocity_units_per_s")
    source_status = str(telemetry.get("input_source_status", "unknown"))
    if telemetry.get("input_backend") == INPUT_BACKEND_CURSOR_FALLBACK:
        if telemetry.get("cursor_rebase_required"):
            return (
                "DEGRADED cursor fallback is waiting for a safe pointer "
                "baseline, so it is deliberately discarding deltas."
            )
    if source_status not in ("running", "degraded_cursor_fallback", "stopped"):
        return f"Input source status is {source_status!r}."

    tau_ms = (
        settings.smoothing_tau_ms
        if settings.filter_semantics == FILTER_SEMANTICS_V2
        else 0.0
    )
    gate = (
        settings.noise_gate_units_per_s
        if settings.filter_semantics == FILTER_SEMANTICS_V2
        else settings.noise_gate_px * LEGACY_FILTER_REFERENCE_HZ
    )
    hysteresis = (
        settings.centering_hysteresis_units_per_s
        if settings.filter_semantics == FILTER_SEMANTICS_V2
        else settings.hysteresis_px * LEGACY_FILTER_REFERENCE_HZ
    )
    unit = gui_source_terminology(settings)["velocity"]
    if abs(source) < gate:
        return (
            f"Rejected by the noise gate: source {source:+.0f} {unit} is below "
            f"the {gate:+.0f} {unit} threshold."
        )
    if filtered == 0.0 and hysteresis > 0.0:
        return (
            f"Gated after filtering: the smoothed value is below the "
            f"{hysteresis:+.0f} {unit} hysteresis threshold"
            + (
                f"; the filter has not built up yet (time constant {tau_ms:.0f} ms)."
                if tau_ms > 0.0
                else "."
            )
        )
    if filtered == 0.0:
        return (
            f"No movement is reaching the core (source {source:+.0f} {unit}); "
            "the mouse is stationary or its deltas are being discarded."
        )
    age_ms = max(0.0, _gui_number(telemetry, "input_age_s")) * 1000.0
    return (
        f"Accepted: filtered {filtered:+.0f} {unit}, last effective input "
        f"{age_ms:.0f} ms ago."
    )


def gui_pad_connection_state(telemetry: dict[str, Any]) -> str:
    """State whether the virtual pad is known to exist, and how confidently."""
    state = str(telemetry.get("lifecycle_state", RUNTIME_STOPPED))
    cleanup_complete = bool(telemetry.get("session_cleanup_complete", True))
    session_id = telemetry.get("session_id")
    rate = _gui_number(telemetry, "loop_dt_s")
    rate_text = f" at ~{1.0 / rate:.0f} Hz" if rate > 0.0 else ""
    if state in (RUNTIME_RUNNING, RUNTIME_PAUSED):
        return (
            f"Connected: virtual Xbox 360 pad is being driven by session "
            f"{session_id}{rate_text}."
        )
    if state == RUNTIME_STARTING:
        return "Not confirmed: the virtual pad is being created."
    if state == RUNTIME_STOPPING:
        return (
            "Not confirmed: shutdown is in progress — the pad may still be "
            "connected. Do not assume it was released."
        )
    if state == RUNTIME_FAULTED:
        if cleanup_complete:
            return (
                "Released: the session faulted, but cleanup was confirmed and "
                "the pad reference was dropped."
            )
        return (
            "Not confirmed: the session faulted and cleanup is still pending — "
            "the pad may still be connected."
        )
    if cleanup_complete:
        return "Disconnected: no session owns a virtual pad."
    return "Not confirmed: the last session's cleanup was not verified."


@dataclass(frozen=True)
class GuiRuntimeView:
    """Pure, Tk-independent presentation data for the Step-21 GUI surface.

    The UI must not infer source units, lifecycle truth, or a safety policy from
    widget state.  Keeping that interpretation here lets tests exercise the
    driver-facing language without constructing a display server/Tk root.
    """

    input_source: str
    steering: str
    profile: str
    safety: str
    lifecycle: str
    diagnostics: tuple[tuple[str, str], ...]
    fault: str | None
    # Step-21 additions. They are defaulted so any caller that constructed a
    # view positionally against the earlier, narrower contract still works.
    lifecycle_state: str = RUNTIME_STOPPED
    cleanup_confirmed: bool = True
    input_backend: str = ""
    safety_guard_configured: bool = False
    centering_reason: str = ""
    input_gate_reason: str = ""
    pad_connection: str = ""
    late_guard_reason: str = ""
    session_label: str = ""
    input_indicator: Any = field(
        default_factory=lambda: safety_contract.input_indicator(
            lifecycle_state=RUNTIME_STOPPED,
            gate=None,
            input_age_s=0.0,
            filtered_velocity=0.0,
        )
    )
    safety_gate_state: str = safety_contract.GATE_OPEN
    safety_gate_warning: str | None = None
    pause_notice: str | None = None


def _gui_gate_from_telemetry(telemetry: dict[str, Any]) -> Any:
    """Rebuild the engine's gate decision from published telemetry.

    The GUI never re-evaluates the safety policy: the control loop already
    decided, and this only re-presents that decision. Re-deriving it here would
    risk the banner disagreeing with the pad.
    """
    state = str(telemetry.get("safety_gate_state", safety_contract.GATE_OPEN))
    return safety_contract.GateDecision(
        state=state,
        open=state == safety_contract.GATE_OPEN,
        cause=telemetry.get("pause_cause"),
        reason=str(
            telemetry.get("safety_gate_reason") or "The safety gate is open."
        ),
    )


def _gui_safety_blocked_text(telemetry: dict[str, Any]) -> str:
    """Return the "blocked for N s over M holds" suffix, or an empty string."""
    holds = int(_gui_number(telemetry, "safety_gate_close_count"))
    if holds <= 0:
        return ""
    return (
        f"; blocked {_gui_number(telemetry, 'safety_blocked_s'):.1f} s "
        f"over {holds} hold(s)"
    )


def _gui_number(telemetry: dict[str, Any], key: str, default: float = 0.0) -> float:
    """Read an optional telemetry number defensively for a presentation view."""
    try:
        value = float(telemetry.get(key, default))
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


def build_gui_runtime_view(
    settings: Settings,
    telemetry: dict[str, Any],
    *,
    profile_label: str | None = None,
) -> GuiRuntimeView:
    """Describe runtime facts for the persistent header/Diagnostics tab.

    This function performs no Tk, platform, input, or gamepad work. It reports
    the operational-safety policy as the engine published it (Step 22): the gate
    state, the reason for any hold, and the indicator banner. It never decides
    the policy itself — it re-presents telemetry, so the banner can never
    disagree with what the virtual pad is actually receiving.
    """
    backend = str(telemetry.get("input_backend", settings.input_mode))
    source_status = str(telemetry.get("input_source_status", "not_started"))
    if backend == INPUT_BACKEND_RAW_INPUT:
        input_source = f"Raw Input — relative HID counts ({source_status})"
        velocity_unit = "counts/s"
    elif backend == INPUT_BACKEND_CURSOR_FALLBACK:
        input_source = (
            "DEGRADED cursor fallback — cursor-coordinate deltas "
            f"({source_status})"
        )
        velocity_unit = "cursor units/s"
    else:
        input_source = f"Unknown input backend {backend!r} ({source_status})"
        velocity_unit = "source units/s"

    mode_labels = {
        "release_to_center": "Release-to-centre",
        "continuous_spring": "Continuous spring",
        "manual": "Manual",
    }
    mode = str(telemetry.get("steering_mode", settings.steering_mode))
    centering = str(telemetry.get("centering_state", "idle")).replace("_", " ")
    steering = f"{mode_labels.get(mode, mode)} — centering: {centering}"

    semantics = {
        "v2": "Native schema-v2 semantics",
        "legacy_v1": "Legacy-compatible semantics",
    }.get(settings.profile_semantics, f"Unknown semantics {settings.profile_semantics!r}")
    profile = f"{profile_label or 'Direct settings (no profile path)'} — {semantics}"

    state = str(telemetry.get("lifecycle_state", RUNTIME_STOPPED))
    cleanup_complete = bool(telemetry.get("session_cleanup_complete", True))
    lifecycle_labels = {
        RUNTIME_STOPPED: (
            "Stopped — cleanup confirmed"
            if cleanup_complete
            else "Stopped — cleanup status unavailable"
        ),
        RUNTIME_STARTING: "Starting",
        RUNTIME_RUNNING: "Running",
        RUNTIME_PAUSED: "Paused",
        RUNTIME_STOPPING: "Stopping — cleanup not yet confirmed",
        RUNTIME_FAULTED: (
            "Faulted — cleanup confirmed"
            if cleanup_complete
            else "Faulted — cleanup pending"
        ),
    }
    lifecycle = lifecycle_labels.get(state, f"Unknown lifecycle state {state!r}")

    raw_velocity = _gui_number(telemetry, "raw_velocity_units_per_s")
    filtered_velocity = _gui_number(telemetry, "filtered_velocity_units_per_s")
    input_age_s = max(0.0, _gui_number(telemetry, "input_age_s"))
    loop_dt_s = _gui_number(telemetry, "loop_dt_s")
    measured_hz = (1.0 / loop_dt_s) if loop_dt_s > 1e-9 else 0.0
    diagnostics: list[tuple[str, str]] = [
        (
            "Source velocity",
            f"{raw_velocity:+.1f} {velocity_unit} (pre-gate)",
        ),
        (
            "Accepted filtered velocity",
            f"{filtered_velocity:+.1f} {velocity_unit}",
        ),
        ("Effective-input age", f"{input_age_s * 1000.0:.0f} ms"),
        ("Raw position", f"{_gui_number(telemetry, 'pos'):+.3f}"),
        ("Axis target", f"{_gui_number(telemetry, 'axis_target'):+.1%}"),
        (
            "Sent axis",
            f"{_gui_number(telemetry, 'axis_output', _gui_number(telemetry, 'out')):+.1%}"
            " — value actually written to the pad",
        ),
        (
            "Actual loop rate",
            (
                f"{measured_hz:.1f} Hz (interval {loop_dt_s * 1000.0:.1f} ms; "
                f"configured {settings.update_hz:.0f} Hz)"
                if measured_hz > 0.0
                else "not measured yet"
            ),
        ),
        (
            "Worst jitter",
            f"{_gui_number(telemetry, 'worst_jitter_s') * 1000.0:.1f} ms"
            f"; late reports {int(_gui_number(telemetry, 'late_tick_count'))}",
        ),
        (
            "Scheduler slice",
            (
                f"integrating {_gui_number(telemetry, 'integration_dt_s') * 1000.0:.1f} ms"
                f" of {_gui_number(telemetry, 'max_integration_dt_s', MAX_CONTROL_INTEGRATION_S) * 1000.0:.0f} ms max"
            ),
        ),
    ]
    if backend == INPUT_BACKEND_RAW_INPUT:
        diagnostics.append(
            (
                "Raw Input queue",
                (
                    f"{int(_gui_number(telemetry, 'pending_input_events'))}/"
                    f"{int(_gui_number(telemetry, 'input_queue_capacity'))} events; "
                    f"dropped {int(_gui_number(telemetry, 'input_dropped_events'))}"
                    f"; packet errors {int(_gui_number(telemetry, 'input_packet_errors'))}"
                ),
            )
        )
    elif backend == INPUT_BACKEND_CURSOR_FALLBACK:
        diagnostics.append(
            (
                "Cursor fallback",
                (
                    f"recenter attempts {int(_gui_number(telemetry, 'cursor_warp_attempt_count'))}; "
                    f"failures {int(_gui_number(telemetry, 'cursor_warp_failure_count'))}; "
                    f"expired {int(_gui_number(telemetry, 'cursor_expired_warp_count'))}"
                ),
            )
        )
    if telemetry.get("input_source_error"):
        diagnostics.append(("Input source error", str(telemetry["input_source_error"])))
    if telemetry.get("late_tick"):
        diagnostics.append(
            (
                "Late-report guard",
                f"active; lag {_gui_number(telemetry, 'integration_lag_s') * 1000.0:.0f} ms",
            )
        )
    # Same reconstruction the runtime view uses: this function is called by the
    # view and by the diagnostics tab, and must not re-derive the policy.
    gate = _gui_gate_from_telemetry(telemetry)
    guard_configured = safety_contract.safety_guard_configured(settings)
    diagnostics.append(
        (
            "Safety gate",
            f"{gate.state}"
            + (f" — {gate.reason}" if gate.held else "")
            + _gui_safety_blocked_text(telemetry),
        )
    )
    if guard_configured:
        diagnostics.append(
            (
                "Deadman key",
                (
                    f"{settings.deadman_key} "
                    f"({'held' if telemetry.get('deadman_held') else 'not held'})"
                    if settings.deadman_enabled
                    else "not armed"
                ),
            )
        )
    if settings.focus_guard_enabled:
        diagnostics.append(
            (
                "Focus target",
                (
                    str(telemetry.get("focus_target") or "not sampled yet")
                    + f" ({len(settings.focus_allowlist)} rule(s),"
                    f" {settings.safety_fail_mode})"
                ),
            )
        )
    diagnostics.append(
        (
            "Session",
            (
                f"id {telemetry.get('session_id')}"
                f"; shutdown {telemetry.get('shutdown_status', STOP_RESULT_NOT_REQUESTED)}"
            ),
        )
    )

    fault = None
    if telemetry.get("error_type") or telemetry.get("error"):
        fault = format_telemetry_diagnostic(telemetry)
    if telemetry.get("diagnostic_log_path") and fault:
        fault = f"{fault} (log: {telemetry['diagnostic_log_path']})"

    mode_label = _gui_mode_label(mode)

    gate = _gui_gate_from_telemetry(telemetry)
    guard_configured = safety_contract.safety_guard_configured(settings)
    if gate.held:
        safety = f"HOLDING STEERING — {gate.reason}"
    else:
        safety = safety_contract.safety_summary_line(settings)
    warning = telemetry.get("safety_gate_warning")
    if warning:
        safety = f"{safety} WARNING: {warning}"

    indicator = safety_contract.input_indicator(
        lifecycle_state=state,
        gate=gate,
        input_age_s=input_age_s,
        filtered_velocity=filtered_velocity,
        pause_cause=telemetry.get("pause_cause"),
        input_degraded=bool(telemetry.get("input_degraded")),
        source_error=telemetry.get("input_source_error"),
    )
    return GuiRuntimeView(
        input_source=input_source,
        steering=f"{mode_label} — centring: {centering}",
        profile=profile,
        safety=safety,
        lifecycle=lifecycle,
        diagnostics=tuple(diagnostics),
        fault=fault,
        lifecycle_state=state,
        cleanup_confirmed=cleanup_complete,
        input_backend=backend,
        safety_guard_configured=guard_configured,
        input_indicator=indicator,
        safety_gate_state=gate.state,
        safety_gate_warning=warning,
        pause_notice=telemetry.get("pause_notice"),
        centering_reason=gui_centering_explanation(settings, telemetry),
        input_gate_reason=gui_input_gate_explanation(settings, telemetry),
        pad_connection=gui_pad_connection_state(telemetry),
        late_guard_reason=(
            "A late/catch-up report is limiting sent-axis movement to "
            f"{MAX_LATE_AXIS_STEP:.0%} per report until the control timeline "
            "catches up."
            if telemetry.get("late_tick")
            else "No late-report guard is active."
        ),
        session_label=(
            "No session"
            if telemetry.get("session_id") is None
            else f"session {telemetry.get('session_id')} — "
            + (
                "cleanup confirmed"
                if cleanup_complete
                else "cleanup NOT confirmed"
            )
        ),
    )


def gui_settings_snapshot_replace_permission(
    telemetry: dict[str, Any],
) -> tuple[bool, str]:
    """Say whether a Load preset/Defaults replacement is safe *right now*.

    Live single-field tuning already uses the engine's validated next-tick
    snapshots. Replacing every setting at once is different: it may switch
    profile semantics/locks/curves abruptly, so it is deliberately deferred
    until no session is armed and no cleanup is pending.
    """
    state = str(telemetry.get("lifecycle_state", RUNTIME_STOPPED))
    cleanup_complete = bool(telemetry.get("session_cleanup_complete", True))
    if state == RUNTIME_STOPPED:
        return True, ""
    if state == RUNTIME_FAULTED and cleanup_complete:
        return True, ""
    if state == RUNTIME_FAULTED:
        return (
            False,
            "Cannot replace all settings while fault cleanup is pending. Retry cleanup first.",
        )
    if state == RUNTIME_STOPPING:
        return (
            False,
            "Cannot replace all settings while shutdown is pending; do not assume the virtual pad is disconnected.",
        )
    if state in _RUNTIME_ACTIVE_STATES:
        return (
            False,
            "Stop the active session before loading a profile or resetting all settings.",
        )
    return False, f"Cannot replace all settings in lifecycle state {state!r}."


def _legacy_adv_groups() -> list[tuple[str, list]]:
    """Deprecated read-only view of the pre-Step-21 advanced row list.

    The Tk surface now builds every row from :data:`GUI_ROWS` so a row's label,
    unit, range, and bindings come from one place. This adapter is retained only
    so external tooling that enumerates the old ``(key, label, lo, hi, res,
    hint, fmt)`` shape keeps working; it is rendered against a legacy-default
    snapshot, so its wording is the retained-cursor vocabulary and must not be
    used to label a native-v2 profile.
    """
    reference = Settings()
    grouped: dict[str, list] = {name: [] for name in ADV_GROUP_NAMES}
    for row in GUI_ROWS:
        if row.main or row.group not in grouped:
            continue
        if row.text or row.key in GUI_COMBO_ROW_KEYS:
            # Text and combo rows have no ``(key, label, lo, hi, res, hint,
            # fmt)`` equivalent; they are intentionally absent from this
            # deprecated slider-shaped view.
            continue
        label = gui_field_label(row, reference)
        hint = gui_field_hint(row, reference)
        if row.boolean:
            grouped[row.group].append(("bool", row.key, label, hint))
        else:
            grouped[row.group].append(
                (row.key, label, row.lo, row.hi, row.res, hint, row.fmt)
            )
    return [(name, grouped[name]) for name in ADV_GROUP_NAMES]


ADV_GROUPS: list[tuple[str, list]] = _legacy_adv_groups()

#: Indicator banner colours per severity. ``foreground`` is what every theme
#: honours; ``background`` is applied as well where the theme supports it, and
#: the banner also carries bold text plus a ridge so it is visible regardless.
GUI_INDICATOR_COLOURS: dict[str, tuple[str, str]] = {
    safety_contract.INDICATOR_ACTIVE: ("#0a5a2a", "#e6ffef"),
    safety_contract.INDICATOR_DEGRADED: ("#8a5200", "#fff4e0"),
    safety_contract.INDICATOR_HELD: ("#8a5200", "#fff4e0"),
    safety_contract.INDICATOR_INACTIVE: ("#444444", "#f0f0f0"),
    safety_contract.INDICATOR_FAULT: ("#a31b1b", "#ffecec"),
}



def run_gui(
    settings: Settings,
    input_backend: str | None = None,
    profile_label: str | None = None,
    calibration_runner_factory: Callable[..., "CalibrationRunner"] | None = None,
) -> None:
    install_best_effort_shutdown_handlers()
    # This call intentionally precedes the tkinter import and root creation.
    # main() reaches it first in normal launches; this covers direct callers.
    dpi_status = configure_dpi_awareness()
    settings = sanitise_settings(settings)
    input_backend = settings.input_mode if input_backend is None else input_backend
    if input_backend not in INPUT_BACKENDS:
        raise ValueError(f"Unknown input backend: {input_backend!r}")
    try:
        import tkinter as tk
        from tkinter import filedialog, messagebox, ttk
    except ImportError:  # pragma: no cover
        sys.exit("tkinter is not available -- install it or use --cli mode.")

    engine = SteeringEngine(
        settings,
        show_status=False,
        input_backend=input_backend,
    )

    class SteeringApp(tk.Tk):
        def __init__(self) -> None:
            super().__init__()
            self.title("Mouse -> Analog Steering")
            self.resizable(False, False)
            self.engine = engine
            self.vars: dict[str, tuple[Any, str | None]] = {}
            self.labels: dict[str, Any] = {}
            self.row_widgets: dict[str, tuple[Any, Any, Any]] = {}
            self.combos: dict[str, Any] = {}
            self.combo_vars: dict[str, Any] = {}
            self._in_update = False
            # A refusal/stop/fault message is held on the status bar for a few
            # seconds so the 50 ms poll cannot erase the explanation.
            self._status_hold_until = 0.0
            self._close_pending = False
            # Step 23: the calibration flow runs in the GUI thread, driven by the
            # existing 50 ms poll. It is a view of the pure state machine.
            self.cal_runner: CalibrationRunner | None = None
            self.cal_suggestion: Any = None
            self.cal_output_peak: float = 0.0
            # Production builds the real input-only runner; a test injects one
            # with a scripted source and a controlled clock, which is what makes
            # the GUI's calibration path verifiable without a mouse.
            self.cal_runner_factory = calibration_runner_factory

            self._build_header()
            self._build_meter()
            self._build_notebook()
            self._build_buttons()
            self._build_statusbar()
            self.protocol("WM_DELETE_WINDOW", self._on_close)
            self.after(50, self._poll)

        def set_field(self, key: str, value) -> None:
            """Apply a field without the live-apply policy (compat helper)."""
            try:
                self.engine.set_field(key, value)
            except (AttributeError, ValueError) as exc:
                self.status.config(text=str(exc))

        def _hold_status(self, text: str, seconds: float = 4.0) -> None:
            """Show a message the periodic poll must not immediately erase."""
            self._status_hold_until = time.monotonic() + max(seconds, 0.0)
            self.status.config(text=text)

        def _revert_widget(self, row: GuiFieldRow) -> None:
            """Restore one row's widget to the value the engine actually holds."""
            settings = self.engine.get_settings()
            pair = self.vars.get(row.key)
            if pair is None:
                return
            var, fmt = pair
            self._in_update = True
            try:
                if row.boolean:
                    var.set(bool(getattr(settings, row.key)))
                    return
                value = self._widget_value(gui_row_field(row, settings), settings)
                var.set(value)
                if fmt and row.key in self.labels:
                    self.labels[row.key].config(text=fmt % value)
            finally:
                self._in_update = False

        def _apply_field(
            self,
            row: GuiFieldRow,
            value,
            fmt: str | None,
            *,
            revert=None,
            allow_prompt: bool = False,
        ) -> bool:
            """Apply one edit through the Step-21 safe-live-apply policy.

            The policy decides whether a retargeting edit may happen now; the
            engine still owns value validation. A refused edit is reverted with
            an explanation instead of silently doing nothing, and a deliberate
            one-shot control (combo/checkbox) may confirm an override.
            """
            if row is None:
                return False
            settings = self.engine.get_settings()
            telemetry = self.engine.telemetry()
            decision = gui_live_apply_policy(row.key, settings, telemetry)
            if (
                not decision.allowed
                and allow_prompt
                and decision.kind == GUI_APPLY_NEUTRAL_ONLY
                and messagebox.askyesno(
                    "Apply while steering?",
                    f"{decision.reason}\n\nApply the change anyway?",
                )
            ):
                decision = gui_live_apply_policy(
                    row.key, settings, telemetry, acknowledge_abrupt=True
                )
            if not decision.allowed:
                if revert is not None:
                    revert()
                self._hold_status(decision.reason)
                return False
            value_to_apply = value
            if row.key == "focus_allowlist_text":
                # A list-valued setting edited as text: parse here so the engine
                # still receives validated structure, never raw prose.
                try:
                    value_to_apply = safety_contract.parse_allowlist_text(value)
                except ValueError as exc:
                    if revert is not None:
                        revert()
                    self._hold_status(f"Allowed windows: {exc}")
                    return False
            try:
                self.engine.set_field(decision.field, value_to_apply)
            except (AttributeError, ValueError) as exc:
                if revert is not None:
                    revert()
                self._hold_status(f"{decision.field}: {exc}")
                return False
            if fmt and row.key in self.labels:
                self.labels[row.key].config(text=fmt % value)
            note = decision.reason
            if decision.is_override:
                note = f"ACKNOWLEDGED OVERRIDE — {note}"
            self._hold_status(f"{gui_field_label(row, settings)} updated. {note}")
            return True

        def _widget_value(self, key: str, settings: Settings):
            """Expose retained GUI controls without hiding active v2 values.

            Step 21 will replace these compatibility widgets with native-v2
            unit-bearing controls. Until then, show the inverse of the
            Step-19 bridge so a native profile's visible value and its active
            explicit runtime field do not disagree.
            """
            if key == "focus_allowlist":
                return safety_contract.format_allowlist_text(
                    list(getattr(settings, "focus_allowlist", ()) or ())
                )
            if settings.profile_semantics == "v2":
                if key == "smoothing":
                    tau_s = settings.smoothing_tau_ms / 1000.0
                    return (
                        0.0
                        if tau_s <= 0.0
                        else math.exp(-1.0 / (LEGACY_FILTER_REFERENCE_HZ * tau_s))
                    )
                if key == "noise_gate_px":
                    return (
                        settings.noise_gate_units_per_s
                        / LEGACY_FILTER_REFERENCE_HZ
                    )
                if key == "hysteresis_px":
                    return (
                        settings.centering_hysteresis_units_per_s
                        / LEGACY_FILTER_REFERENCE_HZ
                    )
                if key == "center_time_s":
                    return settings.center_return_time_ms / 1000.0
            return getattr(settings, key)

        def slider_row(self, parent, row: GuiFieldRow) -> None:
            """Create one tuning row from the Step-21 unit contract.

            The label, tooltip, range, and edited field all come from
            :data:`GUI_ROWS`, so a native-v2 profile edits explicit physical
            fields (ms, source units/s) while a retained v1 profile keeps its
            documented 83 Hz-calibrated fields. A slider is a continuous
            control, so an unsafe live edit is refused with an explanation
            instead of interrupting the drag with a dialog.
            """
            settings = self.engine.get_settings()
            lo, hi, res, fmt = gui_row_domain(row, settings)
            frame = ttk.Frame(parent)
            frame.pack(fill="x", padx=8, pady=1)
            label = ttk.Label(frame, text=gui_field_label(row, settings), width=34)
            label.pack(side="left")
            var = tk.DoubleVar(
                value=self._widget_value(gui_row_field(row, settings), settings)
            )
            self.vars[row.key] = (var, fmt)

            def on_move(v, row=row, var=var):
                if self._in_update:
                    return
                lo2, hi2, res2, fmt2 = gui_row_domain(
                    row, self.engine.get_settings()
                )
                val = float(v)
                if res2:
                    val = round(val / res2) * res2
                val = max(lo2, min(hi2, val))
                self._in_update = True
                try:
                    var.set(val)
                finally:
                    self._in_update = False
                self._apply_field(
                    row, val, fmt2, revert=lambda: self._revert_widget(row)
                )

            scale = ttk.Scale(
                frame,
                from_=lo,
                to=hi,
                variable=var,
                length=180,
                command=on_move,
            )
            scale.pack(side="left", padx=4)
            lbl = ttk.Label(frame, text=fmt % var.get(), width=8, anchor="e")
            lbl.pack(side="left")
            self.labels[row.key] = lbl
            self.row_widgets[row.key] = (frame, label, scale)
            hint = gui_field_hint(row, settings)
            if hint:
                ttk.Label(
                    frame,
                    text=hint,
                    foreground="#666",
                    font=("TkDefaultFont", 8),
                ).pack(side="left", padx=6)

        def bool_row(self, parent, row: GuiFieldRow) -> None:
            """Create a boolean row from the same contract as :meth:`slider_row`."""
            settings = self.engine.get_settings()
            frame = ttk.Frame(parent)
            frame.pack(fill="x", padx=8, pady=1)
            var = tk.BooleanVar(value=bool(getattr(settings, row.key)))
            self.vars[row.key] = (var, None)

            def on_toggle(row=row, var=var):
                self._apply_field(
                    row,
                    bool(var.get()),
                    None,
                    revert=lambda: self._revert_widget(row),
                    allow_prompt=True,
                )

            check = ttk.Checkbutton(
                frame,
                text=gui_field_label(row, settings),
                variable=var,
                command=on_toggle,
            )
            check.pack(side="left")
            self.row_widgets[row.key] = (frame, check, None)
            hint = gui_field_hint(row, settings)
            if hint:
                ttk.Label(
                    frame,
                    text=hint,
                    foreground="#666",
                    font=("TkDefaultFont", 8),
                ).pack(side="left", padx=6)

        def text_row(self, parent, row: GuiFieldRow) -> None:
            """Create a free-text row (hotkeys, allowlist rules).

            Text commits on Return or focus-out rather than per keystroke: half
            a hotkey is not a hotkey, and applying it character by character
            would fight the validation that protects the profile.
            """
            settings = self.engine.get_settings()
            frame = ttk.Frame(parent)
            frame.pack(fill="x", padx=8, pady=1)
            ttk.Label(
                frame, text=gui_field_label(row, settings), width=34
            ).pack(side="left")
            var = tk.StringVar(
                value=self._widget_value(gui_row_field(row, settings), settings)
            )
            self.vars[row.key] = (var, None)

            def commit(_event=None, row=row, var=var):
                if self._in_update:
                    return
                self._apply_field(
                    row, var.get(), None, revert=lambda: self._revert_widget(row)
                )

            entry = ttk.Entry(frame, textvariable=var, width=34)
            entry.pack(side="left", padx=4)
            entry.bind("<Return>", commit)
            entry.bind("<FocusOut>", commit)
            self.row_widgets[row.key] = (frame, entry, None)
            hint = gui_field_hint(row, settings)
            if hint:
                ttk.Label(
                    frame,
                    text=hint,
                    foreground="#666",
                    font=("TkDefaultFont", 8),
                ).pack(side="left", padx=6)

        def _build_header(self) -> None:
            bar = ttk.Frame(self, padding=(10, 8))
            bar.pack(fill="x")
            self.start_btn = ttk.Button(bar, text="Start", command=self._toggle_run)
            self.start_btn.pack(side="left")
            self.pause_btn = ttk.Button(
                bar,
                text="Pause",
                command=self._toggle_pause,
                state="disabled",
            )
            self.pause_btn.pack(side="left", padx=6)

            ttk.Label(bar, text="Steering mode:").pack(side="left", padx=(10, 2))
            self.mode_var = tk.StringVar(
                value=self.engine.get_settings().steering_mode
            )
            self.mode_combo = ttk.Combobox(
                bar,
                textvariable=self.mode_var,
                values=STEERING_MODES,
                state="readonly",
                width=20,
            )
            self.mode_combo.pack(side="left", padx=(0, 6))
            self.mode_combo.bind("<<ComboboxSelected>>", self._on_steering_mode)

            self.state_lbl = ttk.Label(bar, text="Stopped", foreground="#a33")
            self.state_lbl.pack(side="left", padx=12)
            ttk.Label(
                bar,
                text="F8 pause/resume   F9 emergency stop",
                foreground="#666",
            ).pack(side="right")

            # Step 22: a highly visible indicator. A driver must be able to see
            # at a glance whether the pad is receiving movement, is being held
            # by a guard, or is idle — without reading a number.
            indicator_bar = ttk.Frame(self, padding=(10, 0, 10, 6))
            indicator_bar.pack(fill="x")
            self.indicator_lbl = ttk.Label(
                indicator_bar,
                text="ENGINE STOPPED",
                font=("TkDefaultFont", 13, "bold"),
                anchor="w",
                padding=(8, 4),
                relief="ridge",
            )
            self.indicator_lbl.pack(fill="x")
            self.indicator_detail_lbl = ttk.Label(
                indicator_bar,
                text="No virtual pad is being driven.",
                foreground="#555",
                font=("TkDefaultFont", 9),
                wraplength=760,
                justify="left",
            )
            self.indicator_detail_lbl.pack(anchor="w", pady=(2, 0))

            # Step 21: a persistent context strip. These four facts were
            # previously inferable only from scattered widget states, yet they
            # decide how every other number should be read: which input source
            # produced the counts, which centring law is active, whether the
            # values are native-v2 physical units or retained v1 calibration,
            # and whether any safety guard actually exists.
            context = ttk.Frame(self, padding=(10, 0, 10, 6))
            context.pack(fill="x")
            self.context_lbls: dict[str, Any] = {}
            for index, (title, key) in enumerate(
                (
                    ("Input source", "input_source"),
                    ("Steering mode / centring", "steering"),
                    ("Profile semantics", "profile"),
                    ("Focus/deadman safety guard", "safety"),
                )
            ):
                cell = ttk.Frame(context)
                cell.grid(row=0, column=index, sticky="nw", padx=(0, 18))
                ttk.Label(
                    cell,
                    text=title,
                    foreground="#666",
                    font=("TkDefaultFont", 8),
                ).pack(anchor="w")
                lbl = ttk.Label(
                    cell,
                    text="\u2014",
                    foreground="#333",
                    font=("TkDefaultFont", 9),
                    wraplength=250,
                    justify="left",
                )
                lbl.pack(anchor="w")
                self.context_lbls[key] = lbl

        def _build_meter(self) -> None:
            frame = ttk.LabelFrame(self, text="Steering", padding=6)
            frame.pack(fill="x", padx=10, pady=4)
            self.canvas = tk.Canvas(
                frame,
                width=760,
                height=86,
                highlightthickness=0,
                bg="#1e1e1e",
            )
            self.canvas.pack()
            self.meter_lbl = ttk.Label(frame, text="steer +0.000   out +0.0%")
            self.meter_lbl.pack(anchor="e")

        def _build_notebook(self) -> None:
            nb = ttk.Notebook(self)
            nb.pack(fill="both", expand=True, padx=10, pady=4)

            main = ttk.Frame(nb, padding=6)
            nb.add(main, text="  Main  ")

            for row in GUI_ROWS:
                if not row.main:
                    continue
                if row.key == "curve_exp":
                    # Keep the response-curve preset directly above the exponent
                    # it controls.
                    self._build_combo_row(
                        main, "curve_preset", GUI_COMBO_VALUES["curve_preset"]
                    )
                if row.key in GUI_COMBO_ROW_KEYS:
                    continue
                if row.boolean:
                    self.bool_row(main, row)
                elif row.text:
                    self.text_row(main, row)
                else:
                    self.slider_row(main, row)

            self.travel_lbl = ttk.Label(main, text="", foreground="#444")
            self.travel_lbl.pack(anchor="w", padx=8, pady=4)
            self.mode_hint_lbl = ttk.Label(
                main,
                text="",
                foreground="#666",
                font=("TkDefaultFont", 8),
            )
            self.mode_hint_lbl.pack(anchor="w", padx=8, pady=(0, 4))
            self._update_mode_hint(self.engine.get_settings())

            adv = ttk.Frame(nb, padding=4)
            nb.add(adv, text="  Advanced settings  ")
            for group_name in ADV_GROUP_NAMES:
                group = ttk.LabelFrame(adv, text=group_name, padding=4)
                group.pack(fill="x", padx=6, pady=3)
                for row in GUI_ROWS:
                    if row.main or row.group != group_name:
                        continue
                    if row.key in GUI_COMBO_ROW_KEYS:
                        self._build_combo_row(
                            group, row.key, GUI_COMBO_VALUES[row.key]
                        )
                    elif row.boolean:
                        self.bool_row(group, row)
                    elif row.text:
                        self.text_row(group, row)
                    else:
                        self.slider_row(group, row)

            self._build_diagnostics_tab(nb)
            self._build_calibration_tab(nb)

        def _build_combo_row(self, parent, key, values) -> None:
            """Build one read-only combo row described by the same contract."""
            row = gui_row_for(key)
            settings = self.engine.get_settings()
            frame = ttk.Frame(parent)
            frame.pack(fill="x", padx=8, pady=2)
            ttk.Label(
                frame, text=gui_field_label(row, settings), width=34
            ).pack(side="left")
            var = tk.StringVar(value=str(getattr(settings, key)))
            self.vars[key] = (var, None)
            self.combo_vars[key] = var
            combo = ttk.Combobox(
                frame,
                textvariable=var,
                values=values,
                state="readonly",
                width=16,
            )
            combo.pack(side="left", padx=4)
            combo.bind(
                "<<ComboboxSelected>>",
                lambda _event, key=key: self._on_combo(key),
            )
            self.combos[key] = combo
            if key == "curve_preset":
                # Retained attribute names: external tooling and the pre-Step-21
                # GUI both referred to this widget as the curve preset control.
                self.curve_var = var
                self.curve_combo = combo
            hint = gui_field_hint(row, settings)
            if hint:
                ttk.Label(
                    frame,
                    text=hint,
                    foreground="#666",
                    font=("TkDefaultFont", 8),
                ).pack(side="left", padx=6)


        def _build_calibration_tab(self, nb) -> None:
            """Add the Step-23 Calibration tab.

            The tab is a *view* of the pure flow in ``src/calibration.py``: the
            prompts, refusals, and proposals all come from that state machine, so
            the GUI cannot drift from the CLI or from the tests. Starting is
            refused while a session is driving the pad, because this flow
            measures the mouse and never creates a pad of its own.
            """
            tab = ttk.Frame(nb, padding=6)
            nb.add(tab, text="  Calibration  ")

            prompt_frame = ttk.LabelFrame(
                tab, text="Measure the mouse", padding=6
            )
            prompt_frame.pack(fill="x", padx=6, pady=3)
            self.cal_step_lbl = ttk.Label(
                prompt_frame,
                text="Not started.",
                font=("TkDefaultFont", 10, "bold"),
                anchor="w",
            )
            self.cal_step_lbl.pack(fill="x")
            self.cal_prompt_lbl = ttk.Label(
                prompt_frame,
                text=CALIBRATION_PROMISE_INPUT_ONLY,
                foreground="#333",
                wraplength=560,
                justify="left",
            )
            self.cal_prompt_lbl.pack(fill="x", pady=(2, 2))
            self.cal_detail_lbl = ttk.Label(
                prompt_frame,
                text="",
                foreground="#8a5200",
                wraplength=560,
                justify="left",
            )
            self.cal_detail_lbl.pack(fill="x")

            controls = ttk.Frame(prompt_frame)
            controls.pack(fill="x", pady=(4, 0))
            self.cal_start_btn = ttk.Button(
                controls, text="Start", command=self._calibration_start
            )
            self.cal_start_btn.pack(side="left", padx=2)
            self.cal_retry_btn = ttk.Button(
                controls,
                text="Retry step",
                command=self._calibration_retry,
                state="disabled",
            )
            self.cal_retry_btn.pack(side="left", padx=2)
            self.cal_skip_btn = ttk.Button(
                controls,
                text="Skip step",
                command=self._calibration_skip,
                state="disabled",
            )
            self.cal_skip_btn.pack(side="left", padx=2)
            self.cal_live_var = tk.BooleanVar(value=False)
            self.cal_live_check = ttk.Checkbutton(
                controls,
                text=(
                    "measure the live output (a steering session keeps running and "
                    "the pad steers while you measure)"
                ),
                variable=self.cal_live_var,
                command=self._calibration_live_toggle,
            )
            self.cal_live_check.pack(side="left", padx=8)
            self.cal_abort_btn = ttk.Button(
                controls,
                text="Abort",
                command=self._calibration_abort,
                state="disabled",
            )
            self.cal_abort_btn.pack(side="left", padx=2)

            # The reports the flow cannot measure for itself: only the operator
            # can see inside the game.
            self.cal_report_frame = ttk.LabelFrame(
                tab, text="Report what you saw in the game", padding=6
            )
            self.cal_report_frame.pack(fill="x", padx=6, pady=3)
            self.cal_report_buttons: dict[str, Any] = {}
            reports = (
                (
                    calibration_contract.STEP_LOCK,
                    (
                        ("Full lock", calibration_contract.OBSERVED_FULL),
                        ("Still short", calibration_contract.OBSERVED_PARTIAL),
                        ("Already past it", calibration_contract.OBSERVED_SATURATED),
                    ),
                ),
                (
                    calibration_contract.STEP_VERIFY,
                    (
                        ("About right", calibration_contract.OBSERVED_ABOUT_RIGHT),
                        ("Too sensitive", calibration_contract.OBSERVED_TOO_SENSITIVE),
                        ("Not enough", calibration_contract.OBSERVED_NOT_ENOUGH),
                    ),
                ),
            )
            for step_name, options in reports:
                for label, value in options:
                    button = ttk.Button(
                        self.cal_report_frame,
                        text=label,
                        command=lambda value=value: self._calibration_report(value),
                        # Only the flow may enable a report: a button that looks
                        # pressable while nothing is being measured would invite
                        # a click that cannot mean anything.
                        state="disabled",
                    )
                    button.pack(side="left", padx=2)
                    self.cal_report_buttons[f"{step_name}:{value}"] = button

            readout = ttk.LabelFrame(tab, text="Live measurement", padding=6)
            readout.pack(fill="x", padx=6, pady=3)
            self.cal_readouts: dict[str, Any] = {}
            for key, title in (
                ("samples", "Samples"),
                ("hz", "Observed rate"),
                ("travel", "Travel (right / left)"),
                ("symmetry", "Symmetry"),
                ("noise", "Noise floor"),
                ("output", "Sent axis peak"),
            ):
                line = ttk.Frame(readout)
                line.pack(fill="x", pady=1)
                ttk.Label(
                    line, text=title + ":", width=22, anchor="w", foreground="#444"
                ).pack(side="left")
                lbl = ttk.Label(line, text="\u2014", anchor="w")
                lbl.pack(side="left", fill="x", expand=True)
                self.cal_readouts[key] = lbl

            proposal = ttk.LabelFrame(
                tab, text="Measured proposal (nothing is written without you)", padding=6
            )
            proposal.pack(fill="both", expand=True, padx=6, pady=3)
            self.cal_proposal_body = ttk.Frame(proposal)
            self.cal_proposal_body.pack(fill="x")
            self.cal_proposal_lbls: list[Any] = []
            buttons = ttk.Frame(proposal)
            buttons.pack(fill="x", pady=(4, 0))
            self.cal_apply_btn = ttk.Button(
                buttons,
                text="Apply to this profile",
                command=self._calibration_apply,
                state="disabled",
            )
            self.cal_apply_btn.pack(side="left", padx=2)
            self.cal_export_btn = ttk.Button(
                buttons, text="Export bundle…", command=self._calibration_export
            )
            self.cal_export_btn.pack(side="left", padx=2)
            self.cal_import_btn = ttk.Button(
                buttons, text="Import bundle…", command=self._calibration_import
            )
            self.cal_import_btn.pack(side="left", padx=2)
            self.cal_note_lbl = ttk.Label(
                proposal,
                text=(
                    "A proposal is only offered for fields that were actually "
                    "measured, and each run may move a value at most "
                    f"{calibration_contract.CONSERVATIVE_STEP_LIMIT:.0%}."
                ),
                foreground="#666",
                wraplength=560,
                justify="left",
            )
            self.cal_note_lbl.pack(anchor="w", pady=(4, 0))

        # ----- Step 23: calibration actions ---------------------------------
        def _calibration_live_toggle(self) -> None:
            """Keep the tab's promise exactly as strong as what it will do.

            Ticking the box changes what is measured *and* what the operator is
            told, in the same action, so the two cannot drift apart.
            """
            live = bool(self.cal_live_var.get()) if self.cal_live_var is not None else False
            self.cal_prompt_lbl.config(
                text=(
                    CALIBRATION_PROMISE_LIVE_OUTPUT
                    if live
                    else CALIBRATION_PROMISE_INPUT_ONLY
                )
            )

        def _calibration_start(self) -> None:
            if self.cal_runner is not None:
                return
            # The calibration source is not the engine's resource, so it is
            # released here rather than relying on session cleanup.
            state = str(self.engine.telemetry().get("lifecycle_state", RUNTIME_STOPPED))
            live = bool(self.cal_live_var.get()) if self.cal_live_var is not None else False
            if live:
                # Opt-in: the running-state refusal is dropped, but only a
                # settled live session is acceptable. A stopped engine publishes
                # a stale frame, and a stale frame is not a measurement.
                if state not in (RUNTIME_RUNNING, RUNTIME_PAUSED):
                    self._hold_status(
                        "Live output measurement needs a running session: start "
                        "the session first, or untick it."
                    )
                    return
                guard = calibration_contract.calibration_precondition(
                    state, engine_running_states=()
                )
            else:
                guard = calibration_contract.calibration_precondition(state)
            if not guard.allowed:
                self._hold_status("Calibration refused: " + guard.reason)
                return
            factory = self.cal_runner_factory or CalibrationRunner
            options: dict[str, Any] = {
                "input_backend": self.engine.input_backend(),
            }
            if live:
                options["output_probe"] = engine_output_probe(self.engine)
                options["allow_running_session"] = True
            runner = factory(self.engine.get_settings(), **options)
            try:
                outcome = runner.start()
            except RuntimeError as exc:
                self._hold_status(f"Calibration could not start: {exc}")
                runner.stop()
                return
            self.cal_runner = runner
            self.cal_suggestion = None
            self._render_calibration(outcome)
            self._hold_status("Calibration started; move the mouse as instructed.")

        def _calibration_poll(self) -> None:
            runner = self.cal_runner
            if runner is None:
                return
            try:
                outcome = runner.poll()
            except (RuntimeError, ValueError) as exc:
                # A failing source must not take the Tk callback down with it:
                # release the measurement and say what happened.
                outcome = runner.abort(str(exc))
                runner.stop()
                self.cal_runner = None
                self._render_calibration(outcome)
                self._hold_status(f"Calibration stopped: {exc}")
                return
            self._render_calibration(outcome)
            if outcome.complete or outcome.aborted:
                self.cal_suggestion = runner.suggestion()
                self.cal_output_peak = runner.report().output_axis_peak
                runner.stop()
                self.cal_runner = None
                self._render_calibration_suggestion(self.cal_suggestion)

        def _calibration_retry(self) -> None:
            if self.cal_runner is not None:
                self._render_calibration(self.cal_runner.retry())

        def _calibration_skip(self) -> None:
            if self.cal_runner is not None:
                self._render_calibration(self.cal_runner.skip())

        def _calibration_abort(self) -> None:
            runner = self.cal_runner
            if runner is None:
                return
            outcome = runner.abort("aborted by the operator")
            runner.stop()
            self.cal_runner = None
            self._render_calibration(outcome)
            self._hold_status("Calibration aborted; nothing was changed.")

        def _calibration_report(self, value: str) -> None:
            runner = self.cal_runner
            if runner is None:
                return
            try:
                outcome = runner.observe(value)
            except ValueError as exc:
                self._hold_status(f"Calibration report refused: {exc}")
                return
            self._render_calibration(outcome)
            if outcome.complete or outcome.aborted:
                self.cal_suggestion = runner.suggestion()
                self.cal_output_peak = runner.report().output_axis_peak
                runner.stop()
                self.cal_runner = None
                self._render_calibration_suggestion(self.cal_suggestion)

        def _render_calibration(self, outcome) -> None:
            """Render one flow state. All wording comes from the flow itself."""
            if outcome.complete:
                self.cal_step_lbl.config(text="Calibration complete.")
            elif outcome.aborted:
                self.cal_step_lbl.config(text=outcome.status_line())
            else:
                self.cal_step_lbl.config(
                    text=(
                        f"Step {outcome.step_index + 1} of {outcome.step_count}: "
                        f"{outcome.title}"
                    )
                )
            self.cal_prompt_lbl.config(text=outcome.prompt)
            self.cal_detail_lbl.config(
                text=(
                    outcome.failure
                    or (outcome.guidance if not outcome.complete else "")
                )
            )
            measured = dict(outcome.measured_so_far)
            self.cal_readouts["samples"].config(
                text=f"{int(measured.get('samples', 0))} "
                f"({outcome.progress:.0%} of this step)"
            )
            self.cal_readouts["hz"].config(
                text=f"{measured.get('observed_hz', 0.0):.1f} Hz observed vs "
                f"{self.engine.get_settings().update_hz:.0f} Hz configured"
            )
            self.cal_readouts["travel"].config(
                text=f"{measured.get('positive_travel_units', 0.0):.0f} / "
                f"{measured.get('negative_travel_units', 0.0):.0f} units"
            )
            self.cal_readouts["noise"].config(
                text=f"{measured.get('rate_p95_units_per_s', 0.0):.1f} units/s "
                "(p95 of the quiet step)"
            )
            runner = self.cal_runner
            symmetry = "not measured yet"
            if runner is not None:
                report = runner.report()
                left_rate = report.measured(
                    calibration_contract.STEP_SWEEP_LEFT, "rate_units_per_s"
                )
                right_rate = report.measured(
                    calibration_contract.STEP_SWEEP_RIGHT, "rate_units_per_s"
                )
                if left_rate > 0.0 and right_rate > 0.0:
                    weaker = "left" if left_rate < right_rate else "right"
                    symmetry = (
                        f"{min(left_rate, right_rate) / max(left_rate, right_rate):.0%}"
                        f" \u2014 the {weaker} sweep was weaker"
                    )
            self.cal_readouts["symmetry"].config(text=symmetry)
            for key, button in self.cal_report_buttons.items():
                step_name = key.split(":", 1)[0]
                button.state(
                    ["!disabled"]
                    if outcome.needs_observation and outcome.step == step_name
                    else ["disabled"]
                )
            for button, enabled in (
                (self.cal_retry_btn, self.cal_runner is not None),
                (self.cal_skip_btn, self.cal_runner is not None),
                (self.cal_abort_btn, self.cal_runner is not None),
                (self.cal_start_btn, self.cal_runner is None),
            ):
                button.state(["!disabled"] if enabled else ["disabled"])

        def _render_calibration_suggestion(self, suggestion) -> None:
            for lbl in self.cal_proposal_lbls:
                lbl.destroy()
            self.cal_proposal_lbls = []
            self.cal_suggestion = suggestion
            if suggestion is None:
                return
            if not suggestion.allowed:
                text = "No profile is offered: " + str(suggestion.blocked_reason)
                lbl = ttk.Label(
                    self.cal_proposal_body,
                    text=text,
                    foreground="#a31b1b",
                    wraplength=560,
                    justify="left",
                )
                lbl.pack(anchor="w")
                self.cal_proposal_lbls.append(lbl)
                self.cal_apply_btn.state(["disabled"])
                return
            if not suggestion.changed_fields:
                lbl = ttk.Label(
                    self.cal_proposal_body,
                    text=(
                        "The measurement agrees with the current profile; nothing "
                        "needs to change."
                    ),
                    foreground="#0a5a2a",
                )
                lbl.pack(anchor="w")
                self.cal_proposal_lbls.append(lbl)
            for item in suggestion.changed_fields:
                lbl = ttk.Label(
                    self.cal_proposal_body,
                    text=f"{item.describe()}  [{item.layer} layer]",
                    wraplength=560,
                    justify="left",
                )
                lbl.pack(anchor="w")
                self.cal_proposal_lbls.append(lbl)
            for warning in suggestion.warnings:
                lbl = ttk.Label(
                    self.cal_proposal_body,
                    text="note: " + warning,
                    foreground="#8a5200",
                    wraplength=560,
                    justify="left",
                )
                lbl.pack(anchor="w")
                self.cal_proposal_lbls.append(lbl)
            self.cal_readouts["output"].config(
                text=(
                    f"{self.cal_output_peak:.2f} (sampled from published telemetry)"
                    if self.cal_output_peak
                    else "not sampled (no session was running)"
                )
            )
            self.cal_apply_btn.state(
                ["!disabled"] if suggestion.changed_fields else ["disabled"]
            )

        def _calibration_apply(self) -> None:
            suggestion = self.cal_suggestion
            if suggestion is None or not suggestion.allowed:
                return
            settings = self.engine.get_settings()
            refused: list[str] = []
            for item in suggestion.changed_fields:
                decision = gui_live_apply_policy(
                    item.field, settings, self.engine.telemetry()
                )
                if not decision.allowed:
                    refused.append(f"{item.field}: {decision.reason}")
                    continue
                try:
                    self.engine.set_field(item.field, item.proposed)
                except (AttributeError, ValueError) as exc:
                    refused.append(f"{item.field}: {exc}")
            try:
                self.engine.set_field("calibration", dict(suggestion.provenance))
            except (AttributeError, ValueError):  # pragma: no cover - defensive
                pass
            self._refresh_widgets(self.engine.get_settings())
            if refused:
                self._hold_status(
                    "Some measured values were refused: " + "; ".join(refused)
                )
                return
            self._hold_status(
                "Applied "
                + ", ".join(item.field for item in suggestion.changed_fields)
                + " from the measurement. Save the profile to keep them."
            )

        def _calibration_export(self) -> None:
            """Write the current profile as a layered bundle."""
            settings = self.engine.get_settings()
            path = filedialog.asksaveasfilename(
                title="Export layered profile bundle",
                defaultextension=".bundle.json",
                initialfile="mouse_steering.bundle.json",
            )
            if not path:
                return
            bundle = calibration_contract.bundle_from_settings(
                settings,
                name=Path(path).stem or "current",
                calibration=dict(settings.calibration),
            )
            try:
                calibration_contract.write_bundle(bundle, Path(path))
            except (OSError, ValueError) as exc:
                self._hold_status(f"Export failed: {exc}")
                return
            self._hold_status(
                "Exported "
                + ", ".join(layer.kind for layer in bundle.layers)
                + f" layers to {path}."
            )

        def _calibration_import(self) -> None:
            """Merge a layered bundle, refusing anything the policy blocks."""
            path = filedialog.askopenfilename(
                title="Import layered profile bundle",
                filetypes=(("Profile bundle", "*.json"), ("All files", "*.*")),
            )
            if not path:
                return
            result = calibration_contract.load_bundle(Path(path))
            if not result.usable or result.bundle is None:
                self._hold_status("Import failed: " + result.summary())
                return
            settings = self.engine.get_settings()
            try:
                merged = calibration_contract.merge_bundle(result.bundle, settings)
            except ValueError as exc:
                self._hold_status(f"Import refused: {exc}")
                return
            refused: list[str] = []
            for application in merged.applications:
                for field_name, value in sorted(application.applied.items()):
                    decision = gui_live_apply_policy(
                        field_name, self.engine.get_settings(), self.engine.telemetry()
                    )
                    if not decision.allowed:
                        refused.append(f"{field_name}: {decision.reason}")
                        continue
                    try:
                        self.engine.set_field(field_name, value)
                    except (AttributeError, ValueError) as exc:
                        refused.append(f"{field_name}: {exc}")
            if merged.provenance:
                try:
                    self.engine.set_field("calibration", dict(merged.provenance))
                except (AttributeError, ValueError):  # pragma: no cover
                    pass
            self._refresh_widgets(self.engine.get_settings())
            for notice in result.notices:
                self._hold_status("Bundle notice: " + notice)
                return
            if refused:
                self._hold_status("Import partially refused: " + "; ".join(refused))
                return
            self._hold_status(
                f"Merged {len(result.bundle.layers)} layer(s) from "
                f"{Path(path).name}; save the profile to keep them."
            )

        def _build_diagnostics_tab(self, nb) -> None:
            """Add the Step-21 Diagnostics tab.

            The tab answers the three questions a driver actually has: why the
            car is centring, why input is being ignored, and whether the virtual
            pad is truly connected. Values are rendered from the Tk-free
            :class:`GuiRuntimeView`, so no interpretation happens in widget code.
            """
            diag = ttk.Frame(nb, padding=6)
            nb.add(diag, text="  Diagnostics  ")

            explain = ttk.LabelFrame(diag, text="What is happening now", padding=6)
            explain.pack(fill="x", padx=6, pady=3)
            self.explain_lbls: dict[str, Any] = {}
            for key, title in (
                ("centering_reason", "Centring"),
                ("input_gate_reason", "Input acceptance"),
                ("pad_connection", "Virtual pad"),
                ("late_guard_reason", "Scheduler safety"),
                ("fault", "Fault cause"),
            ):
                line = ttk.Frame(explain)
                line.pack(fill="x", pady=2)
                ttk.Label(
                    line,
                    text=title + ":",
                    width=18,
                    anchor="nw",
                    foreground="#444",
                ).pack(side="left")
                lbl = ttk.Label(
                    line,
                    text="\u2014",
                    foreground="#111",
                    wraplength=470,
                    justify="left",
                )
                lbl.pack(side="left", fill="x", expand=True)
                self.explain_lbls[key] = lbl

            values = ttk.LabelFrame(diag, text="Signals and timing", padding=6)
            values.pack(fill="both", expand=True, padx=6, pady=3)
            self.diag_values = values
            self.diag_lbls: dict[str, Any] = {}
            self.profile_lbl = ttk.Label(
                diag,
                text="",
                foreground="#444",
                font=("TkDefaultFont", 8),
                wraplength=700,
                justify="left",
            )
            self.profile_lbl.pack(anchor="w", padx=6, pady=(0, 4))

        def _render_diagnostics(self, view: GuiRuntimeView) -> None:
            """Update the Diagnostics tab and context strip from one view."""
            for name, value in view.diagnostics:
                lbl = self.diag_lbls.get(name)
                if lbl is None:
                    cell = ttk.Frame(self.diag_values)
                    cell.pack(fill="x", pady=1)
                    ttk.Label(
                        cell,
                        text=name,
                        width=24,
                        anchor="w",
                        foreground="#444",
                    ).pack(side="left")
                    lbl = ttk.Label(
                        cell,
                        text="",
                        anchor="w",
                        wraplength=470,
                        justify="left",
                    )
                    lbl.pack(side="left", fill="x", expand=True)
                    self.diag_lbls[name] = lbl
                lbl.config(text=value)

            for key, text in (
                ("centering_reason", view.centering_reason),
                ("input_gate_reason", view.input_gate_reason),
                ("pad_connection", view.pad_connection),
                ("late_guard_reason", view.late_guard_reason),
                ("fault", view.fault or "No runtime fault recorded."),
            ):
                self.explain_lbls[key].config(
                    text=text,
                    foreground=(
                        "#a33" if (key == "fault" and view.fault) else "#111"
                    ),
                )

            indicator = view.input_indicator
            foreground, background = GUI_INDICATOR_COLOURS.get(
                indicator.severity, ("#555", "#f0f0f0")
            )
            self.indicator_lbl.config(
                text=f"\u25cf  {indicator.label}",
                foreground=foreground,
                background=background,
            )
            detail = indicator.detail
            if view.pause_notice:
                detail = f"{detail} {view.pause_notice}"
            self.indicator_detail_lbl.config(text=detail)

            for key, text in (
                ("input_source", view.input_source),
                ("steering", view.steering),
                ("profile", view.profile),
            ):
                self.context_lbls[key].config(text=text)
            guard_note = view.safety
            if self._close_pending:
                guard_note += " | window held open: cleanup not confirmed"
            self.context_lbls["safety"].config(
                text=guard_note,
                foreground="#a33" if not view.safety_guard_configured else "#333",
            )
            self.profile_lbl.config(
                text=(
                    f"{view.session_label} | lifecycle {view.lifecycle} | "
                    f"{view.late_guard_reason}"
                )
            )

        def _build_buttons(self) -> None:
            bar = ttk.Frame(self, padding=(10, 4))
            bar.pack(fill="x")
            ttk.Button(
                bar,
                text="Save preset…",
                command=self._save_preset,
            ).pack(side="left")
            self.load_preset_btn = ttk.Button(
                bar,
                text="Load preset…",
                command=self._load_preset,
            )
            self.load_preset_btn.pack(side="left", padx=6)
            self.defaults_btn = ttk.Button(
                bar,
                text="Defaults",
                command=self._reset_defaults,
            )
            self.defaults_btn.pack(side="left")

        def _build_statusbar(self) -> None:
            self.status = ttk.Label(
                self,
                text=(
                    "Ready. Input: Raw Input (default); "
                    f"DPI awareness: {dpi_status.summary}"
                    if input_backend == INPUT_BACKEND_RAW_INPUT
                    else "DEGRADED: cursor fallback — Raw Input is the accepted default; "
                    f"DPI awareness: {dpi_status.summary}"
                ),
                relief="sunken",
                anchor="w",
                padding=(6, 2),
            )
            self.status.pack(fill="x", side="bottom")

        def _show_stop_result(self, result: StopResult) -> None:
            self.status.config(text=format_stop_result(result))

        def _show_runtime_exception(self, exc: BaseException) -> None:
            """Keep a Tk callback on its safe retry/close surface on failure."""
            diagnostic = getattr(exc, "diagnostic", None)
            if isinstance(diagnostic, RuntimeDiagnostic):
                message = format_runtime_diagnostic(diagnostic)
            else:
                message = (
                    f"Operation failed ({type(exc).__name__}: {exc}). Retry after "
                    "correcting the problem; inspect the diagnostic log if one was "
                    "created, and close only after cleanup is confirmed."
                )
            self.status.config(text=message)

        def _toggle_run(self) -> None:
            try:
                telemetry = self.engine.telemetry()
                lifecycle = str(telemetry.get("lifecycle_state", RUNTIME_STOPPED))
                cleanup_complete = bool(
                    telemetry.get("session_cleanup_complete", True)
                )
                should_stop_or_retry = (
                    lifecycle in _RUNTIME_ACTIVE_STATES
                    or lifecycle == RUNTIME_STOPPING
                    or (lifecycle == RUNTIME_FAULTED and not cleanup_complete)
                )
                if should_stop_or_retry:
                    self._show_stop_result(self.engine.stop())
                else:
                    self.engine.start()
                    self.status.config(
                        text=(
                            "Started."
                            if input_backend == INPUT_BACKEND_RAW_INPUT
                            else "DEGRADED cursor fallback started — Raw Input is the accepted default."
                        )
                    )
            except Exception as exc:
                # EngineStartupError covers all expected backend/listener
                # failures; this broad boundary also prevents an unexpected
                # platform error from escaping a Tk callback.
                self._show_runtime_exception(exc)
            finally:
                self._sync_buttons()

        def _toggle_pause(self) -> None:
            try:
                if not self.engine.running:
                    return
                t = self.engine.telemetry()
                self.engine.set_paused(not bool(t["paused"]))
            except Exception as exc:
                self._show_runtime_exception(exc)
            finally:
                self._sync_buttons()

        def _on_steering_mode(self, _event=None) -> None:
            self._on_combo("steering_mode")

        def _on_combo(self, key: str) -> None:
            """Handle one deliberate combo change through the apply policy."""
            row = gui_row_for(key)
            if key == "steering_mode":
                value = self.mode_var.get()

                def revert():
                    self.mode_var.set(self.engine.get_settings().steering_mode)

            else:
                value = str(self.vars[key][0].get()) if key in self.vars else ""
                revert = lambda: self._revert_widget(row)
            applied = self._apply_field(
                row, value, None, revert=revert, allow_prompt=True
            )
            if key == "steering_mode":
                self._update_mode_hint(self.engine.get_settings())
                if applied:
                    self._hold_status(
                        f"Steering mode set to {value}. "
                        + {
                            "release_to_center": (
                                "Steering returns only after the hold time."
                            ),
                            "continuous_spring": (
                                "Input now balances a continuous return every "
                                "control tick."
                            ),
                            "manual": "Nothing returns the steering automatically.",
                        }.get(value, "")
                    )
            elif key == "safety_fail_mode" and applied:
                self._hold_status(
                    "Safety fail mode set to "
                    + value
                    + (
                        ": steering will be held when the foreground window "
                        "cannot be read."
                        if value == safety_contract.SAFETY_FAIL_CLOSED
                        else ": steering continues with a loud warning when the "
                        "foreground window cannot be read."
                    )
                )
            elif key == "curve_preset" and applied:
                # Squared/Cubed imply their exponent. This is a second
                # retargeting edit, so it is policy-routed and reverted too
                # rather than leaving the widget showing an unapplied value.
                implied = {"Squared": 2.0, "Cubed": 3.0}
                if value in implied:
                    exp_row = gui_row_for("curve_exp")
                    self._apply_field(
                        exp_row,
                        implied[value],
                        "%.1f",
                        revert=lambda: self._revert_widget(exp_row),
                    )

        def _update_mode_hint(self, s: Settings) -> None:
            hints = {
                "release_to_center": (
                    "Release-to-centre: waits through hold time, then returns raw steering."
                ),
                "continuous_spring": (
                    "Continuous spring: input and raw-position return act together every tick."
                ),
                "manual": (
                    "Manual: centre controls are inactive; countersteer to unwind."
                ),
            }
            self.mode_hint_lbl.config(text=hints[s.steering_mode])

        def _sync_buttons(self) -> None:
            t = self.engine.telemetry()
            lifecycle = str(t.get("lifecycle_state", RUNTIME_STOPPED))
            paused = lifecycle == RUNTIME_PAUSED
            stopping = lifecycle == RUNTIME_STOPPING
            active = lifecycle in _RUNTIME_ACTIVE_STATES
            cleanup_complete = bool(t.get("session_cleanup_complete", True))
            fault_cleanup_pending = (
                lifecycle == RUNTIME_FAULTED and not cleanup_complete
            )

            settings = self.engine.get_settings()
            self.mode_var.set(settings.steering_mode)
            self._update_mode_hint(settings)

            if stopping:
                run_label = "Retry stop"
            elif fault_cleanup_pending:
                run_label = "Retry cleanup"
            else:
                run_label = "Stop" if active else "Start"
            self.start_btn.config(text=run_label, state="normal")
            self.pause_btn.config(
                text="Resume" if paused else "Pause",
                state=("normal" if lifecycle in (RUNTIME_RUNNING, RUNTIME_PAUSED) else "disabled"),
            )
            replace_allowed, _ = gui_settings_snapshot_replace_permission(t)
            replacement_state = "normal" if replace_allowed else "disabled"
            self.load_preset_btn.config(state=replacement_state)
            self.defaults_btn.config(state=replacement_state)

            state_style = {
                RUNTIME_STOPPED: ("Stopped", "#a33"),
                RUNTIME_STARTING: ("Starting", "#a60"),
                RUNTIME_RUNNING: ("Running", "#2a2"),
                RUNTIME_PAUSED: ("Paused", "#a60"),
                RUNTIME_STOPPING: ("Stopping", "#a60"),
                RUNTIME_FAULTED: ("Faulted", "#a33"),
            }
            label, colour = state_style.get(lifecycle, (lifecycle, "#a33"))
            self.state_lbl.config(text=label, foreground=colour)

        def _on_curve_preset(self, _event=None) -> None:
            self._on_combo("curve_preset")

        def _can_replace_settings_snapshot(self) -> bool:
            allowed, reason = gui_settings_snapshot_replace_permission(
                self.engine.telemetry()
            )
            if not allowed:
                self.status.config(text=reason)
            return allowed

        def _save_preset(self) -> None:
            path = filedialog.asksaveasfilename(
                defaultextension=".json",
                initialfile=PRESET_PATH.name,
                filetypes=[("JSON preset", "*.json")],
            )
            if path:
                try:
                    saved = save_preset(self.engine.get_settings(), Path(path))
                    backup = (
                        f"; retained backup {saved.backup_path.name}"
                        if saved.backup_path is not None
                        else ""
                    )
                    self.status.config(text=f"Saved schema-v2 preset {path}{backup}")
                except (OSError, ValueError) as exc:
                    messagebox.showerror("Save preset", str(exc))

        def _load_preset(self) -> None:
            if not self._can_replace_settings_snapshot():
                return
            path = filedialog.askopenfilename(
                filetypes=[("JSON preset", "*.json")]
            )
            if not path:
                return
            result = load_preset_result(Path(path))
            if not result.usable or result.settings is None:
                messagebox.showerror("Load preset", format_preset_load_result(result))
                return
            loaded = result.settings
            self.engine.replace_settings(loaded)
            self._refresh_widgets(loaded)
            notices = " ".join(result.notices)
            backend_note = ""
            if loaded.input_mode != input_backend:
                backend_note = (
                    f" Profile requests {loaded.input_mode}; current input remains "
                    f"{input_backend} until restart or an explicit mode selection."
                )
            self.status.config(text=f"Loaded {path}. {notices}{backend_note}".strip())
            if result.migration_required:
                messagebox.showinfo("Legacy profile loaded", " ".join(result.notices))

        def _reset_defaults(self) -> None:
            if not self._can_replace_settings_snapshot():
                return
            defaults = new_v2_settings()
            self.engine.replace_settings(defaults)
            self._refresh_widgets(defaults)
            self.status.config(text="All settings reset to defaults.")

        def _refresh_widgets(self, s: Settings) -> None:
            """Re-render every row for a whole-snapshot replacement.

            A loaded profile may change semantics, so this re-reads the bound
            field, its unit, and its range from the contract instead of assuming
            the previous widgets still describe the active law.
            """
            self._in_update = True
            try:
                for row in GUI_ROWS:
                    pair = self.vars.get(row.key)
                    if pair is None:
                        continue
                    var, fmt = pair
                    if row.boolean:
                        var.set(bool(getattr(s, row.key)))
                        continue
                    value = self._widget_value(gui_row_field(row, s), s)
                    var.set(value)
                    if fmt and row.key in self.labels:
                        self.labels[row.key].config(text=fmt % value)
                self.mode_var.set(s.steering_mode)
                self._update_mode_hint(s)
                self._rebind_rows(s)
            finally:
                self._in_update = False

        def _rebind_rows(self, s: Settings) -> None:
            """Re-apply unit-contract labels and ranges after a replacement."""
            for row in GUI_ROWS:
                widgets = self.row_widgets.get(row.key)
                if widgets is None:
                    continue
                _, label_widget, scale = widgets
                label_widget.config(text=gui_field_label(row, s))
                if scale is not None and not row.boolean:
                    lo, hi, _res, _fmt = gui_row_domain(row, s)
                    scale.config(from_=lo, to=hi)

        def _poll(self) -> None:
            try:
                self._calibration_poll()
                t = self.engine.telemetry()
                s = self.engine.get_settings()
                view = build_gui_runtime_view(s, t, profile_label=profile_label)
                self._render_diagnostics(view)
                c = self.canvas
                c.delete("all")
                w, h, y0 = 760, 86, 46
                cx = w // 2
                half = w // 2 - 30

                c.create_line(30, y0, w - 30, y0, fill="#444")
                c.create_line(cx, y0 - 18, cx, y0 + 18, fill="#888")

                lock_r = max(s.lock_right, 0.01)
                lock_l = max(s.lock_left, 0.01)
                xr = cx + half * lock_r
                xl = cx - half * lock_l
                c.create_line(xr, y0 - 12, xr, y0 + 12, fill="#a55")
                c.create_line(xl, y0 - 12, xl, y0 + 12, fill="#a55")
                c.create_text(xr + 14, y0 + 22, text="R lock", fill="#a55")
                c.create_text(xl - 14, y0 + 22, text="L lock", fill="#a55")

                pos = float(t["pos"])
                target = float(t.get("axis_target", t["out"]))
                out = float(t.get("axis_output", t["out"]))
                x_pos = cx + half * pos
                x_out = cx + half * out
                c.create_line(cx, y0, x_pos, y0, fill="#2a7", width=6)
                c.create_line(x_pos, y0 - 14, x_pos, y0 + 14, fill="#5f5", width=2)
                c.create_oval(
                    x_out - 5,
                    y0 - 5,
                    x_out + 5,
                    y0 + 5,
                    outline="#8cf",
                    fill="#26a",
                )

                state = str(t.get("lifecycle_state", RUNTIME_STOPPED))
                mode = str(t.get("steering_mode", s.steering_mode))
                centering = str(t.get("centering_state", "idle"))
                extra = "  [shoving at lock]" if t["saturated"] else ""
                if t.get("input_degraded"):
                    extra += "  [DEGRADED CURSOR]"
                    if t.get("cursor_warp_pending"):
                        extra += "  [recenter pending]"
                    if t.get("cursor_rebase_required"):
                        extra += "  [awaiting safe rebase]"
                if t.get("late_tick"):
                    extra += (
                        f"  [LATE +{float(t.get('integration_lag_s', 0.0)) * 1000:.0f} ms"
                        f", #{int(t.get('late_tick_count', 0))}]"
                    )
                c.create_text(
                    30,
                    16,
                    anchor="w",
                    fill="#aaa",
                    text=f"{state}  {mode}  [{centering}]{extra}",
                )
                timing = (
                    f"   LATE: {float(t.get('integration_dt_s', 0.0)) * 1000:.0f} ms slice"
                    if t.get("late_tick")
                    else f"   loop {float(t.get('loop_dt_s', 0.0)) * 1000:.1f} ms"
                )
                input_backend_name = str(
                    t.get("input_backend", INPUT_BACKEND_RAW_INPUT)
                )
                input_diag = ""
                if input_backend_name == INPUT_BACKEND_RAW_INPUT:
                    input_diag = (
                        f"   Raw q {int(t.get('pending_input_events', 0))}"
                        f"/{int(t.get('input_queue_capacity', 0))}"
                        f" drop {int(t.get('input_dropped_events', 0))}"
                    )
                    packet_errors = int(t.get("input_packet_errors", 0))
                    if packet_errors:
                        input_diag += f" err {packet_errors}"
                elif t.get("input_degraded"):
                    input_diag = (
                        "   DEGRADED cursor"
                        f" warp {int(t.get('cursor_warp_attempt_count', 0))}"
                        f" fail {int(t.get('cursor_warp_failure_count', 0))}"
                    )
                self.meter_lbl.config(
                    text=(
                        f"steer {pos:+.3f}   target {target:+.1%}"
                        f"   sent {out:+.1%}{timing}{input_diag}"
                    )
                )

                l, r = travel_px(s)
                source_unit = (
                    "counts"
                    if input_backend_name == INPUT_BACKEND_RAW_INPUT
                    else "px"
                )
                self.travel_lbl.config(
                    text=(
                        f"Travel to full lock:  L ≈ {l:.0f} {source_unit}"
                        f"   R ≈ {r:.0f} {source_unit}"
                    )
                )

                if time.monotonic() < self._status_hold_until:
                    # A refusal, stop result, or fault explanation is being held
                    # on purpose; the periodic mirror must not overwrite it.
                    self._sync_buttons()
                    return

                shutdown_status = str(t.get("shutdown_status", STOP_RESULT_NOT_REQUESTED))
                cleanup_complete = bool(t.get("session_cleanup_complete", True))
                if shutdown_status == STOP_RESULT_STOPPING_TIMEOUT:
                    timeout_message = format_telemetry_diagnostic(t)
                    self.status.config(
                        text=(
                            timeout_message
                            if t.get("error_type")
                            else (
                                "Shutdown still pending — retry Stop or wait; do not "
                                "assume the virtual pad is disconnected."
                            )
                        )
                    )
                elif shutdown_status == STOP_RESULT_CALLED_FROM_WORKER:
                    self.status.config(
                        text=(
                            "Runtime callback requested stop; owner cleanup is pending. "
                            "Do not assume the virtual pad is disconnected."
                        )
                    )
                elif state == RUNTIME_FAULTED and not cleanup_complete:
                    fault_message = format_telemetry_diagnostic(t)
                    self.status.config(
                        text=(
                            f"{fault_message} Retry cleanup before starting a new session."
                            if t.get("error_type")
                            else (
                                "Engine faulted; cleanup is pending. Retry cleanup before "
                                "starting a new session."
                            )
                        )
                    )
                elif t.get("error") and not t["running"]:
                    self.status.config(text=format_telemetry_diagnostic(t))
                elif t.get("input_source_error"):
                    source_label = (
                        "Raw Input"
                        if input_backend_name == INPUT_BACKEND_RAW_INPUT
                        else "DEGRADED cursor fallback"
                    )
                    self.status.config(
                        text=f"{source_label}: {t['input_source_error']}"
                    )

                self._sync_buttons()
            finally:
                if self.winfo_exists():
                    self.after(50, self._poll)

        def _on_close(self) -> None:
            """Close only after cleanup is confirmed; otherwise stay open.

            Destroying the window on an unconfirmed stop would present an
            unverified cleanup as a normal safe disconnect and would remove the
            only place the driver can retry it.
            """
            if self.cal_runner is not None:
                # Release the calibration input source before the window goes
                # away: it is not the engine's resource to clean up, and leaving a
                # listener running after the UI is gone would be dishonest.
                self.cal_runner.stop()
                self.cal_runner = None
            try:
                result = self.engine.stop()
                may_close, message = gui_close_window_decision(
                    result, self.engine.telemetry()
                )
                self._hold_status(message, seconds=6.0)
                if may_close:
                    self.destroy()
                    return
                # Retain the window as an honest retry surface with the
                # stopping/fault state visible rather than implied.
                self._show_stop_result(result)
                self._close_pending = True
            except Exception as exc:
                self._show_runtime_exception(exc)
                self._close_pending = True
            finally:
                if self.winfo_exists():
                    self._sync_buttons()

    app = SteeringApp()
    app.mainloop()


# ===========================================================================
# ENTRY POINT
# ===========================================================================


class _LauncherArgumentParser(argparse.ArgumentParser):
    """Make invalid launcher invocations self-correcting for scripts/users.

    ``argparse`` normally emits only a compact usage line on error. Step 20's
    launcher contract promises help text too, so retain the standard status-2
    behavior while including the full generated option reference on stderr.
    Help is intentionally delayed until syntactic parsing completes: a trailing
    unknown, invalid, or mutually exclusive flag must not be hidden merely
    because ``--help`` appeared earlier in the invocation.
    """

    def error(self, message: str) -> None:
        self.exit(2, f"{self.prog}: error: {message}\n\n{self.format_help()}")

    def parse_args(self, args=None, namespace=None) -> argparse.Namespace:
        options = super().parse_args(args, namespace)
        # Keep launcher-specific incompatibility validation in the same parser
        # authority as argparse's choices and mutually exclusive groups. This
        # also ensures --help cannot conceal a semantically invalid invocation.
        if options.dpi_diagnostics and (
            options.cli
            or options.profile is not None
            or options.input_mode is not None
            or options.calibrate
            or options.profile_export is not None
            or options.profile_import is not None
        ):
            self.error(
                "--dpi-diagnostics cannot be combined with --cli, --profile, "
                "--calibrate, --profile-export, --profile-import, or an "
                "input-mode override"
            )
        if options.calibrate_live and not options.calibrate:
            self.error(
                "--calibrate-live only applies to --calibrate; add --calibrate "
                "or drop --calibrate-live"
            )
        if options.calibrate_write is not None and not options.calibrate:
            self.error(
                "--calibrate-write only applies to --calibrate; add --calibrate "
                "or drop --calibrate-write"
            )
        if options.profile_export is not None and options.calibrate:
            # One writes the file, the other offers to write it: combining them
            # would make the destination ambiguous.
            self.error(
                "--profile-export writes a file and --calibrate offers one; use "
                "one at a time"
            )
        if options._show_launcher_help:
            self.print_help()
            self.exit(0)
        # It is an implementation detail of delayed help, not a launcher
        # setting that callers or diagnostic logs should observe.
        delattr(options, "_show_launcher_help")
        return options


def build_argument_parser() -> argparse.ArgumentParser:
    """Build the dependency-free command-line contract for launchers/scripts."""
    parser = _LauncherArgumentParser(
        prog="mouse_steering.py",
        # The custom delayed help action below validates the whole command line
        # before exiting, unlike argparse's eager default help action.
        add_help=False,
        # Script/launcher contracts must not let a typo become an abbreviated
        # different control (for example, --cl silently meaning --cli).
        allow_abbrev=False,
        description=(
            "Map mouse movement to a virtual Xbox steering axis. Windows Raw "
            "Input is the normal backend; cursor fallback is visibly degraded."
        ),
        epilog=(
            "A missing default preset starts intentional native-v2 defaults. "
            "An explicitly requested profile must load successfully; this "
            "avoids launching a script with unrelated defaults."
        ),
    )
    parser.add_argument(
        "-h",
        "--help",
        dest="_show_launcher_help",
        action="store_true",
        help="show this help message and exit",
    )
    parser.add_argument(
        "--cli",
        action="store_true",
        help="run the console interface instead of the tuning GUI",
    )
    parser.add_argument(
        "--profile",
        type=Path,
        metavar="PATH",
        default=None,
        help=(
            "load this preset path (must be usable); without this option the "
            "normal default preset path is used"
        ),
    )
    input_group = parser.add_mutually_exclusive_group()
    input_group.add_argument(
        "--input-mode",
        choices=INPUT_BACKENDS,
        metavar="{raw_input,cursor_fallback}",
        help="override the profile input backend for this launch",
    )
    # Retain established Step-15 spellings as real compatibility aliases while
    # making --input-mode the one documented canonical setting name.
    input_group.add_argument(
        "--raw-input",
        dest="input_mode",
        action="store_const",
        const=INPUT_BACKEND_RAW_INPUT,
        help="compatibility alias for --input-mode raw_input",
    )
    input_group.add_argument(
        "--cursor-fallback",
        dest="input_mode",
        action="store_const",
        const=INPUT_BACKEND_CURSOR_FALLBACK,
        help="compatibility alias for --input-mode cursor_fallback (DEGRADED)",
    )
    parser.add_argument(
        "--log-level",
        type=str.lower,
        choices=LOG_LEVEL_CHOICES,
        metavar="LEVEL",
        default="warning",
        help="application diagnostic level: " + ", ".join(LOG_LEVEL_CHOICES),
    )
    parser.add_argument(
        "--dpi-diagnostics",
        action="store_true",
        help="print dependency-free Windows DPI/coordinate diagnostics and exit",
    )
    # ----- Step 23: calibration and the layered profile workflow -----------
    parser.add_argument(
        "--calibrate",
        action="store_true",
        help=(
            "measure the mouse and offer a conservative profile; runs input-only, "
            "never a virtual pad"
        ),
    )
    parser.add_argument(
        "--calibrate-write",
        type=Path,
        metavar="PATH",
        default=None,
        help=(
            "save the calibrated profile here (default: report only, nothing is "
            "written)"
        ),
    )
    parser.add_argument(
        "--calibrate-live",
        action="store_true",
        help=(
            "measure the output trace too: keep a real steering session running "
            "for the duration (opt-in; the virtual pad steers while you measure) "
            "so the profile records the axis the pad actually received"
        ),
    )
    export_group = parser.add_mutually_exclusive_group()
    export_group.add_argument(
        "--profile-export",
        type=Path,
        metavar="PATH",
        default=None,
        help=(
            "write the profile in use as a layered bundle (global/game/car/style) "
            "and exit"
        ),
    )
    export_group.add_argument(
        "--profile-import",
        type=Path,
        metavar="PATH",
        default=None,
        help=(
            "merge a layered bundle over the profile in use, then launch with "
            "the merged result; the bundle's units must match the profile, and "
            "the GUI's save writes it back"
        ),
    )
    return parser


def parse_command_line(argv: list[str] | None = None) -> argparse.Namespace:
    """Parse and cross-validate launcher options before any platform setup."""
    return build_argument_parser().parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    """Run one parsed launch and return a script-friendly process status."""
    options = parse_command_line(sys.argv[1:] if argv is None else argv)
    log_level = configure_log_level(options.log_level)
    _LOGGER.debug("Parsed launch options: %s", vars(options))

    if options.dpi_diagnostics:
        # Diagnostics own their documented early-DPI path, but parser/help
        # errors above still exit before any platform call.
        configure_dpi_awareness()
        if sys.platform != "win32":
            print(
                "Warning: ViGEm is Windows-only; this program is intended for Windows.",
                file=sys.stderr,
            )
        _LOGGER.info("Running DPI diagnostics at %s level.", log_level)
        run_dpi_diagnostics()
        return 0

    profile_explicit = options.profile is not None
    profile_path = options.profile if profile_explicit else PRESET_PATH
    try:
        # Preserve the no-argument call for the ordinary default profile so
        # existing embedders/mocks retain its stable compatibility surface.
        preset_result = (
            load_preset_result(profile_path)
            if profile_explicit
            else load_preset_result()
        )
        if preset_result.status == "missing" and not profile_explicit:
            # A missing default profile is the one normal fallback case. New
            # defaults are intentionally native-v2: Raw Input, explicit units,
            # continuous spring, and axis-target return-time semantics.
            settings = new_v2_settings()
            profile_label = (
                f"no profile file at {profile_path}; intentional native-v2 "
                "defaults"
            )
        elif preset_result.usable and preset_result.settings is not None:
            settings = preset_result.settings
            profile_label = str(profile_path)
            for notice in preset_result.notices:
                print(f"Preset notice: {notice}", file=sys.stderr)
        elif profile_explicit:
            # An explicit path is an operator/launcher contract. Starting with
            # unrelated defaults after a typo, missing file, or corrupt profile
            # would be less safe and less deterministic than failing clearly.
            message = format_preset_load_result(preset_result)
            terminal = "" if message.endswith((".", "!", "?")) else "."
            print(
                f"Profile error: {message}{terminal} "
                "Refusing to start with unrelated defaults.",
                file=sys.stderr,
            )
            _LOGGER.debug("Explicit profile could not be used: %s", profile_path)
            return 2
        else:
            # Corrupt/unsupported/invalid *default* profiles are never silently
            # treated as ordinary missing settings. Continue only with a clear
            # warning and fresh v2 defaults so an interactive user can recover.
            message = format_preset_load_result(preset_result)
            terminal = "" if message.endswith((".", "!", "?")) else "."
            print(
                f"Preset warning: {message}{terminal} "
                "Starting with new schema-v2 defaults; the source file was not changed.",
                file=sys.stderr,
            )
            settings = new_v2_settings()
            profile_label = (
                f"{profile_path} was unusable; started from native-v2 defaults "
                "without rewriting it"
            )
        settings = sanitise_settings(settings)

        # ----- Phase 4 / Step 23: layered profiles and calibration ----------
        # All of this is pure I/O and validation, so it finishes before any
        # platform side effect, and before a session could ever steer anything.
        if options.profile_export is not None:
            bundle = calibration_contract.bundle_from_settings(
                settings,
                name=profile_path.stem or "current",
                calibration=dict(settings.calibration),
                notes=(
                    f"exported from {profile_path}",
                    "global holds the input/safety policy; game, car, and style "
                    "hold the layers that describe them",
                ),
            )
            bundle_path = Path(options.profile_export)
            try:
                backup = calibration_contract.write_bundle(bundle, bundle_path)
            except (OSError, ValueError) as exc:
                print(
                    f"Profile export failed: {type(exc).__name__}: {exc}",
                    file=sys.stderr,
                )
                return 2
            layers = ", ".join(layer.kind for layer in bundle.layers)
            print(f"Exported {layers} layers to {bundle_path}.")
            if backup is not None:
                print(f"Previous bundle retained at {backup}.")
            return 0

        if options.profile_import is not None:
            bundle_result = calibration_contract.load_bundle(
                Path(options.profile_import)
            )
            if not bundle_result.usable or bundle_result.bundle is None:
                print(
                    "Profile import failed: " + bundle_result.summary(),
                    file=sys.stderr,
                )
                return 2
            try:
                merged = calibration_contract.merge_bundle(
                    bundle_result.bundle, settings
                )
            except ValueError as exc:
                print(f"Profile import failed: {exc}", file=sys.stderr)
                return 2
            for notice in bundle_result.notices:
                print(f"Bundle notice: {notice}", file=sys.stderr)
            for line in merged.summary_lines():
                print(f"Bundle layer: {line}")
            if merged.warnings:
                for warning in merged.warnings:
                    print(f"Bundle warning: {warning}", file=sys.stderr)
            settings = merged.settings
            profile_label = (
                f"{profile_path} + {bundle_result.path.name} "
                f"({len(bundle_result.bundle.layers)} layers)"
            )

        if options.calibrate:
            configure_dpi_awareness()
            if sys.platform != "win32":
                print(
                    "Warning: ViGEm is Windows-only; this program is intended for "
                    "Windows.",
                    file=sys.stderr,
                )
            _LOGGER.info("Running the Step-23 calibration flow.")
            if options.calibrate_live:
                return _run_live_calibration(
                    settings,
                    input_backend=options.input_mode or settings.input_mode,
                    write_path=options.calibrate_write,
                )
            return run_calibration_console(
                settings,
                input_backend=options.input_mode or settings.input_mode,
                write_path=options.calibrate_write,
            )

        # Profile validation/I/O is pure and deliberately finishes before any
        # platform side effect. DPI still configures before run_gui() imports
        # Tk or engine.start() imports/starts input/listener resources.
        configure_dpi_awareness()
        if sys.platform != "win32":
            print(
                "Warning: ViGEm is Windows-only; this program is intended for Windows.",
                file=sys.stderr,
            )

        # An explicit command-line setting always wins. Without one, schema-v2
        # profile input mode is honored; a migrated v1 profile deliberately
        # remains cursor fallback until the user recalibrates it for Raw Input.
        input_backend = options.input_mode or settings.input_mode
        _LOGGER.info(
            "Launching %s with profile %s and input mode %s.",
            "CLI" if options.cli else "GUI",
            profile_path,
            input_backend,
        )
        if options.cli:
            run_cli(settings, input_backend=input_backend)
        else:
            run_gui(
                settings,
                input_backend=input_backend,
                profile_label=profile_label,
            )
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
