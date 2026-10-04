"""Operational safety controls: hotkeys, deadman hold, and foreground gating.

This module owns the *policy* for the three operational guards Phase 4 / Step 22
requires, plus the driver-facing wording that explains them:

1. configurable pause/stop hotkeys and a hold-to-enable **deadman** key;
2. an opt-in **foreground window/process allowlist** that auto-pauses steering
   when an unlisted window has focus;
3. the highly visible **input indicator** state for the GUI.

Design constraints, matching the rest of the project:

* Platform-free at import time. Win32 access happens only inside
  :class:`WindowsForegroundProbe` and only when it is read, so ``--help``, the
  deterministic suite, and non-Windows hosts never touch ``ctypes.windll``.
* No Tk, no threads, and no engine ownership here. The engine decides *when* to
  read a probe or act on an event; this module only decides what the reading
  means, so every policy below is unit-testable without a display or a mouse.
* These guards decide **whether** steering is applied. They never change how it
  is computed, and nothing here attempts to bypass a game's or platform's
  policies. See ``docs/GAME_POLICY.md``.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Any, Iterable, Sequence

# ---------------------------------------------------------------------------
# Constants and defaults
# ---------------------------------------------------------------------------

SAFETY_FAIL_CLOSED = "fail_closed"
SAFETY_FAIL_OPEN = "fail_open"
SAFETY_FAIL_MODES = (SAFETY_FAIL_CLOSED, SAFETY_FAIL_OPEN)

DEFAULT_HOTKEY_PAUSE = "<f8>"
DEFAULT_HOTKEY_STOP = "<f9>"
DEFAULT_DEADMAN_KEY = "<f10>"

#: How often a running session samples the foreground window. This is a
#: deliberate, cheap poll: a focus change must be noticed well before a driver
#: could steer, but it must not become a control-loop cost.
FOCUS_POLL_HZ = 10.0

#: A window is treated as "not allowed" only after the guard has been armed;
#: this is the ceiling the allowlist is validated against.
MAX_ALLOWLIST_ENTRIES = 32
MAX_ALLOWLIST_ENTRY_LENGTH = 120
MAX_KEY_SPEC_LENGTH = 32

# Gate states ---------------------------------------------------------------
GATE_OPEN = "open"
GATE_HELD = "held"
GATE_UNAVAILABLE = "unavailable"

# Pause causes --------------------------------------------------------------
PAUSE_CAUSE_USER = "user"
PAUSE_CAUSE_FOCUS = "focus_guard"
PAUSE_CAUSE_DEADMAN = "deadman"
SAFETY_PAUSE_CAUSES = frozenset((PAUSE_CAUSE_FOCUS, PAUSE_CAUSE_DEADMAN))

#: A session is reported as actively steering only while accepted input is at
#: most this old. It is intentionally close to one report at the slowest
#: supported rate (20 Hz) so the indicator means "input is moving the axis now".
ACTIVE_INPUT_WINDOW_S = 0.05

# Indicator severities ------------------------------------------------------
INDICATOR_ACTIVE = "active"
INDICATOR_HELD = "held"
INDICATOR_INACTIVE = "inactive"
INDICATOR_FAULT = "fault"
INDICATOR_DEGRADED = "degraded"

# Allowlist match prefixes --------------------------------------------------
ALLOWLIST_PREFIX_PROCESS = "proc:"
ALLOWLIST_PREFIX_TITLE = "title:"


# ---------------------------------------------------------------------------
# Key specifications
# ---------------------------------------------------------------------------

_MODIFIER_ALIASES = {
    "control": "ctrl",
    "ctrl": "ctrl",
    "alt": "alt",
    "altgr": "alt_gr",
    "alt_gr": "alt_gr",
    "shift": "shift",
    "cmd": "cmd",
    "super": "cmd",
    "windows": "cmd",
}
_MODIFIER_NAMES = frozenset(_MODIFIER_ALIASES.values())

_SINGLE_CHAR_RE = re.compile(
    r"^[A-Za-z0-9`~!@#$%^&*()\-_=+\[\]{};:'\",.<>/?\\|]$"
)
_IDENTIFIER_RE = re.compile(r"^[a-z][a-z0-9_]*$")

#: Named keys are validated against the vocabulary a keyboard listener can
#: actually report. Accepting any identifier would turn a typo ("strg") into a
#: binding that silently never fires, which is exactly the failure mode the
#: launcher contract (Step 20) rejected for command-line flags.
_KNOWN_KEY_NAMES = frozenset(
    (
        "alt",
        "alt_gr",
        "alt_l",
        "alt_r",
        "backspace",
        "caps_lock",
        "cmd",
        "cmd_l",
        "cmd_r",
        "ctrl",
        "ctrl_l",
        "ctrl_r",
        "delete",
        "down",
        "end",
        "enter",
        "esc",
        "home",
        "insert",
        "left",
        "media_fast_forward",
        "media_next",
        "media_play_pause",
        "media_previous",
        "media_rewind",
        "media_stop",
        "media_volume_down",
        "media_volume_mute",
        "media_volume_up",
        "menu",
        "num_lock",
        "page_down",
        "page_up",
        "pause",
        "print_screen",
        "right",
        "scroll_lock",
        "shift",
        "shift_l",
        "shift_r",
        "space",
        "tab",
        "up",
        "dead_abovecomma",
        "dead_abovedot",
        "dead_abovereversedcomma",
        "dead_abovering",
        "dead_acute",
        "dead_belowbreve",
        "dead_belowcircumflex",
        "dead_belowcomma",
        "dead_belowdiaeresis",
        "dead_belowdot",
        "dead_belowmacron",
        "dead_belowring",
        "dead_belowtilde",
        "dead_breve",
        "dead_caron",
        "dead_cedilla",
        "dead_circumflex",
        "dead_currency",
        "dead_diaeresis",
        "dead_doubleacute",
        "dead_grave",
        "dead_hook",
        "dead_horn",
        "dead_invertedbreve",
        "dead_iota",
        "dead_macron",
        "dead_ogonek",
        "dead_perispomeni",
        "dead_semivoiced_sound",
        "dead_stroke",
        "dead_tilde",
        "dead_voiced_sound",
    )
) | {f"f{index}" for index in range(1, 36)}


@dataclass(frozen=True)
class KeySpec:
    """A validated key or hotkey specification.

    ``parts`` is normalized to lowercase names without angle brackets. A single
    part is either a one-character key (``"a"``, ``"7"``) or a named key
    (``"f8"``, ``"space"``, ``"shift"``); longer chords are ``("ctrl", "shift",
    "h")``.
    """

    parts: tuple[str, ...]
    raw: str

    @property
    def name(self) -> str:
        """The single key this spec refers to (the chord's final key)."""
        return self.parts[-1]

    @property
    def modifiers(self) -> tuple[str, ...]:
        return self.parts[:-1]

    @property
    def is_chord(self) -> bool:
        return len(self.parts) > 1

    @property
    def is_single_character(self) -> bool:
        return len(self.name) == 1

    def canonical(self) -> str:
        """Return the ``pynput``-compatible spelling, e.g. ``<ctrl>+<shift>+h``."""
        return "+".join(
            part if len(part) == 1 else f"<{part}>" for part in self.parts
        )

    @property
    def label(self) -> str:
        """Return a driver-facing label, e.g. ``CTRL + SHIFT + H``."""
        return " + ".join(part.upper() for part in self.parts)


def parse_key_spec(
    text: Any,
    *,
    allow_chord: bool = True,
    allow_modifier_only: bool = False,
) -> KeySpec:
    """Validate and normalize a key/hotkey string.

    Accepts ``"f8"``, ``"<f8>"``, ``"F8"``, ``"a"``, ``"<shift>"`` and, when
    chords are allowed, ``"<ctrl>+<alt>+h"``. Invalid input raises ``ValueError``
    with a message intended for a status bar or a profile-validation report.
    """
    if not isinstance(text, str):
        raise ValueError("a key must be a string such as <f8> or <shift>+x")
    raw = text.strip()
    if not raw:
        raise ValueError("a key must not be empty")
    if len(raw) > MAX_KEY_SPEC_LENGTH:
        raise ValueError(
            f"a key must be at most {MAX_KEY_SPEC_LENGTH} characters"
        )

    parts: list[str] = []
    for piece in raw.split("+"):
        # Brackets must balance. Quietly repairing "<f8" would make a typo in a
        # profile indistinguishable from the intended binding.
        piece = piece.strip()
        if piece.startswith("<") or piece.endswith(">"):
            if not (piece.startswith("<") and piece.endswith(">")) or len(
                piece
            ) <= 2:
                raise ValueError(f"{raw!r} has an unbalanced key name")
        piece = piece.strip("<>").strip().lower()
        if not piece:
            raise ValueError(f"{raw!r} is not a valid key or hotkey")
        piece = _MODIFIER_ALIASES.get(piece, piece)
        if len(piece) > 1 and not _IDENTIFIER_RE.match(piece):
            raise ValueError(
                f"{piece!r} is not a valid key name (use single characters or "
                "names such as f8, space or shift)"
            )
        if len(piece) > 1 and piece not in _KNOWN_KEY_NAMES:
            raise ValueError(
                f"{piece!r} is not a key this program recognizes; use a single "
                "character such as x, or a name such as f8, space, tab, shift "
                "or media_play_pause"
            )
        parts.append(piece)

    if len(parts) > 1:
        if not allow_chord:
            raise ValueError(
                f"{raw!r} must be a single key; a held control cannot be a chord"
            )
        if len(parts) > 4:
            raise ValueError("a hotkey may combine at most four keys")
        for modifier in parts[:-1]:
            if modifier not in _MODIFIER_NAMES:
                raise ValueError(
                    f"{modifier!r} is not a modifier; use ctrl, alt, shift or cmd"
                )
        if parts[-1] in _MODIFIER_NAMES and not allow_modifier_only:
            raise ValueError(
                "a hotkey needs a final non-modifier key, for example <ctrl>+h"
            )
    elif parts[0] in _MODIFIER_NAMES and not allow_modifier_only:
        raise ValueError(
            f"{raw!r} is a modifier; a hotkey needs a non-modifier key, for "
            "example f8"
        )

    if len(parts) == 1:
        name = parts[0]
        if len(name) > 1 and not _IDENTIFIER_RE.match(name):
            raise ValueError(f"{raw!r} is not a valid key name")
        if len(name) == 1 and not _SINGLE_CHAR_RE.match(name):
            raise ValueError(f"{raw!r} is not a valid single-character key")

    return KeySpec(parts=tuple(parts), raw=raw)


def parse_single_key_spec(text: Any) -> KeySpec:
    """Validate a held control key (no chords, modifiers allowed)."""
    return parse_key_spec(text, allow_chord=False, allow_modifier_only=True)


def matches_key(spec: KeySpec, key: Any) -> bool:
    """Return whether a ``pynput`` keyboard event refers to ``spec``.

    Matching is duck-typed so deterministic fakes can stand in for real events:
    named keys expose ``.name`` (``"f8"``), character keys expose ``.char``.
    """
    if spec.is_chord:
        # Chords are handled by the hotkey listener, not by press/release state.
        return False
    name = getattr(key, "name", None)
    if name:
        return str(name).lower() == spec.name
    char = getattr(key, "char", None)
    if isinstance(char, str) and len(char) == 1:
        return spec.name == char.lower()
    return False


def describe_key_event(key: Any) -> str:
    """Return a short, log-friendly description of a keyboard event."""
    name = getattr(key, "name", None)
    if name:
        return str(name)
    char = getattr(key, "char", None)
    if isinstance(char, str) and char:
        return char
    return repr(key)


# ---------------------------------------------------------------------------
# Foreground allowlist
# ---------------------------------------------------------------------------


def normalise_allowlist(entries: Iterable[Any]) -> list[str]:
    """Clean an allowlist: strip blanks, de-duplicate, and bound the result."""
    if isinstance(entries, (str, bytes)) or not isinstance(entries, Iterable):
        raise ValueError("the focus allowlist must be a list of match rules")
    result: list[str] = []
    for entry in entries:
        if not isinstance(entry, str):
            raise ValueError("every focus allowlist entry must be a string")
        cleaned = " ".join(entry.split())
        if not cleaned:
            continue
        # "title: Rally" and "title:Rally" must be the same rule, otherwise a
        # GUI-edited entry would stop matching after a round trip.
        cleaned = re.sub(
            r"^(proc|title)\s*:\s*", r"\1:", cleaned, flags=re.IGNORECASE
        )
        # A prefixed rule with an empty value can never match any window. Left
        # in an armed allowlist it would hold steering forever, which is the
        # same trap as arming a guard with no rules at all, so refuse it here
        # rather than at the moment it silently never matches.
        match = re.match(r"^(proc|title):(.*)$", cleaned, flags=re.IGNORECASE)
        if match and not match.group(2).strip():
            raise ValueError(
                f"focus allowlist rule {cleaned!r} has no value after "
                f"'{match.group(1).lower()}:' and could never match a window"
            )
        if len(cleaned) > MAX_ALLOWLIST_ENTRY_LENGTH:
            raise ValueError(
                f"a focus allowlist entry must be at most "
                f"{MAX_ALLOWLIST_ENTRY_LENGTH} characters"
            )
        if cleaned.lower() not in {item.lower() for item in result}:
            result.append(cleaned)
    if len(result) > MAX_ALLOWLIST_ENTRIES:
        raise ValueError(
            f"the focus allowlist may hold at most {MAX_ALLOWLIST_ENTRIES} entries"
        )
    return result


def parse_allowlist_text(text: Any) -> list[str]:
    """Parse a GUI/CLI multiline allowlist into normalized entries."""
    if text is None:
        return []
    if not isinstance(text, str):
        raise ValueError("the focus allowlist must be text")
    return normalise_allowlist(re.split(r"[\n,;]+", text))


def format_allowlist_text(entries: Sequence[str]) -> str:
    """Render an allowlist for a text widget, one rule per line."""
    return "\n".join(entries)


def match_allowlist(
    entries: Sequence[str],
    process: str | None,
    title: str | None,
) -> str | None:
    """Return the first allowlist entry matching a window, or ``None``.

    Rules are intentionally simple and explainable:

    * ``proc:game.exe`` — the process basename must contain ``game.exe``;
    * ``title:Assetto`` — the window title must contain ``Assetto``;
    * ``game.exe`` — either of the above may match (a bare rule is a hint that
      the driver does not need to know which one matched).
    """
    process_text = (process or "").lower()
    title_text = (title or "").lower()
    for entry in entries:
        candidate = entry.strip().lower()
        if not candidate:
            continue
        if candidate.startswith(ALLOWLIST_PREFIX_PROCESS):
            needle = candidate[len(ALLOWLIST_PREFIX_PROCESS) :].strip()
            if needle and needle in process_text:
                return entry
            continue
        if candidate.startswith(ALLOWLIST_PREFIX_TITLE):
            needle = candidate[len(ALLOWLIST_PREFIX_TITLE) :].strip()
            if needle and needle in title_text:
                return entry
            continue
        if candidate in process_text or candidate in title_text:
            return entry
    return None


# ---------------------------------------------------------------------------
# Foreground probe
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class FocusReading:
    """One sample of the foreground window, or why it could not be read."""

    ok: bool
    process: str | None = None
    title: str | None = None
    detail: str | None = None

    @property
    def label(self) -> str:
        if not self.ok:
            return f"unavailable ({self.detail})" if self.detail else "unavailable"
        parts = [part for part in (self.process, self.title) if part]
        return " — ".join(parts) if parts else "unknown window"


class UnavailableForegroundProbe:
    """A probe that always reports a truthful, non-fatal failure."""

    def __init__(self, detail: str) -> None:
        self._detail = detail

    def read(self) -> FocusReading:
        return FocusReading(ok=False, detail=self._detail)


class StaticForegroundProbe:
    """A probe with a fixed reading, for tests and embedded callers."""

    def __init__(self, reading: FocusReading) -> None:
        self.reading = reading

    def read(self) -> FocusReading:
        return self.reading


class WindowsForegroundProbe:
    """Read the foreground window's process name and title on Windows.

    Win32 access is entirely lazy: constructing this probe touches nothing, and
    every import/binding error is converted into an ``ok=False`` reading rather
    than an exception, because a broken probe must never crash a control loop.
    """

    _PROCESS_QUERY_LIMITED_INFORMATION = 0x1000
    _MAX_TITLE = 512
    _MAX_PATH = 1024

    def __init__(self) -> None:
        self._ctypes: Any = None
        self._user32: Any = None
        self._kernel32: Any = None
        self._bind_error: str | None = None
        self.probe_error_count = 0
        self.last_error: str | None = None

    def _bind(self) -> None:
        if self._user32 is not None or self._bind_error is not None:
            return
        try:
            import ctypes

            self._ctypes = ctypes
            self._user32 = ctypes.windll.user32
            self._kernel32 = ctypes.windll.kernel32
        except Exception as exc:  # pragma: no cover - Windows-only boundary
            self._bind_error = f"{type(exc).__name__}: {exc}"

    def read(self) -> FocusReading:
        """Sample the foreground window without ever raising."""
        self._bind()
        if self._bind_error is not None:
            return FocusReading(
                ok=False, detail=f"foreground probing unavailable ({self._bind_error})"
            )
        try:
            return self._read_bound()
        except Exception as exc:  # pragma: no cover - Win32 boundary
            self.probe_error_count += 1
            self.last_error = f"{type(exc).__name__}: {exc}"
            return FocusReading(
                ok=False,
                detail=f"foreground probe failed ({self.last_error})",
            )

    def _read_bound(self) -> FocusReading:  # pragma: no cover - Windows-only
        ctypes = self._ctypes
        user32 = self._user32
        kernel32 = self._kernel32

        hwnd = user32.GetForegroundWindow()
        if not hwnd:
            return FocusReading(ok=False, detail="Windows reported no foreground window")

        process_id = ctypes.c_ulong(0)
        user32.GetWindowThreadProcessId(
            ctypes.c_void_p(hwnd), ctypes.byref(process_id)
        )

        process_name: str | None = None
        if process_id.value:
            handle = kernel32.OpenProcess(
                self._PROCESS_QUERY_LIMITED_INFORMATION, False, process_id.value
            )
            if handle:
                try:
                    buffer = ctypes.create_unicode_buffer(self._MAX_PATH)
                    size = ctypes.c_ulong(self._MAX_PATH)
                    if kernel32.QueryFullProcessImageNameW(
                        ctypes.c_void_p(handle), 0, buffer, ctypes.byref(size)
                    ):
                        process_name = buffer.value.rsplit("\\", 1)[-1] or None
                finally:
                    kernel32.CloseHandle(ctypes.c_void_p(handle))

        title_buffer = ctypes.create_unicode_buffer(self._MAX_TITLE)
        user32.GetWindowTextW(
            ctypes.c_void_p(hwnd), title_buffer, self._MAX_TITLE
        )
        title = title_buffer.value or None

        if not process_name and not title:
            return FocusReading(
                ok=False, detail="the foreground window exposed no process or title"
            )
        return FocusReading(ok=True, process=process_name, title=title)


def create_foreground_probe() -> Any:
    """Return the best available probe for this host, never raising."""
    import sys

    if sys.platform != "win32":
        return UnavailableForegroundProbe(
            "foreground window probing is Windows-only"
        )
    try:
        return WindowsForegroundProbe()
    except Exception as exc:  # pragma: no cover - platform boundary
        return UnavailableForegroundProbe(f"{type(exc).__name__}: {exc}")


# ---------------------------------------------------------------------------
# Gate evaluation
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class FocusAssessment:
    """Result of applying the allowlist policy to one reading."""

    enabled: bool
    allowed: bool | None
    target: str | None
    matched_entry: str | None
    detail: str


def focus_assessment(settings: Any, reading: FocusReading | None) -> FocusAssessment:
    """Decide whether the current foreground window permits steering."""
    if not getattr(settings, "focus_guard_enabled", False):
        return FocusAssessment(
            enabled=False,
            allowed=None,
            target=None,
            matched_entry=None,
            detail="focus guard is not armed",
        )
    entries = list(getattr(settings, "focus_allowlist", ()) or ())
    if not entries:
        return FocusAssessment(
            enabled=True,
            allowed=False,
            target=None,
            matched_entry=None,
            detail="the focus guard is armed but its allowlist is empty",
        )
    if reading is None:
        return FocusAssessment(
            enabled=True,
            allowed=None,
            target=None,
            matched_entry=None,
            detail="the foreground window has not been sampled yet",
        )
    if not reading.ok:
        return FocusAssessment(
            enabled=True,
            allowed=None,
            target=None,
            matched_entry=None,
            detail=reading.detail or "the foreground window could not be read",
        )
    matched = match_allowlist(entries, reading.process, reading.title)
    target = reading.label
    if matched is None:
        return FocusAssessment(
            enabled=True,
            allowed=False,
            target=target,
            matched_entry=None,
            detail=f"{target} is not in the focus allowlist",
        )
    return FocusAssessment(
        enabled=True,
        allowed=True,
        target=target,
        matched_entry=matched,
        detail=f"{target} matched allowlist entry {matched!r}",
    )


@dataclass(frozen=True)
class GateDecision:
    """The safety gate for one control report."""

    state: str
    open: bool
    cause: str | None
    reason: str
    warning: str | None = None
    applicable: bool = True
    focus: FocusAssessment | None = None

    @property
    def held(self) -> bool:
        return not self.open


def evaluate_safety_gate(
    settings: Any,
    *,
    lifecycle_state: str,
    deadman_held: bool,
    focus_reading: FocusReading | None,
) -> GateDecision:
    """Combine the deadman and focus guards into one decision.

    Ordering is deliberate: the deadman is the driver's own hand, so it is
    reported as the cause when both guards are closed. A focus-probe *failure*
    is handled by ``safety_fail_mode`` — ``fail_closed`` holds steering (the
    only setting that satisfies "an unlisted window cannot silently steer"),
    while ``fail_open`` keeps steering and reports a warning that the GUI and
    CLI surface loudly.
    """
    paused_or_running = lifecycle_state in ("RUNNING", "PAUSED")
    if not paused_or_running:
        return GateDecision(
            state=GATE_OPEN,
            open=True,
            cause=None,
            reason=(
                f"No armed session (state {lifecycle_state}); the safety gate "
                "is not being evaluated."
            ),
            applicable=False,
        )

    focus = focus_assessment(settings, focus_reading)
    reasons: list[str] = []
    cause: str | None = None
    warning: str | None = None
    unavailable = False

    deadman_enabled = bool(getattr(settings, "deadman_enabled", False))
    if deadman_enabled and not deadman_held:
        cause = PAUSE_CAUSE_DEADMAN
        key_label = _deadman_label(settings)
        reasons.append(f"the deadman key {key_label} is not held")

    if focus.enabled:
        fail_mode = getattr(settings, "safety_fail_mode", SAFETY_FAIL_CLOSED)
        if focus.allowed is True:
            pass
        elif focus.allowed is False:
            cause = cause or PAUSE_CAUSE_FOCUS
            reasons.append(focus.detail)
        else:
            unavailable = True
            if fail_mode == SAFETY_FAIL_OPEN:
                warning = (
                    "Focus guard could not read the foreground window "
                    f"({focus.detail}); steering continues because the guard is "
                    "set to fail open."
                )
            else:
                cause = cause or PAUSE_CAUSE_FOCUS
                reasons.append(
                    "the focus guard could not confirm the foreground window "
                    f"({focus.detail})"
                )

    if reasons:
        return GateDecision(
            state=GATE_HELD,
            open=False,
            cause=cause,
            reason="Steering held: " + "; ".join(reasons) + ".",
            warning=warning,
            focus=focus,
        )
    detail = focus.detail if focus.enabled else "no operational guard is armed"
    return GateDecision(
        state=GATE_UNAVAILABLE if unavailable else GATE_OPEN,
        open=True,
        cause=None,
        reason=f"Steering allowed: {detail}.",
        warning=warning,
        focus=focus,
    )


def _deadman_label(settings: Any) -> str:
    try:
        return parse_single_key_spec(getattr(settings, "deadman_key", "")).label
    except ValueError:
        return str(getattr(settings, "deadman_key", "?")) or "?"


# ---------------------------------------------------------------------------
# Driver-facing summaries
# ---------------------------------------------------------------------------


def safety_guard_configured(settings: Any) -> bool:
    """Whether any operational guard is armed in this settings snapshot."""
    return bool(
        getattr(settings, "deadman_enabled", False)
        or getattr(settings, "focus_guard_enabled", False)
    )


def safety_summary_line(settings: Any) -> str:
    """One sentence describing the active safety policy."""
    deadman = ""
    if getattr(settings, "deadman_enabled", False):
        deadman = f"deadman hold {_deadman_label(settings)}"
    focus = ""
    if getattr(settings, "focus_guard_enabled", False):
        entries = list(getattr(settings, "focus_allowlist", ()) or ())
        fail_mode = getattr(settings, "safety_fail_mode", SAFETY_FAIL_CLOSED)
        fail_label = (
            "fails closed" if fail_mode == SAFETY_FAIL_CLOSED else "fails open"
        )
        focus = (
            f"focus allowlist ({len(entries)} rule"
            f"{'s' if len(entries) != 1 else ''}, {fail_label})"
        )
    if not deadman and not focus:
        return (
            "NOT CONFIGURED — steering is active whenever the runtime session is "
            "running, even if another application has focus. Arm the deadman or "
            "focus guard in the Safety settings."
        )
    return "Armed: " + " + ".join(part for part in (deadman, focus) if part) + "."


def safety_policy_notice() -> str:
    """Return the standing policy notice shown by CLI/GUI and in the docs."""
    return (
        "Virtual-controller and mouse-input use must follow each game's and "
        "platform's rules. This program provides no anti-cheat bypass, and "
        "nothing here attempts to defeat or hide from one."
    )


def hotkey_summary_line(settings: Any) -> str:
    """Return the configured hotkeys as a driver-facing line."""
    pause = getattr(settings, "hotkey_pause", DEFAULT_HOTKEY_PAUSE)
    stop = getattr(settings, "hotkey_stop", DEFAULT_HOTKEY_STOP)
    pause_label = parse_key_spec(pause, allow_modifier_only=True).label
    stop_label = parse_key_spec(stop, allow_modifier_only=True).label
    return f"pause {pause_label}, stop {stop_label} (configurable)"


@dataclass(frozen=True)
class InputIndicator:
    """The highly visible "is steering happening?" banner state."""

    label: str
    severity: str
    detail: str

    @property
    def is_active(self) -> bool:
        return self.severity in (INDICATOR_ACTIVE, INDICATOR_DEGRADED)


def input_indicator(
    *,
    lifecycle_state: str,
    gate: GateDecision | None,
    input_age_s: float,
    filtered_velocity: float,
    pause_cause: str | None = None,
    input_degraded: bool = False,
    source_error: str | None = None,
) -> InputIndicator:
    """Return the indicator banner for one telemetry snapshot.

    The banner deliberately distinguishes *armed but idle* from *actively
    steering*: a driver must be able to see the instant the virtual pad starts
    receiving movement, and see equally clearly when a guard is holding it.
    """
    state = lifecycle_state or "STOPPED"
    if state == "FAULTED":
        return InputIndicator(
            "ENGINE FAULTED",
            INDICATOR_FAULT,
            "Cleanup may still be pending; the virtual pad may not be released.",
        )
    if state == "STOPPING":
        return InputIndicator(
            "STOPPING — PAD MAY STILL BE ACTIVE",
            INDICATOR_HELD,
            "Do not assume the virtual pad has been disconnected.",
        )
    if state == "STOPPED":
        return InputIndicator(
            "ENGINE STOPPED",
            INDICATOR_INACTIVE,
            "No virtual pad is being driven.",
        )
    if state == "STARTING":
        return InputIndicator(
            "STARTING",
            INDICATOR_INACTIVE,
            "Input capture and the virtual pad are being created.",
        )

    if gate is not None and gate.held:
        if gate.cause == PAUSE_CAUSE_DEADMAN:
            headline = "HELD — DEADMAN KEY NOT HELD"
        elif gate.cause == PAUSE_CAUSE_FOCUS:
            headline = "HELD — WINDOW NOT ALLOWED"
        else:
            headline = "HELD — SAFETY GUARD"
        return InputIndicator(headline, INDICATOR_HELD, gate.reason)

    if state == "PAUSED":
        detail = (
            "Paused by the operator; the pad is held neutral."
            if pause_cause in (None, PAUSE_CAUSE_USER)
            else f"Paused by {pause_cause}; the pad is held neutral."
        )
        return InputIndicator("PAUSED — PAD HELD NEUTRAL", INDICATOR_HELD, detail)

    if source_error:
        return InputIndicator(
            "RUNNING — INPUT SOURCE ERROR",
            INDICATOR_FAULT,
            str(source_error),
        )

    moving = abs(filtered_velocity) > 0.0 and input_age_s <= ACTIVE_INPUT_WINDOW_S
    if moving and input_degraded:
        return InputIndicator(
            "STEERING ACTIVE — DEGRADED INPUT",
            INDICATOR_DEGRADED,
            "Accepted cursor-fallback input is moving the virtual axis.",
        )
    if moving:
        return InputIndicator(
            "STEERING ACTIVE",
            INDICATOR_ACTIVE,
            (
                f"Accepted input {filtered_velocity:+.0f} units/s "
                f"{input_age_s * 1000.0:.0f} ms ago is moving the virtual axis."
            ),
        )
    return InputIndicator(
        "RUNNING — NO INPUT",
        INDICATOR_INACTIVE,
        "The engine is armed; the axis is not being moved right now.",
    )


__all__ = [
    "ACTIVE_INPUT_WINDOW_S",
    "ALLOWLIST_PREFIX_PROCESS",
    "ALLOWLIST_PREFIX_TITLE",
    "DEFAULT_DEADMAN_KEY",
    "DEFAULT_HOTKEY_PAUSE",
    "DEFAULT_HOTKEY_STOP",
    "FOCUS_POLL_HZ",
    "FocusAssessment",
    "FocusReading",
    "GATE_HELD",
    "GATE_OPEN",
    "GATE_UNAVAILABLE",
    "GateDecision",
    "INDICATOR_ACTIVE",
    "INDICATOR_DEGRADED",
    "INDICATOR_FAULT",
    "INDICATOR_HELD",
    "INDICATOR_INACTIVE",
    "InputIndicator",
    "KeySpec",
    "MAX_ALLOWLIST_ENTRIES",
    "PAUSE_CAUSE_DEADMAN",
    "PAUSE_CAUSE_FOCUS",
    "PAUSE_CAUSE_USER",
    "SAFETY_FAIL_CLOSED",
    "SAFETY_FAIL_MODES",
    "SAFETY_FAIL_OPEN",
    "SAFETY_PAUSE_CAUSES",
    "StaticForegroundProbe",
    "UnavailableForegroundProbe",
    "WindowsForegroundProbe",
    "create_foreground_probe",
    "describe_key_event",
    "evaluate_safety_gate",
    "focus_assessment",
    "format_allowlist_text",
    "hotkey_summary_line",
    "input_indicator",
    "match_allowlist",
    "matches_key",
    "normalise_allowlist",
    "parse_allowlist_text",
    "parse_key_spec",
    "parse_single_key_spec",
    "safety_guard_configured",
    "safety_policy_notice",
    "safety_summary_line",
]
