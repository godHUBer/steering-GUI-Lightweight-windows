"""Platform-independent steering settings, math, and preset handling.

This module is intentionally free of Tk, pynput, vgamepad, ctypes, and thread
runtime ownership. It was extracted from the original program in Phase 1 /
Step 4 and receives narrowly documented pure-core repairs in later steps.
"""
from __future__ import annotations

import json
import math
import os
import tempfile
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any

# Operational safety (Phase 4 / Step 22) is validated here so an invalid hotkey
# or an armed guard with an empty allowlist fails at profile-load time. The
# safety module is platform-free at import time (Win32 access is lazy inside its
# probe), so this keeps the core free of Tk/pynput/vgamepad/ctypes imports.
if __package__:
    from . import safety as safety_contract
else:
    import safety as safety_contract


PRESET_PATH = Path(__file__).with_name("mouse_steering.json")
PRESET_SCHEMA_V1 = 1
PRESET_SCHEMA_V2 = 2
CURRENT_PRESET_SCHEMA = PRESET_SCHEMA_V2

# Input selection is profile data in schema v2. The runtime facade keeps the
# same string values so an explicit command-line selection can safely override
# a profile without unit conversion or a hidden fallback.
INPUT_MODE_RAW_INPUT = "raw_input"
INPUT_MODE_CURSOR_FALLBACK = "cursor_fallback"
INPUT_MODES = (INPUT_MODE_RAW_INPUT, INPUT_MODE_CURSOR_FALLBACK)

# A migrated v1 profile retains the exact v1 control interpretation. A native
# v2 profile uses explicit source-rate/filter units and axis-target return-time
# semantics. Keeping this fact explicit is what prevents a version bump from
# silently changing steering feel.
PROFILE_SEMANTICS_LEGACY_V1 = "legacy_v1"
PROFILE_SEMANTICS_V2 = "v2"
FILTER_SEMANTICS_LEGACY_V1 = "legacy_v1"
FILTER_SEMANTICS_V2 = "v2"
CENTERING_SEMANTICS_RAW_POSITION = "raw_position"
CENTERING_SEMANTICS_AXIS_TARGET = "axis_target"

PRESET_STATUS_LOADED = "loaded"
PRESET_STATUS_MIGRATED = "migrated"
PRESET_STATUS_MISSING = "missing"
PRESET_STATUS_CORRUPT = "corrupt"
PRESET_STATUS_UNSUPPORTED_VERSION = "unsupported_version"
PRESET_STATUS_VALIDATION_ERROR = "validation_error"
PRESET_STATUS_IO_ERROR = "io_error"

CURVE_PRESETS = ["Linear", "Squared", "Cubed", "S-Curve", "Hybrid", "Custom power"]
STEERING_MODES = (
    "release_to_center",
    "continuous_spring",
    "manual",
)

# Reference raw-position input speed for a saturated steering-acceleration
# boost, in normalized locks/s.
ACCEL_V_REF = 4.0
CENTER_FLOOR = 0.15
PRECISION_BAND = 0.10
RAIL_EPS = 1e-6

# Phase 1 / Step 12 safety policy. A control report never integrates more than
# this elapsed interval in one state update. A late report additionally cannot
# move the *sent* axis by more than this normalized amount, even when the
# profile's normal output slew setting is unlimited.
MAX_CONTROL_INTEGRATION_S = 0.050
MAX_LATE_AXIS_STEP = 0.25

# Schema-v1 filtering fields are expressed as per-tick values. Phase 1 / Step
# 7 maps them to physical time using the original default report frequency,
# preserving their meaning at 83 Hz for retained legacy profiles. Native
# schema-v2 profiles use explicit milliseconds and source-units/second fields.
LEGACY_FILTER_REFERENCE_HZ = 83.0
# Schema-v1 profiles that omitted this field inherited a 2% hard output gate.
# New settings use the Step 9 virtual-axis default of zero; load_preset() keeps
# the historical missing-field value explicit for an existing v1 profile.
LEGACY_OUTPUT_DEADZONE_DEFAULT = 0.02


# ===========================================================================
# SETTINGS
# ===========================================================================


@dataclass
class Settings:
    # Input scaling / steering range
    full_lock_px: float = 500.0
    mouse_sensitivity: float = 1.0
    sens_left: float = 1.0
    sens_right: float = 1.0
    raw_scale: float = 1.0
    lock_left: float = 1.0
    lock_right: float = 1.0

    # Response curve
    curve_preset: str = "Cubed"
    curve_exp: float = 3.0
    precision_zone: float = 0.0
    precision_gain: float = 0.5

    # Steering dynamics
    steering_accel: float = 0.0
    # Legacy schema-v1 serialized name. It is the normal raw-position rate
    # base cap in normalized locks/s; 0 explicitly means unlimited.
    max_steering_rate: float = 6.0
    saturate_clean: bool = True

    # Input filtering (schema-v1 compatibility values calibrated at 83 Hz).
    # The core converts these to time-normalized velocity parameters; native
    # schema-v2 profiles use the explicit tau-ms and source-units/s fields
    # declared below instead.
    smoothing: float = 0.0
    noise_gate_px: float = 0.0
    hysteresis_px: float = 0.0

    # Centering. These retained schema-v1-compatible values operate on raw
    # position; native-v2 profiles use center_return_time_ms/axis-target
    # semantics declared below.
    center_strength: float = 1.0
    center_time_s: float = 0.30
    center_curve: float = 0.0
    idle_grace_ms: float = 60.0
    # Retained as the schema-v1 compatibility toggle. Sanitisation derives it
    # from steering_mode, and an omitted mode derives release/manual from it.
    auto_return_enabled: bool = True
    # Backward-compatible schema-v1 extension. Settings() intentionally keeps
    # the existing release-to-centre behavior; new_v2_settings() supplies the
    # native-v2 continuous-spring default explicitly.
    steering_mode: str = "release_to_center"

    # Output
    max_slew_rate: float = 0.0
    # Virtual axes start with no post-curve output deadzone. A nonzero value
    # is continuously remapped by the output stage rather than hard-clipped.
    output_deadzone: float = 0.0
    invert_axis: bool = False

    # Timing
    update_hz: float = 83.0

    # Optional schema-v1-compatible Step 11 extension. Appended so existing
    # positional Settings construction retains its prior field layout. Units
    # are normalized locks/s; 0 inherits max_steering_rate rather than meaning
    # an independent unlimited reversal cap.
    max_reversal_rate: float = 0.0

    # Schema-v2 runtime metadata. These values deliberately live after the
    # established v1 field layout so programmatic positional construction from
    # the original API remains compatible. Native-v2 JSON uses explicit field
    # names; legacy-v1 JSON never needs to know these implementation details.
    input_mode: str = INPUT_MODE_RAW_INPUT
    profile_semantics: str = PROFILE_SEMANTICS_LEGACY_V1
    filter_semantics: str = FILTER_SEMANTICS_LEGACY_V1
    smoothing_tau_ms: float = 0.0
    noise_gate_units_per_s: float = 0.0
    centering_hysteresis_units_per_s: float = 0.0
    centering_semantics: str = CENTERING_SEMANTICS_RAW_POSITION
    center_return_time_ms: float = 300.0
    # Unknown JSON fields are retained as metadata so loading them is visible
    # and an explicit v1->v2 migration does not silently erase future data.
    preset_unknown_fields: dict[str, Any] = field(
        default_factory=dict,
        repr=False,
        compare=False,
    )
    # Calibration provenance (Phase 4 / Step 23). This records *what was
    # measured* when the profile was calibrated -- the trace statistics the
    # numbers were derived from -- so a later run, or a reader of the file, can
    # tell a measured value from a hand-typed one. It is metadata: it never
    # changes how steering is computed, and it is excluded from equality so
    # existing comparisons keep working.
    calibration: dict[str, Any] = field(
        default_factory=dict,
        repr=False,
        compare=False,
    )

    # Operational safety (Phase 4 / Step 22). These values decide *whether*
    # steering is applied; they never change how it is computed, so they are
    # additive to every schema and cannot alter the feel of an existing profile.
    # ``focus_allowlist`` accepts "proc:name.exe", "title:substring", or a bare
    # rule that may match either (see src/safety.py). All guards default to off:
    # the journal's contract is opt-in allowlisting, and an armed guard with no
    # rules would hold steering forever.
    hotkey_pause: str = safety_contract.DEFAULT_HOTKEY_PAUSE
    hotkey_stop: str = safety_contract.DEFAULT_HOTKEY_STOP
    deadman_enabled: bool = False
    deadman_key: str = safety_contract.DEFAULT_DEADMAN_KEY
    focus_guard_enabled: bool = False
    focus_allowlist: list[str] = field(default_factory=list)
    safety_fail_mode: str = safety_contract.SAFETY_FAIL_CLOSED

    def to_dict(self) -> dict[str, Any]:
        """Return the stable schema-v1-compatible runtime field mapping.

        Schema-v2 serialisation is intentionally handled by ``save_preset``.
        Keeping this method's old shape preserves programmatic compatibility and
        makes it impossible to accidentally emit internal migration metadata as
        legacy settings.
        """
        return {name: getattr(self, name) for name in _LEGACY_SETTING_FIELD_NAMES}

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "Settings":
        """Construct a legacy-v1-compatible runtime settings snapshot.

        Use :func:`load_preset_result` for version-aware JSON loading. This
        method remains the narrow compatibility entry point used by pure-core
        callers and v1 migration.
        """
        if not isinstance(data, dict):
            raise ValueError("Preset root must be a JSON object.")
        values = {k: v for k, v in data.items() if k in _LEGACY_SETTING_FIELD_SET}
        unknown = {
            k: v for k, v in data.items() if k not in _LEGACY_SETTING_FIELD_SET
        }
        # Existing schema-v1 profiles know only auto_return_enabled. Preserve
        # their behavior explicitly rather than treating omission as a new
        # centering default: true is legacy release mode, false is manual.
        if "steering_mode" not in values:
            values["steering_mode"] = (
                "release_to_center"
                if values.get("auto_return_enabled", True)
                else "manual"
            )
        elif isinstance(values.get("auto_return_enabled", True), bool):
            # An explicit modern mode owns a conflicting legacy boolean on
            # JSON import. Keep malformed booleans intact so validation still
            # rejects them rather than silently accepting bad profile data.
            values["auto_return_enabled"] = values["steering_mode"] != "manual"
        settings = cls(
            **values,
            input_mode=INPUT_MODE_CURSOR_FALLBACK,
            profile_semantics=PROFILE_SEMANTICS_LEGACY_V1,
            filter_semantics=FILTER_SEMANTICS_LEGACY_V1,
            centering_semantics=CENTERING_SEMANTICS_RAW_POSITION,
            # Validate center_time_s through the normal numeric path before
            # deriving this internal compatibility mirror. Calling float() on
            # untrusted JSON here would otherwise let a list/dict escape the
            # detailed validation-result path as TypeError.
            center_return_time_ms=300.0,
            preset_unknown_fields=unknown,
        )
        validated = sanitise_settings(settings)
        return replace(
            validated,
            center_return_time_ms=validated.center_time_s * 1000.0,
        )


# Keep the historic field set explicit rather than deriving it from the
# dataclass: schema-v2 runtime metadata must never leak into a v1-compatible
# payload merely because a new internal field is added later.
_LEGACY_SETTING_FIELD_NAMES = (
    "full_lock_px",
    "mouse_sensitivity",
    "sens_left",
    "sens_right",
    "raw_scale",
    "lock_left",
    "lock_right",
    "curve_preset",
    "curve_exp",
    "precision_zone",
    "precision_gain",
    "steering_accel",
    "max_steering_rate",
    "saturate_clean",
    "smoothing",
    "noise_gate_px",
    "hysteresis_px",
    "center_strength",
    "center_time_s",
    "center_curve",
    "idle_grace_ms",
    "auto_return_enabled",
    "steering_mode",
    "max_slew_rate",
    "output_deadzone",
    "invert_axis",
    "update_hz",
    "max_reversal_rate",
)
_LEGACY_SETTING_FIELD_SET = frozenset(_LEGACY_SETTING_FIELD_NAMES)

# The exact public fields in a native schema-v2 ``settings`` object. Fields
# that have no implemented runtime behavior yet intentionally remain absent;
# accepting and discarding them would be a silent reinterpretation.
_V2_SETTING_FIELD_NAMES = frozenset(
    (
        "input_mode",
        "full_lock_input_units",
        "input_scale",
        "mouse_sensitivity",
        "sensitivity_left",
        "sensitivity_right",
        "lock_left",
        "lock_right",
        "curve_preset",
        "curve_exponent",
        "centre_position_zone",
        "centre_position_gain",
        "saturate_clean",
        "smoothing_tau_ms",
        "noise_gate_units_per_s",
        "centering_hysteresis_units_per_s",
        "max_position_rate_locks_per_s",
        "max_reversal_rate_locks_per_s",
        "steering_accel",
        "max_output_slew_sticks_per_s",
        "steering_mode",
        "idle_grace_ms",
        "center_return_time_ms",
        "center_strength",
        "center_curve",
        "output_axis_deadzone",
        "invert_axis",
        "update_hz",
    )
)


def _finite_number(name: str, value: Any) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be numeric.")
    try:
        value = float(value)
    except OverflowError as exc:
        raise ValueError(f"{name} must be finite.") from exc
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite.")
    return value


def _bool_value(name: str, value: Any) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{name} must be true or false.")
    return value


@dataclass(frozen=True)
class ControlInterval:
    """Bounded control-time plan for one scheduler report.

    ``integration_dt_s`` is deliberately separate from elapsed wall time. The
    runtime uses it to consume timestamped input only through the corresponding
    point on its control timeline, so a stalled scheduler cannot relabel a long
    burst of physical movement as one short, high-velocity sample.
    """

    integration_dt_s: float
    max_integration_dt_s: float
    late: bool


def plan_control_interval(elapsed_dt_s: float, nominal_dt_s: float) -> ControlInterval:
    """Return the bounded state-integration plan for an elapsed scheduler tick.

    The visual/control integration ceiling is fixed at 50 ms. A normal 20 Hz
    report is not called late merely because its nominal period is itself
    50 ms; any interval beyond that ceiling is late. At faster configured
    rates, reaching the 50 ms ceiling is itself late. The runtime retains
    timestamped input and works through any remaining timeline on following
    reports.
    """

    elapsed = _finite_number("elapsed_dt_s", elapsed_dt_s)
    nominal = _finite_number("nominal_dt_s", nominal_dt_s)
    if elapsed < 0.0:
        elapsed = 0.0
    if nominal <= 0.0:
        raise ValueError("nominal_dt_s must be > 0")

    epsilon = 1e-9
    nominal_at_ceiling = nominal >= MAX_CONTROL_INTEGRATION_S - epsilon
    late = (
        elapsed > MAX_CONTROL_INTEGRATION_S + epsilon
        if nominal_at_ceiling
        else elapsed >= MAX_CONTROL_INTEGRATION_S - epsilon
    )
    return ControlInterval(
        integration_dt_s=min(elapsed, MAX_CONTROL_INTEGRATION_S),
        max_integration_dt_s=MAX_CONTROL_INTEGRATION_S,
        late=late,
    )


def resolve_steering_mode(s: Settings) -> str:
    """Return the compatible effective centering mode for a settings snapshot.

    Core tests and programmatic callers may pass a freshly replaced Settings
    object before it has gone through sanitise_settings(). Treat the legacy
    false auto-return toggle as manual in that case, while an explicit new
    continuous/manual mode remains authoritative.
    """
    mode = getattr(s, "steering_mode", "release_to_center")
    if mode not in STEERING_MODES:
        return "release_to_center"
    if mode == "release_to_center" and not s.auto_return_enabled:
        return "manual"
    return mode


def sanitise_settings(settings: Settings) -> Settings:
    """Validate values from JSON/programmatic changes against GUI ranges."""
    s = replace(settings)

    numeric_ranges = {
        "full_lock_px": (100.0, 2000.0),
        "mouse_sensitivity": (0.05, 5.0),
        "sens_left": (0.05, 5.0),
        "sens_right": (0.05, 5.0),
        "raw_scale": (0.05, 5.0),
        "lock_left": (0.10, 1.0),
        "lock_right": (0.10, 1.0),
        "curve_exp": (0.5, 5.0),
        "precision_zone": (0.0, 0.5),
        "precision_gain": (0.05, 1.0),
        "steering_accel": (0.0, 2.0),
        # Step 11 gives the retained v1 position-rate field its documented
        # zero meaning. The new reversal cap uses zero specifically to inherit
        # that normal cap.
        "max_steering_rate": (0.0, 30.0),
        "max_reversal_rate": (0.0, 30.0),
        "smoothing": (0.0, 0.95),
        "noise_gate_px": (0.0, 10.0),
        "hysteresis_px": (0.0, 20.0),
        "center_strength": (0.0, 2.0),
        "center_time_s": (0.05, 2.0),
        "center_curve": (-1.0, 1.0),
        "idle_grace_ms": (0.0, 500.0),
        "max_slew_rate": (0.0, 100.0),
        "output_deadzone": (0.0, 0.2),
        "update_hz": (20.0, 500.0),
    }

    # Native-v2 files use physical/source-unit fields whose contract only
    # imposes meaningful lower bounds (and an output deadzone below one), not
    # the narrower slider limits historically inherited by a v1 cursor preset.
    # Keep those v1 limits intact for retained profiles while accepting the
    # documented v2 domain here.
    if s.profile_semantics == PROFILE_SEMANTICS_V2:
        numeric_ranges.update(
            {
                "full_lock_px": (0.0, None),
                "mouse_sensitivity": (0.0, None),
                "sens_left": (0.0, None),
                "sens_right": (0.0, None),
                "raw_scale": (0.0, None),
                "precision_gain": (0.0, None),
                "steering_accel": (0.0, None),
                "max_steering_rate": (0.0, None),
                "max_reversal_rate": (0.0, None),
                # These aliases remain editable by the pre-Step-21 GUI. When
                # a native profile edits one, the runtime translates it into
                # the explicit physical field rather than dropping the edit.
                "smoothing": (0.0, math.nextafter(1.0, 0.0)),
                "noise_gate_px": (0.0, None),
                "hysteresis_px": (0.0, None),
                # Kept only as an internal mirror while native profiles use
                # center_return_time_ms; it must not re-impose the old 2 s UI
                # ceiling after a valid native profile is re-sanitised.
                "center_time_s": (0.0, None),
                "center_strength": (0.0, None),
                "idle_grace_ms": (0.0, None),
                "max_slew_rate": (0.0, None),
                "output_deadzone": (0.0, math.nextafter(1.0, 0.0)),
                "update_hz": (0.0, None),
            }
        )

    for name, (lo, hi) in numeric_ranges.items():
        value = _finite_number(name, getattr(s, name))
        if value < lo or (hi is not None and value > hi):
            upper = "unbounded" if hi is None else str(hi)
            raise ValueError(f"{name} must be between {lo} and {upper}.")
        setattr(s, name, value)

    if s.profile_semantics == PROFILE_SEMANTICS_V2:
        for name in (
            "full_lock_px",
            "mouse_sensitivity",
            "sens_left",
            "sens_right",
            "raw_scale",
            "precision_gain",
            "update_hz",
        ):
            if getattr(s, name) <= 0.0:
                raise ValueError(f"{name} must be greater than zero for v2.")

    if s.curve_preset not in CURVE_PRESETS:
        raise ValueError(f"curve_preset must be one of: {', '.join(CURVE_PRESETS)}")
    if s.steering_mode not in STEERING_MODES:
        raise ValueError(
            f"steering_mode must be one of: {', '.join(STEERING_MODES)}"
        )

    s.saturate_clean = _bool_value("saturate_clean", s.saturate_clean)
    s.auto_return_enabled = _bool_value("auto_return_enabled", s.auto_return_enabled)
    s.invert_axis = _bool_value("invert_axis", s.invert_axis)

    if s.input_mode not in INPUT_MODES:
        raise ValueError(f"input_mode must be one of: {', '.join(INPUT_MODES)}")
    if s.profile_semantics not in (
        PROFILE_SEMANTICS_LEGACY_V1,
        PROFILE_SEMANTICS_V2,
    ):
        raise ValueError("profile_semantics must be legacy_v1 or v2.")
    if s.filter_semantics not in (
        FILTER_SEMANTICS_LEGACY_V1,
        FILTER_SEMANTICS_V2,
    ):
        raise ValueError("filter_semantics must be legacy_v1 or v2.")
    if s.centering_semantics not in (
        CENTERING_SEMANTICS_RAW_POSITION,
        CENTERING_SEMANTICS_AXIS_TARGET,
    ):
        raise ValueError("centering_semantics must be raw_position or axis_target.")

    v2_parameter_ranges = (
        ("smoothing_tau_ms", 0.0, None),
        ("noise_gate_units_per_s", 0.0, None),
        ("centering_hysteresis_units_per_s", 0.0, None),
        ("center_return_time_ms", 0.0, None),
    )
    for name, lo, hi in v2_parameter_ranges:
        value = _finite_number(name, getattr(s, name))
        if value < lo or (hi is not None and value > hi):
            upper = "unbounded" if hi is None else str(hi)
            raise ValueError(f"{name} must be between {lo} and {upper}.")
        setattr(s, name, value)
    if (
        s.profile_semantics == PROFILE_SEMANTICS_V2
        and s.center_return_time_ms <= 0.0
    ):
        raise ValueError("center_return_time_ms must be greater than zero for v2.")

    if not isinstance(s.preset_unknown_fields, dict):
        raise ValueError("preset_unknown_fields must be a mapping.")
    # Do not mutate a caller-owned unknown-field mapping through the copied
    # settings object. It is metadata, but preserving it exactly is part of
    # forward-compatible migration behavior.
    s.preset_unknown_fields = dict(s.preset_unknown_fields)

    # Native-v2 profiles always use their explicit unit-bearing filter and
    # axis-target return-time semantics. A legacy profile keeps its v1 bridge
    # even after an explicit v1->v2 envelope migration.
    if s.profile_semantics == PROFILE_SEMANTICS_V2:
        s.filter_semantics = FILTER_SEMANTICS_V2
        s.centering_semantics = CENTERING_SEMANTICS_AXIS_TARGET
        s.center_time_s = s.center_return_time_ms / 1000.0

    # Operational safety (Step 22). Validation is deliberately strict: a bad
    # hotkey or an armed guard with an empty allowlist must fail here, at load
    # time, rather than producing a session that can never steer.
    pause_spec = safety_contract.parse_key_spec(
        s.hotkey_pause, allow_modifier_only=True
    )
    stop_spec = safety_contract.parse_key_spec(
        s.hotkey_stop, allow_modifier_only=True
    )
    deadman_spec = safety_contract.parse_single_key_spec(s.deadman_key)
    if pause_spec.parts == stop_spec.parts:
        raise ValueError("hotkey_pause and hotkey_stop must be different keys")
    if (
        not pause_spec.is_chord
        and not stop_spec.is_chord
        and pause_spec.name == stop_spec.name
    ):
        raise ValueError("hotkey_pause and hotkey_stop must be different keys")
    if deadman_spec.name in (
        pair
        for spec in (pause_spec, stop_spec)
        if not spec.is_chord
        for pair in (spec.name,)
    ):
        raise ValueError(
            "deadman_key must differ from the configured pause/stop hotkeys"
        )
    s.hotkey_pause = pause_spec.canonical()
    s.hotkey_stop = stop_spec.canonical()
    s.deadman_key = deadman_spec.canonical()
    s.deadman_enabled = _bool_value("deadman_enabled", s.deadman_enabled)
    s.focus_guard_enabled = _bool_value(
        "focus_guard_enabled", s.focus_guard_enabled
    )
    s.focus_allowlist = safety_contract.normalise_allowlist(s.focus_allowlist)
    if s.focus_guard_enabled and not s.focus_allowlist:
        raise ValueError(
            "focus_guard_enabled requires at least one focus_allowlist rule; an "
            "empty allowlist would hold steering indefinitely"
        )
    if s.safety_fail_mode not in safety_contract.SAFETY_FAIL_MODES:
        raise ValueError(
            "safety_fail_mode must be one of: "
            + ", ".join(safety_contract.SAFETY_FAIL_MODES)
        )

    # An ambiguous direct Settings snapshot with legacy false and the default
    # release mode maps to manual. JSON import and engine mode selection
    # canonicalize their explicit-mode intent before reaching this fallback.
    # Thereafter steering_mode keeps the retained boolean serialised coherently.
    if s.steering_mode == "release_to_center" and not s.auto_return_enabled:
        s.steering_mode = "manual"
    s.auto_return_enabled = s.steering_mode != "manual"
    return s


def _legacy_filter_parameters(s: Settings) -> tuple[float, float, float, float]:
    """Return v1-bridged EMA/gate parameters in time-based source units.

    The return values are ``(tau_s, noise_velocity, hysteresis_velocity,
    output_lag_s)``. ``output_lag_s`` preserves v1's sampled-EMA integration
    at exactly 83 Hz while allowing the same filter state to be integrated
    consistently over any elapsed interval.

    This is a schema-v1 compatibility bridge. At exactly 83 Hz it reproduces
    the old coefficient and px/tick thresholds, while at other report rates it
    gives the same settings a time-based interpretation. Schema-v2 migration
    will replace these legacy field names with explicit unit-bearing fields.
    """
    smoothing = clamp(s.smoothing, 0.0, 0.95)
    if smoothing <= 0.0:
        tau_s = 0.0
        output_lag_s = 0.0
    else:
        tau_s = -1.0 / (LEGACY_FILTER_REFERENCE_HZ * math.log(smoothing))
        # At the reference rate this makes the interval integral exactly
        # equal to ``new_filtered_velocity / LEGACY_FILTER_REFERENCE_HZ``,
        # which is the archived v1 EMA-delta behavior.
        output_lag_s = (
            smoothing
            / (1.0 - smoothing)
            / LEGACY_FILTER_REFERENCE_HZ
        )
    noise_velocity = max(float(s.noise_gate_px), 0.0) * LEGACY_FILTER_REFERENCE_HZ
    hysteresis_velocity = (
        max(float(s.hysteresis_px), 0.0) * LEGACY_FILTER_REFERENCE_HZ
    )
    return tau_s, noise_velocity, hysteresis_velocity, output_lag_s


def _filter_parameters(s: Settings) -> tuple[float, float, float, float]:
    """Return the active filter parameters without conflating schema domains.

    Legacy-v1 profiles retain the calibrated 83 Hz bridge exactly. Native-v2
    profiles use their explicit physical units and the exact exponential
    integral, so a time constant is not reinterpreted as a per-report factor.
    """
    if s.filter_semantics == FILTER_SEMANTICS_V2:
        tau_s = s.smoothing_tau_ms / 1000.0
        return (
            tau_s,
            s.noise_gate_units_per_s,
            s.centering_hysteresis_units_per_s,
            tau_s,
        )
    return _legacy_filter_parameters(s)


@dataclass(frozen=True)
class PresetLoadResult:
    """Structured outcome for a versioned profile load attempt."""

    status: str
    path: Path
    settings: Settings | None = None
    source_version: int | None = None
    notices: tuple[str, ...] = ()
    unknown_fields: tuple[str, ...] = ()
    detail: str | None = None

    @property
    def usable(self) -> bool:
        return self.settings is not None and self.status in (
            PRESET_STATUS_LOADED,
            PRESET_STATUS_MIGRATED,
        )

    @property
    def migration_required(self) -> bool:
        return self.status == PRESET_STATUS_MIGRATED


@dataclass(frozen=True)
class PresetSaveResult:
    """Confirmed atomic-save outcome, including any retained prior profile."""

    path: Path
    backup_path: Path | None
    version: int = CURRENT_PRESET_SCHEMA
    notices: tuple[str, ...] = ()


def new_v2_settings() -> Settings:
    """Return intentional defaults for a newly created schema-v2 profile.

    ``Settings()`` remains a programmatic schema-v1-compatible snapshot so an
    existing caller is not silently switched to a new centering law. Product
    entry points call this factory when no preset exists or a user selects
    defaults, making the v2 defaults explicit and reviewable.
    """
    return sanitise_settings(
        Settings(
            input_mode=INPUT_MODE_RAW_INPUT,
            profile_semantics=PROFILE_SEMANTICS_V2,
            filter_semantics=FILTER_SEMANTICS_V2,
            centering_semantics=CENTERING_SEMANTICS_AXIS_TARGET,
            steering_mode="continuous_spring",
            auto_return_enabled=True,
            smoothing_tau_ms=0.0,
            noise_gate_units_per_s=0.0,
            centering_hysteresis_units_per_s=0.0,
            center_return_time_ms=300.0,
            output_deadzone=0.0,
        )
    )


def _json_constant_error(token: str):
    raise ValueError(f"Non-finite JSON number is not permitted: {token}")


def _legacy_settings_from_payload(
    payload: dict[str, Any],
    *,
    input_mode: str = INPUT_MODE_CURSOR_FALLBACK,
    unknown_fields: dict[str, Any] | None = None,
) -> Settings:
    """Build a retained-v1 behavior profile without changing its feel."""
    legacy_payload = dict(payload)
    # Schema-v1 profiles that omitted this field used the archived 2% hard
    # gate. The repaired runtime remaps it continuously, but preserving the
    # numeric threshold remains essential to avoid a silent default change.
    if "output_deadzone" not in legacy_payload:
        legacy_payload["output_deadzone"] = LEGACY_OUTPUT_DEADZONE_DEFAULT
    settings = Settings.from_dict(legacy_payload)
    retained_unknown = dict(settings.preset_unknown_fields)
    if unknown_fields:
        _merge_unknown_fields(retained_unknown, unknown_fields)
    return sanitise_settings(
        replace(
            settings,
            input_mode=input_mode,
            profile_semantics=PROFILE_SEMANTICS_LEGACY_V1,
            filter_semantics=FILTER_SEMANTICS_LEGACY_V1,
            centering_semantics=CENTERING_SEMANTICS_RAW_POSITION,
            center_return_time_ms=settings.center_time_s * 1000.0,
            preset_unknown_fields=retained_unknown,
        )
    )


def _v2_native_settings_from_payload(
    payload: dict[str, Any],
    *,
    unknown_fields: dict[str, Any] | None = None,
) -> Settings:
    """Map a native v2 public payload into the internal runtime snapshot."""
    base = new_v2_settings()
    values = {
        "input_mode": payload.get("input_mode", base.input_mode),
        "full_lock_px": payload.get("full_lock_input_units", base.full_lock_px),
        "raw_scale": payload.get("input_scale", base.raw_scale),
        "mouse_sensitivity": payload.get(
            "mouse_sensitivity", base.mouse_sensitivity
        ),
        "sens_left": payload.get("sensitivity_left", base.sens_left),
        "sens_right": payload.get("sensitivity_right", base.sens_right),
        "lock_left": payload.get("lock_left", base.lock_left),
        "lock_right": payload.get("lock_right", base.lock_right),
        "curve_preset": payload.get("curve_preset", base.curve_preset),
        "curve_exp": payload.get("curve_exponent", base.curve_exp),
        "precision_zone": payload.get(
            "centre_position_zone", base.precision_zone
        ),
        "precision_gain": payload.get(
            "centre_position_gain", base.precision_gain
        ),
        "saturate_clean": payload.get("saturate_clean", base.saturate_clean),
        "smoothing_tau_ms": payload.get(
            "smoothing_tau_ms", base.smoothing_tau_ms
        ),
        "noise_gate_units_per_s": payload.get(
            "noise_gate_units_per_s", base.noise_gate_units_per_s
        ),
        "centering_hysteresis_units_per_s": payload.get(
            "centering_hysteresis_units_per_s",
            base.centering_hysteresis_units_per_s,
        ),
        "max_steering_rate": payload.get(
            "max_position_rate_locks_per_s", base.max_steering_rate
        ),
        "max_reversal_rate": payload.get(
            "max_reversal_rate_locks_per_s", base.max_reversal_rate
        ),
        "steering_accel": payload.get("steering_accel", base.steering_accel),
        "max_slew_rate": payload.get(
            "max_output_slew_sticks_per_s", base.max_slew_rate
        ),
        "steering_mode": payload.get("steering_mode", base.steering_mode),
        "idle_grace_ms": payload.get("idle_grace_ms", base.idle_grace_ms),
        "center_return_time_ms": payload.get(
            "center_return_time_ms", base.center_return_time_ms
        ),
        "center_strength": payload.get("center_strength", base.center_strength),
        "center_curve": payload.get("center_curve", base.center_curve),
        "output_deadzone": payload.get(
            "output_axis_deadzone", base.output_deadzone
        ),
        "invert_axis": payload.get("invert_axis", base.invert_axis),
        "update_hz": payload.get("update_hz", base.update_hz),
        "profile_semantics": PROFILE_SEMANTICS_V2,
        "filter_semantics": FILTER_SEMANTICS_V2,
        "centering_semantics": CENTERING_SEMANTICS_AXIS_TARGET,
        "preset_unknown_fields": dict(unknown_fields or {}),
    }
    # The retained boolean is an internal compatibility alias. Native v2 mode
    # is authoritative, so it is derived instead of accepted as a hidden fourth
    # mode control.
    values["auto_return_enabled"] = values["steering_mode"] != "manual"
    return sanitise_settings(replace(base, **values))


def _native_v2_payload(settings: Settings) -> dict[str, Any]:
    """Serialise every implemented native-v2 public setting exactly once."""
    return {
        "input_mode": settings.input_mode,
        "full_lock_input_units": settings.full_lock_px,
        "input_scale": settings.raw_scale,
        "mouse_sensitivity": settings.mouse_sensitivity,
        "sensitivity_left": settings.sens_left,
        "sensitivity_right": settings.sens_right,
        "lock_left": settings.lock_left,
        "lock_right": settings.lock_right,
        "curve_preset": settings.curve_preset,
        "curve_exponent": settings.curve_exp,
        "centre_position_zone": settings.precision_zone,
        "centre_position_gain": settings.precision_gain,
        "saturate_clean": settings.saturate_clean,
        "smoothing_tau_ms": settings.smoothing_tau_ms,
        "noise_gate_units_per_s": settings.noise_gate_units_per_s,
        "centering_hysteresis_units_per_s": (
            settings.centering_hysteresis_units_per_s
        ),
        "max_position_rate_locks_per_s": settings.max_steering_rate,
        "max_reversal_rate_locks_per_s": settings.max_reversal_rate,
        "steering_accel": settings.steering_accel,
        "max_output_slew_sticks_per_s": settings.max_slew_rate,
        "steering_mode": settings.steering_mode,
        "idle_grace_ms": settings.idle_grace_ms,
        "center_return_time_ms": settings.center_return_time_ms,
        "center_strength": settings.center_strength,
        "center_curve": settings.center_curve,
        "output_axis_deadzone": settings.output_deadzone,
        "invert_axis": settings.invert_axis,
        "update_hz": settings.update_hz,
    }


def _serialise_preset(settings: Settings) -> tuple[dict[str, Any], tuple[str, ...]]:
    """Return v2 JSON without laundering a legacy behavior profile into v2."""
    if settings.profile_semantics == PROFILE_SEMANTICS_LEGACY_V1:
        payload: dict[str, Any] = {
            "version": CURRENT_PRESET_SCHEMA,
            "compatibility": {
                "mode": PROFILE_SEMANTICS_LEGACY_V1,
                "source_version": PRESET_SCHEMA_V1,
            },
            "settings": {
                "input_mode": settings.input_mode,
                "legacy_v1_settings": settings.to_dict(),
            },
            "safety": _safety_payload(settings),
        }
        if settings.calibration:
            payload["calibration"] = _calibration_payload(settings)
        notices = (
            "Saved as a schema-v2 legacy-compatibility profile; v1 filter and "
            "raw-position return semantics were retained.",
        )
    else:
        payload = {
            "version": CURRENT_PRESET_SCHEMA,
            "settings": _native_v2_payload(settings),
            "safety": _safety_payload(settings),
        }
        if settings.calibration:
            payload["calibration"] = _calibration_payload(settings)
        notices = ()
    if settings.preset_unknown_fields:
        payload["extensions"] = dict(settings.preset_unknown_fields)
    return payload, notices


# Operational safety is stored under its own top-level "safety" object rather
# than inside "settings". That keeps the documented schema-v1 field mapping
# (``Settings.to_dict()``) exactly as it was: an existing consumer never sees a
# new key, and a legacy profile can still carry an operational policy without
# its steering fields being reinterpreted.
_SAFETY_FIELD_NAMES = frozenset(
    (
        "hotkey_pause",
        "hotkey_stop",
        "deadman_enabled",
        "deadman_key",
        "focus_guard_enabled",
        "focus_allowlist",
        "safety_fail_mode",
    )
)


def _safety_payload(settings: Settings) -> dict[str, Any]:
    """Serialise every implemented operational-safety setting exactly once."""
    return {
        "hotkey_pause": settings.hotkey_pause,
        "hotkey_stop": settings.hotkey_stop,
        "deadman_enabled": settings.deadman_enabled,
        "deadman_key": settings.deadman_key,
        "focus_guard_enabled": settings.focus_guard_enabled,
        "focus_allowlist": list(settings.focus_allowlist),
        "safety_fail_mode": settings.safety_fail_mode,
    }


def _apply_safety_payload(
    settings: Settings,
    payload: Any,
    unknown_fields: dict[str, Any] | None = None,
) -> Settings:
    """Overlay a loaded ``safety`` section without dropping unknown keys."""
    if payload is None:
        return settings
    if not isinstance(payload, dict):
        raise ValueError("the preset safety section must be a JSON object.")
    values: dict[str, Any] = {}
    for key, value in payload.items():
        if key in _SAFETY_FIELD_NAMES:
            values[key] = value
        elif unknown_fields is not None:
            unknown_fields[f"safety.{key}"] = value
    return replace(settings, **values)


# Calibration provenance is stored under its own top-level "calibration" object,
# for the same reason safety is: the documented schema-v1 field mapping stays
# exactly as it was, and a legacy consumer never sees a new key.
_CALIBRATION_FIELD_NAMES = frozenset(
    (
        "schema",
        "backend",
        "reported_hz",
        "observed_hz",
        "elapsed_s",
        "noise_floor_units_per_s",
        "left_peak_units_per_s",
        "right_peak_units_per_s",
        "left_travel_units",
        "right_travel_units",
        "full_lock_units",
        "full_lock_observed",
        "output_axis_peak",
        "fields",
        "verification",
        "measured_at",
        "notes",
    )
)


def _calibration_payload(settings: Settings) -> dict[str, Any]:
    """Serialise the calibration provenance section, if the profile has one."""
    return {key: value for key, value in settings.calibration.items()}


def _apply_calibration_payload(
    settings: Settings,
    payload: Any,
    unknown_fields: dict[str, Any] | None = None,
) -> Settings:
    """Overlay a loaded ``calibration`` section without dropping unknown keys."""
    if payload is None:
        return settings
    if not isinstance(payload, dict):
        raise ValueError("the preset calibration section must be a JSON object.")
    values: dict[str, Any] = {}
    for key, value in payload.items():
        if key in _CALIBRATION_FIELD_NAMES:
            values[key] = value
        elif unknown_fields is not None:
            unknown_fields[f"calibration.{key}"] = value
    if values.get("schema") is not None and (
        isinstance(values["schema"], bool) or not isinstance(values["schema"], int)
    ):
        raise ValueError("the calibration schema must be an integer.")
    return replace(settings, calibration=values)


def _read_extensions(data: dict[str, Any]) -> dict[str, Any]:
    extensions = data.get("extensions", {})
    if not isinstance(extensions, dict):
        raise ValueError("extensions must be a JSON object.")
    return dict(extensions)


def _unknown_top_level_fields(data: dict[str, Any]) -> dict[str, Any]:
    known = {
        "version",
        "settings",
        "compatibility",
        "extensions",
        "safety",
        "calibration",
    }
    return {f"root.{key}": value for key, value in data.items() if key not in known}


def _merge_unknown_fields(
    target: dict[str, Any], additions: dict[str, Any]
) -> None:
    """Retain every forward-compatible value even when names collide.

    Generated path labels such as ``settings.future`` can themselves be used
    by a producer as an extension key. Preserve both values instead of letting
    a later ``dict.update`` silently overwrite one of them.
    """
    for key, value in additions.items():
        candidate = str(key)
        if candidate not in target or target[candidate] == value:
            target[candidate] = value
            continue
        suffix = 2
        alternate = f"{candidate} [retained collision {suffix}]"
        while alternate in target:
            suffix += 1
            alternate = f"{candidate} [retained collision {suffix}]"
        target[alternate] = value


def _unknown_notice(unknown: dict[str, Any]) -> tuple[str, ...]:
    if not unknown:
        return ()
    names = ", ".join(sorted(unknown))
    return (
        f"Unknown preset fields were retained under extensions: {names}.",
    )


def _load_v1_result(data: dict[str, Any], path: Path) -> PresetLoadResult:
    if "settings" in data:
        payload = data["settings"]
    else:
        payload = {key: value for key, value in data.items() if key != "version"}
    if not isinstance(payload, dict):
        raise ValueError("Schema-v1 settings must be a JSON object.")
    settings = _legacy_settings_from_payload(payload)
    safety_unknown: dict[str, Any] = {}
    settings = sanitise_settings(
        _apply_safety_payload(settings, data.get("safety"), safety_unknown)
    )
    settings = _apply_calibration_payload(
        settings, data.get("calibration"), safety_unknown
    )
    if safety_unknown:
        merged_unknown = dict(settings.preset_unknown_fields)
        merged_unknown.update(safety_unknown)
        settings = replace(settings, preset_unknown_fields=merged_unknown)
    notices = (
        "Loaded schema-v1 settings in legacy compatibility mode; the source "
        "file was not changed.",
        "Cursor-coordinate units, the 83 Hz filter bridge, raw-position "
        "return time, and legacy release/manual behavior were retained. Use an "
        "explicit save/migration to create a schema-v2 file and .bak backup.",
    ) + _unknown_notice(settings.preset_unknown_fields)
    return PresetLoadResult(
        status=PRESET_STATUS_MIGRATED,
        path=path,
        settings=settings,
        source_version=PRESET_SCHEMA_V1,
        notices=notices,
        unknown_fields=tuple(sorted(settings.preset_unknown_fields)),
    )


def _load_v2_result(data: dict[str, Any], path: Path) -> PresetLoadResult:
    payload = data.get("settings")
    if not isinstance(payload, dict):
        raise ValueError("Schema-v2 settings must be a JSON object.")
    extensions = _read_extensions(data)
    _merge_unknown_fields(extensions, _unknown_top_level_fields(data))
    compatibility = data.get("compatibility", {})
    if not isinstance(compatibility, dict):
        raise ValueError("compatibility must be a JSON object.")
    mode = compatibility.get("mode", PROFILE_SEMANTICS_V2)

    if mode == PROFILE_SEMANTICS_LEGACY_V1:
        legacy_payload = payload.get("legacy_v1_settings")
        if not isinstance(legacy_payload, dict):
            raise ValueError(
                "A legacy-v1 compatibility profile needs legacy_v1_settings."
            )
        wrapper_unknown = {
            f"settings.{key}": value
            for key, value in payload.items()
            if key not in {"input_mode", "legacy_v1_settings"}
        }
        compatibility_unknown = {
            f"compatibility.{key}": value
            for key, value in compatibility.items()
            if key not in {"mode", "source_version"}
        }
        _merge_unknown_fields(extensions, wrapper_unknown)
        _merge_unknown_fields(extensions, compatibility_unknown)
        settings = _legacy_settings_from_payload(
            legacy_payload,
            input_mode=payload.get("input_mode", INPUT_MODE_CURSOR_FALLBACK),
            unknown_fields=extensions,
        )
        settings = _apply_safety_payload(settings, data.get("safety"), extensions)
        settings = _apply_calibration_payload(
            settings, data.get("calibration"), extensions
        )
        settings = sanitise_settings(
            replace(settings, preset_unknown_fields=dict(extensions))
        )
        notices = (
            "Loaded a schema-v2 legacy-compatibility profile; v1 control "
            "semantics remain active until a deliberate recalibration creates "
            "a native-v2 profile.",
        ) + _unknown_notice(extensions)
        return PresetLoadResult(
            status=PRESET_STATUS_LOADED,
            path=path,
            settings=settings,
            source_version=PRESET_SCHEMA_V2,
            notices=notices,
            unknown_fields=tuple(sorted(extensions)),
        )

    if mode != PROFILE_SEMANTICS_V2:
        raise ValueError(f"Unknown compatibility mode: {mode!r}.")

    unknown = {
        f"settings.{key}": value
        for key, value in payload.items()
        if key not in _V2_SETTING_FIELD_NAMES
    }
    compatibility_unknown = {
        f"compatibility.{key}": value
        for key, value in compatibility.items()
        if key != "mode"
    }
    _merge_unknown_fields(extensions, unknown)
    _merge_unknown_fields(extensions, compatibility_unknown)
    settings = _v2_native_settings_from_payload(payload, unknown_fields=extensions)
    settings = _apply_safety_payload(settings, data.get("safety"), extensions)
    settings = _apply_calibration_payload(
        settings, data.get("calibration"), extensions
    )
    settings = sanitise_settings(
        replace(settings, preset_unknown_fields=dict(extensions))
    )
    return PresetLoadResult(
        status=PRESET_STATUS_LOADED,
        path=path,
        settings=settings,
        source_version=PRESET_SCHEMA_V2,
        notices=_unknown_notice(extensions),
        unknown_fields=tuple(sorted(extensions)),
    )


def load_preset_result(path: Path = PRESET_PATH) -> PresetLoadResult:
    """Load a preset with an explicit status rather than silent ``None`` loss."""
    profile_path = Path(path)
    try:
        text = profile_path.read_text(encoding="utf-8")
    except FileNotFoundError:
        return PresetLoadResult(
            status=PRESET_STATUS_MISSING,
            path=profile_path,
            detail="Preset file does not exist.",
        )
    except UnicodeError as exc:
        return PresetLoadResult(
            status=PRESET_STATUS_CORRUPT,
            path=profile_path,
            detail=f"Preset is not valid UTF-8 text: {exc}",
        )
    except OSError as exc:
        return PresetLoadResult(
            status=PRESET_STATUS_IO_ERROR,
            path=profile_path,
            detail=f"Could not read preset: {type(exc).__name__}: {exc}",
        )

    try:
        data = json.loads(text, parse_constant=_json_constant_error)
    except (json.JSONDecodeError, ValueError) as exc:
        return PresetLoadResult(
            status=PRESET_STATUS_CORRUPT,
            path=profile_path,
            detail=f"Malformed preset JSON: {exc}",
        )
    if not isinstance(data, dict):
        return PresetLoadResult(
            status=PRESET_STATUS_CORRUPT,
            path=profile_path,
            detail="Preset root must be a JSON object.",
        )

    version = data.get("version", PRESET_SCHEMA_V1)
    if isinstance(version, bool) or not isinstance(version, int):
        return PresetLoadResult(
            status=PRESET_STATUS_UNSUPPORTED_VERSION,
            path=profile_path,
            detail="Preset version must be an integer.",
        )
    if version not in (PRESET_SCHEMA_V1, PRESET_SCHEMA_V2):
        return PresetLoadResult(
            status=PRESET_STATUS_UNSUPPORTED_VERSION,
            path=profile_path,
            source_version=version,
            detail=(
                f"Unsupported preset version {version}; supported versions are "
                f"{PRESET_SCHEMA_V1} and {PRESET_SCHEMA_V2}."
            ),
        )

    try:
        if version == PRESET_SCHEMA_V1:
            return _load_v1_result(data, profile_path)
        return _load_v2_result(data, profile_path)
    except (OverflowError, TypeError, ValueError) as exc:
        return PresetLoadResult(
            status=PRESET_STATUS_VALIDATION_ERROR,
            path=profile_path,
            source_version=version,
            detail=str(exc),
        )


def format_preset_load_result(result: PresetLoadResult) -> str:
    """Return honest user-facing wording for CLI/GUI preset presentation."""
    if result.usable:
        suffix = " ".join(result.notices)
        return suffix or f"Loaded preset {result.path}."
    labels = {
        PRESET_STATUS_MISSING: "Preset not found",
        PRESET_STATUS_CORRUPT: "Preset is corrupt",
        PRESET_STATUS_UNSUPPORTED_VERSION: "Preset version is unsupported",
        PRESET_STATUS_VALIDATION_ERROR: "Preset validation failed",
        PRESET_STATUS_IO_ERROR: "Preset could not be read",
    }
    label = labels.get(result.status, "Preset could not be loaded")
    return f"{label}: {result.detail or result.path}"


def load_preset(path: Path = PRESET_PATH) -> Settings | None:
    """Compatibility wrapper returning settings only for a usable profile."""
    result = load_preset_result(path)
    return result.settings if result.usable else None


def _write_temp_bytes(directory: Path, stem: str, data: bytes) -> Path:
    fd, temporary_name = tempfile.mkstemp(
        prefix=f".{stem}.",
        suffix=".tmp",
        dir=str(directory),
    )
    temporary_path = Path(temporary_name)
    try:
        with os.fdopen(fd, "wb") as handle:
            handle.write(data)
            handle.flush()
            os.fsync(handle.fileno())
    except Exception:
        try:
            temporary_path.unlink()
        except OSError:
            pass
        raise
    return temporary_path


def _fsync_directory(directory: Path) -> None:
    """Best-effort metadata flush; unavailable filesystems remain supported."""
    try:
        descriptor = os.open(str(directory), os.O_RDONLY)
    except OSError:
        return
    try:
        os.fsync(descriptor)
    except OSError:
        pass
    finally:
        os.close(descriptor)


def write_json_atomically(path: Path, payload: dict[str, Any]) -> Path | None:
    """Public atomic JSON write: temp file, fsync, replace, prior bytes kept.

    Exposed so the Step-23 bundle exporter reuses exactly the same durability
    path as profile saving rather than growing a second implementation.
    """
    return _atomic_write_preset(path, payload)


def parse_json_object(text: str) -> dict[str, Any]:
    """Parse JSON text into an object, rejecting NaN/Infinity constants.

    Shared by the preset and bundle loaders so both refuse the same malformed
    inputs, and so a bundle can never be accepted where a preset would not be.
    """
    data = json.loads(text, parse_constant=_json_constant_error)
    if not isinstance(data, dict):
        raise ValueError("JSON root must be an object.")
    return data


def _atomic_write_preset(path: Path, payload: dict[str, Any]) -> Path | None:
    target = Path(path)
    encoded = (
        json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n"
    ).encode("utf-8")
    temporary_path: Path | None = None
    backup_temporary: Path | None = None
    backup_path: Path | None = None
    try:
        temporary_path = _write_temp_bytes(target.parent, target.name, encoded)
        if target.exists():
            backup_path = target.with_name(target.name + ".bak")
            backup_temporary = _write_temp_bytes(
                target.parent,
                backup_path.name,
                target.read_bytes(),
            )
            os.replace(backup_temporary, backup_path)
            backup_temporary = None
            # Make the recovery copy durable before attempting the only
            # operation that replaces the live target.
            _fsync_directory(target.parent)
        os.replace(temporary_path, target)
        temporary_path = None
        _fsync_directory(target.parent)
        return backup_path
    finally:
        for candidate in (temporary_path, backup_temporary):
            if candidate is not None:
                try:
                    candidate.unlink()
                except OSError:
                    pass


def save_preset(
    settings: Settings,
    path: Path = PRESET_PATH,
) -> PresetSaveResult:
    """Validate and atomically write schema-v2 JSON, retaining ``.bak`` first."""
    validated = sanitise_settings(settings)
    payload, notices = _serialise_preset(validated)
    backup_path = _atomic_write_preset(Path(path), payload)
    return PresetSaveResult(
        path=Path(path),
        backup_path=backup_path,
        notices=notices,
    )


def migrate_preset(path: Path = PRESET_PATH) -> PresetSaveResult:
    """Explicitly replace a readable v1 file with its v2 compatibility wrapper.

    The original bytes are first retained as ``<name>.bak``. No caller needs to
    accept a unit/centering reinterpretation merely to obtain an atomic v2
    envelope: migrated settings continue to execute the retained v1 behavior.
    """
    result = load_preset_result(path)
    if not result.usable or result.settings is None:
        raise ValueError(format_preset_load_result(result))
    if result.source_version == PRESET_SCHEMA_V2:
        return PresetSaveResult(
            path=Path(path),
            backup_path=None,
            notices=("Preset already uses schema version 2; no migration was written.",),
        )
    return save_preset(result.settings, path)


# ===========================================================================
# PURE HELPERS
# ===========================================================================


def clamp(value: float, lo: float = -1.0, hi: float = 1.0) -> float:
    return lo if value < lo else hi if value > hi else value


def smoothstep(t: float) -> float:
    t = clamp(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def response_curve(x: float, preset: str = "Cubed", exponent: float = 3.0) -> float:
    if x == 0.0:
        return 0.0
    a = abs(x)
    if preset == "Linear":
        r = a
    elif preset == "Squared":
        r = a * a
    elif preset == "S-Curve":
        r = a * a * (3.0 - 2.0 * a)
    elif preset == "Hybrid":
        r = 0.35 * a + 0.65 * a**3
    elif preset in ("Cubed", "Custom power"):
        r = a ** (3.0 if preset == "Cubed" else max(exponent, 0.05))
    else:
        r = a
    return math.copysign(r, x)


def centering_factor(u: float, curve: float) -> float:
    u = clamp(u, 0.0, 1.0)
    return max(CENTER_FLOOR, 1.0 + curve * (1.0 - 2.0 * u))


def precision_factor(u: float, zone: float, gain: float) -> float:
    if zone <= 0.0:
        return 1.0
    u = abs(u)
    if u <= zone:
        return gain
    if u >= zone + PRECISION_BAND:
        return 1.0
    return gain + (1.0 - gain) * smoothstep((u - zone) / PRECISION_BAND)


# ===========================================================================
# STEERING MATH
# ===========================================================================


@dataclass
class SteerState:
    pos: float = 0.0
    out: float = 0.0
    # Filtered source velocity in source units per second.
    ema_velocity: float = 0.0
    # Legacy/diagnostic timestamp: last raw-position change.
    last_counted_t: float = 0.0
    holding: bool = False
    saturated: bool = False
    # Timestamp of accepted post-filter input, including clean rail input.
    # Kept before new Step 9 state so existing positional construction remains
    # compatible with the earlier state layout.
    last_effective_input_t: float = 0.0
    # Desired post-curve/deadzone/inversion axis before report-rate limiting.
    axis_target: float = 0.0
    # Observable mode-state label: idle, waiting, centering, input, disabled,
    # or manual. It is emitted through runtime telemetry on every tick.
    centering_state: str = "idle"
    # True when raw centering actually ran on this tick, including the final
    # tick that reaches zero and subsequently reports the idle state.
    centering_active: bool = False
    # Step-21 diagnostic snapshots. ``source_velocity`` is the un-gated source
    # delta divided by the control interval; it remains visible when gates
    # reject it. ``filtered_velocity`` is the final post-filter/post-gate value
    # accepted by the steering core. Source units are backend-specific counts
    # or cursor-coordinate units per second, never silently called pixels.
    source_velocity: float = 0.0
    filtered_velocity: float = 0.0
    # Elapsed control time since the last accepted effective input. This is
    # separate from raw-position movement: clean input at a saturated rail can
    # reset it without changing ``pos``.
    input_age_s: float = 0.0

    @property
    def raw_position(self) -> float:
        """Explicit Step 9 name for the legacy ``pos`` raw-position field."""
        return self.pos

    @raw_position.setter
    def raw_position(self, value: float) -> None:
        self.pos = float(value)

    @property
    def axis_output(self) -> float:
        """Actual slew-limited axis value sent by the runtime this tick.

        ``out`` remains as a compatibility alias for existing callers/state
        construction; it now has this explicit sent-axis meaning.
        """
        return self.out

    @axis_output.setter
    def axis_output(self, value: float) -> None:
        self.out = float(value)


def _apply_raw_centering(
    pos: float,
    lock_l: float,
    lock_r: float,
    dt: float,
    s: Settings,
) -> tuple[float, bool]:
    """Apply the retained schema-v1 raw-position centering law once.

    Legacy profiles continue to use ``center_time_s`` in this raw-position
    domain. Native schema-v2 profiles are dispatched to the separate
    axis-target return-time law, so loading an old profile never silently
    changes its centering feel.
    """
    if (
        pos == 0.0
        or s.center_strength <= 0.0
        or s.center_time_s <= 0.0
    ):
        return pos, False
    side = lock_r if pos > 0.0 else lock_l
    u = clamp(abs(pos) / side, 0.0, 1.0)
    rate = (
        s.center_strength
        / s.center_time_s
        * centering_factor(u, s.center_curve)
    )
    step = rate * side * dt
    centered = max(0.0, pos - step) if pos > 0.0 else min(0.0, pos + step)
    return centered, centered != pos


def _axis_target_for_raw_position(
    position: float,
    lock_l: float,
    lock_r: float,
    s: Settings,
) -> float:
    """Return the post-curve/deadzone/inversion target for one raw position."""
    side = lock_r if position >= 0.0 else lock_l
    if side <= 0.0:
        return 0.0
    normalized = clamp(position / side, -1.0, 1.0)
    curve_axis = response_curve(normalized, s.curve_preset, s.curve_exp) * side
    return to_axis(curve_axis, s)


def _apply_axis_target_centering(
    pos: float,
    lock_l: float,
    lock_r: float,
    dt: float,
    s: Settings,
) -> tuple[float, bool]:
    """Center using v2's visible axis-target return-time contract.

    With baseline shape/strength and no output slew, a full current axis target
    loses one normalized-target fraction per ``center_return_time_ms``. The
    inverse response/deadzone mapping is solved monotonically, so a Cubed curve
    no longer turns a visible-time control into a raw-position-time control.
    """
    if (
        pos == 0.0
        or s.center_strength <= 0.0
        or s.center_return_time_ms <= 0.0
    ):
        return pos, False

    raw_sign = 1.0 if pos > 0.0 else -1.0
    side = lock_r if raw_sign > 0.0 else lock_l
    endpoint = abs(
        _axis_target_for_raw_position(raw_sign * side, lock_l, lock_r, s)
    )
    current = abs(_axis_target_for_raw_position(pos, lock_l, lock_r, s))
    if endpoint <= 1e-12 or current <= 1e-12:
        return 0.0, pos != 0.0

    normalized_target = clamp(current / endpoint, 0.0, 1.0)
    rate = (
        s.center_strength
        / (s.center_return_time_ms / 1000.0)
        * centering_factor(normalized_target, s.center_curve)
    )
    desired = max(0.0, normalized_target - rate * dt) * endpoint
    if desired <= 1e-12:
        return 0.0, True

    # Every currently supported response curve and the continuous output
    # deadzone map are monotonic in occupied-side magnitude. Bisection avoids
    # pretending a closed-form inverse exists for S-Curve/Hybrid/deadzone.
    lo = 0.0
    hi = clamp(abs(pos) / side, 0.0, 1.0)
    for _ in range(48):
        mid = (lo + hi) / 2.0
        candidate = abs(
            _axis_target_for_raw_position(raw_sign * mid * side, lock_l, lock_r, s)
        )
        if candidate < desired:
            lo = mid
        else:
            hi = mid
    centered = raw_sign * ((lo + hi) / 2.0) * side
    return centered, centered != pos


def _apply_centering(
    pos: float,
    lock_l: float,
    lock_r: float,
    dt: float,
    s: Settings,
) -> tuple[float, bool]:
    if s.centering_semantics == CENTERING_SEMANTICS_AXIS_TARGET:
        return _apply_axis_target_centering(pos, lock_l, lock_r, dt, s)
    return _apply_raw_centering(pos, lock_l, lock_r, dt, s)


def _limit_input_position_rate(
    requested_rate: float,
    position: float,
    s: Settings,
) -> float:
    """Apply Step-11 raw-position build-up/reversal rate policy.

    ``requested_rate`` is the signed, post-filter/post-gain input rate in
    normalized locks/s before acceleration or any position-rate cap. A move
    toward zero from an occupied side is a reversal. ``max_reversal_rate`` is
    an optional base cap for that case; zero deliberately inherits the normal
    ``max_steering_rate`` cap. A zero selected cap is unlimited.

    Acceleration is intentionally applied to both the requested rate and the
    selected *base* cap. This makes the configured boost observable even when
    input is already cap-limited, while retaining a fully explicit bounded
    effective cap: ``base_cap * (1 + steering_accel * flick)``. The output-axis
    slew limiter is a separate, later stage and is never changed here.
    """
    if requested_rate == 0.0:
        return 0.0

    reversing = position != 0.0 and requested_rate * position < 0.0
    reversal_cap = max(float(getattr(s, "max_reversal_rate", 0.0)), 0.0)
    normal_cap = max(float(s.max_steering_rate), 0.0)
    base_cap = reversal_cap if reversing and reversal_cap > 0.0 else normal_cap

    # Use the pre-acceleration raw-position input rate so gains/precision have
    # already established the driver's requested steering speed. ACCEL_V_REF
    # is expressed in the same normalized-locks/s domain.
    flick = min(abs(requested_rate) / ACCEL_V_REF, 1.0)
    acceleration = max(float(s.steering_accel), 0.0)
    accel_factor = 1.0 + acceleration * flick
    accelerated_rate = requested_rate * accel_factor

    if base_cap <= 0.0:
        return accelerated_rate
    effective_cap = base_cap * accel_factor
    return clamp(accelerated_rate, -effective_cap, effective_cap)


def step_steering(
    st: SteerState,
    dx_px: float,
    dt: float,
    now: float,
    s: Settings,
    *,
    max_axis_step: float | None = None,
) -> SteerState:
    """Advance the pure steering state by one control report.

    The runtime owns scheduler-time attribution. It supplies a bounded report
    interval and, while catching up from a late tick, ``max_axis_step`` so this
    pure output stage combines the safety bound with any configured output
    slew. Leaving the optional bound as ``None`` preserves normal report-rate
    behavior for deterministic/core callers.
    """
    ns = SteerState(
        pos=st.pos,
        out=st.out,
        ema_velocity=st.ema_velocity,
        last_counted_t=st.last_counted_t,
        last_effective_input_t=st.last_effective_input_t,
        axis_target=st.axis_target,
        centering_state=st.centering_state,
        centering_active=st.centering_active,
        source_velocity=st.source_velocity,
        filtered_velocity=st.filtered_velocity,
        input_age_s=st.input_age_s,
    )
    dt = max(float(dt), 1e-6)
    if max_axis_step is not None:
        max_axis_step = max(float(max_axis_step), 0.0)
    mode = resolve_steering_mode(s)
    release_mode = mode == "release_to_center"
    continuous_mode = mode == "continuous_spring"

    lock_l = max(s.lock_left, 0.01)
    lock_r = max(s.lock_right, 0.01)

    # Settings are snapshotted once by the runtime at a control-tick boundary.
    # A newly smaller lock must constrain both state values before *any* input,
    # centering, curve, or slew calculation; otherwise a stale position/output
    # can leak beyond the new rail for one or more reports.
    ns.pos = clamp(ns.pos, -lock_l, lock_r)
    # ``out`` is now the actual sent-axis cache. Inversion swaps the physical
    # left/right output bounds, so clamp in output orientation as well.
    output_lo, output_hi = (
        (-lock_r, lock_l) if s.invert_axis else (-lock_l, lock_r)
    )
    ns.out = clamp(ns.out, output_lo, output_hi)

    # Schema-v1 filter fields are calibrated in px/report at 83 Hz. Convert
    # them once per step to source-velocity units so their physical meaning is
    # stable when reports arrive faster or slower. A zero gate/hysteresis is
    # disabled (zero-valued input still remains no input naturally).
    tau_s, noise_gate_v, hysteresis_v, ema_output_lag_s = _filter_parameters(s)
    # Preserve the pre-gate source velocity for Step-21 diagnostics. The local
    # ``raw_velocity`` below is intentionally allowed to become zero as gates
    # reject input; relabelling that suppressed value as the source would hide
    # exactly the diagnostic distinction a driver needs.
    ns.source_velocity = float(dx_px) / dt
    raw_velocity = ns.source_velocity

    # Raw deadband first.
    if abs(raw_velocity) < noise_gate_v:
        raw_velocity = 0.0

    # ``holding`` retains the legacy input-age meaning for diagnostics and
    # schema-v1 release behavior. Continuous mode deliberately does not let
    # this grace window suppress centering; manual mode never centers.
    holding = (now - ns.last_effective_input_t) * 1000.0 < s.idle_grace_ms
    # Manual retains schema-v1 re-engagement-gate behavior for its retained
    # legacy filter fields, but it never uses idle grace to start centering.
    centering_gate_active = continuous_mode or not holding
    if centering_gate_active and abs(raw_velocity) < hysteresis_v:
        raw_velocity = 0.0
    phys = raw_velocity != 0.0

    # Time-normalized EMA in the velocity domain. A zero smoothing setting
    # explicitly bypasses the filter and clears its memory. At 83 Hz this
    # alpha equals the old schema-v1 per-report smoothing coefficient.
    if tau_s > 0.0:
        prior_filtered_velocity = ns.ema_velocity
        alpha = math.exp(-dt / tau_s)
        ns.ema_velocity = (
            alpha * prior_filtered_velocity + (1.0 - alpha) * raw_velocity
        )
        filtered_velocity = ns.ema_velocity
    else:
        prior_filtered_velocity = 0.0
        alpha = 0.0
        ns.ema_velocity = 0.0
        filtered_velocity = raw_velocity

    # Apply the gates after filtering too, retaining the Step 5 rule that a
    # suppressed output cannot erase EMA memory. Otherwise sustained valid
    # input below hysteresis/(1 - alpha) could never accumulate past it.
    if abs(filtered_velocity) < noise_gate_v:
        filtered_velocity = 0.0
    if centering_gate_active and abs(filtered_velocity) < hysteresis_v:
        filtered_velocity = 0.0
    ns.filtered_velocity = filtered_velocity

    # Integrate the accepted filter over this elapsed interval. The
    # reference-calibrated lag makes this exactly the old filtered delta at
    # 83 Hz, while the exponential state makes equivalent constant-velocity
    # traces agree across other report rates too. A rejected post-filter value
    # must contribute no displacement even though its EMA memory is retained.
    if filtered_velocity == 0.0:
        dx_f = 0.0
    elif tau_s > 0.0:
        dx_f = raw_velocity * dt + (
            (prior_filtered_velocity - raw_velocity)
            * ema_output_lag_s
            * (1.0 - alpha)
        )
    else:
        dx_f = raw_velocity * dt

    # Effective input is post-filter/post-gate input accepted by the core. It
    # refreshes idle grace even when clean saturation prevents a position
    # change at an already-engaged rail.
    effective_input = filtered_velocity != 0.0
    if effective_input:
        ns.last_effective_input_t = now
        holding = True
    ns.input_age_s = max(0.0, now - ns.last_effective_input_t)

    du = 0.0
    if dx_f:
        # Precision-zone location describes the position currently occupied,
        # not the side toward which this report moves. Direction still selects
        # its own sensitivity below.
        occupied_lock_side = lock_r if ns.pos >= 0.0 else lock_l
        u = abs(ns.pos) / occupied_lock_side
        sens = s.sens_right if dx_f > 0.0 else s.sens_left
        dx_eff = (
            dx_f
            * s.raw_scale
            * s.mouse_sensitivity
            * sens
            * precision_factor(u, s.precision_zone, s.precision_gain)
        )
        requested_rate = dx_eff / max(s.full_lock_px, 1.0) / dt
        applied_rate = _limit_input_position_rate(requested_rate, ns.pos, s)
        du = applied_rate * dt

    sat = False
    skip_decay = False
    if du > 0.0 and ns.pos >= lock_r - RAIL_EPS:
        sat = True
    elif du < 0.0 and ns.pos <= -lock_l + RAIL_EPS:
        sat = True

    # ``sat`` preserves the legacy "already pressing at a rail" diagnostic.
    # Continuous spring additionally needs to recognize an accepted outward
    # input that reaches a rail in *this* report; otherwise it would clamp to
    # the rail and immediately pull away before the next held-input report.
    reaches_outward_rail = (
        (du > 0.0 and ns.pos + du >= lock_r - RAIL_EPS)
        or (du < 0.0 and ns.pos + du <= -lock_l + RAIL_EPS)
    )

    if sat and s.saturate_clean:
        du = 0.0
        if phys:
            skip_decay = True
        else:
            ns.ema_velocity = 0.0

    # In continuous mode, only accepted effective input pressing into or
    # reaching a rail opposes centering, even if clean saturation was turned
    # off. Filter-suppressed physical noise therefore cannot keep a held rail
    # from returning forever, while a real held full-lock command still wins.
    if (sat or reaches_outward_rail) and continuous_mode:
        skip_decay = effective_input

    if du:
        ns.pos = clamp(ns.pos + du, -lock_l, lock_r)
        ns.last_counted_t = now
        holding = True

    centering_active = False
    if release_mode:
        # This is deliberately the legacy release-to-centre condition and raw
        # formula: only inactive input after the grace window starts return.
        if not holding and not skip_decay:
            ns.pos, centering_active = _apply_centering(
                ns.pos, lock_l, lock_r, dt, s
            )
        if ns.pos == 0.0:
            ns.centering_state = "idle"
        elif s.center_strength <= 0.0 or s.center_time_s <= 0.0:
            ns.centering_state = "disabled"
        elif centering_active:
            ns.centering_state = "centering"
        elif holding or skip_decay:
            ns.centering_state = "waiting"
        else:
            ns.centering_state = "idle"
    elif continuous_mode:
        # Input integration above and raw centering below happen in the same
        # report. There is no binary idle/grace gate in this mode, so a driver
        # can intentionally balance input against return at a partial angle.
        if not skip_decay:
            ns.pos, centering_active = _apply_centering(
                ns.pos, lock_l, lock_r, dt, s
            )
        if ns.pos == 0.0:
            ns.centering_state = "idle"
        elif centering_active:
            ns.centering_state = "centering"
        elif s.center_strength <= 0.0 or s.center_time_s <= 0.0:
            ns.centering_state = "disabled"
        elif skip_decay or effective_input:
            ns.centering_state = "input"
        else:
            ns.centering_state = "idle"
    else:
        # Manual mode intentionally leaves raw position unchanged in the
        # absence of user input, regardless of all centre controls.
        ns.centering_state = "manual"

    ns.centering_active = centering_active
    ns.holding = holding
    ns.saturated = sat

    side = lock_r if ns.pos >= 0.0 else lock_l
    u = clamp(ns.pos / side, -1.0, 1.0) if side > 0.0 else 0.0
    curve_axis = response_curve(u, s.curve_preset, s.curve_exp) * side

    # Keep the post-curve target distinct from the actual report-limited axis.
    # to_axis() owns post-curve deadzone remapping and inversion; the slew
    # limiter then operates on the desired output axis, as required by the
    # signal-pipeline contract.
    ns.axis_target = clamp(to_axis(curve_axis, s), output_lo, output_hi)

    # The configured slew rate remains the normal feel control. During a late
    # scheduler update, however, an unlimited (zero) configured slew must not
    # turn a bounded integration slice into an unlimited one-report XInput
    # transition. Combine both limits when both apply.
    axis_step_limit: float | None = None
    if s.max_slew_rate > 0.0:
        axis_step_limit = s.max_slew_rate * dt
    if max_axis_step is not None:
        axis_step_limit = (
            max_axis_step
            if axis_step_limit is None
            else min(axis_step_limit, max_axis_step)
        )

    if axis_step_limit is None:
        ns.out = ns.axis_target
    else:
        ns.out = ns.out + clamp(
            ns.axis_target - ns.out,
            -axis_step_limit,
            axis_step_limit,
        )

    ns.out = clamp(ns.out, output_lo, output_hi)
    return ns


def remap_output_deadzone(axis_target: float, deadzone: float) -> float:
    """Apply a continuous post-curve output-axis deadzone.

    Zero is disabled. At a nonzero threshold the remaining magnitude is
    linearly expanded back to the full axis range, so no discontinuous output
    step appears when crossing the threshold.
    """
    deadzone = clamp(float(deadzone), 0.0, 0.99)
    magnitude = abs(float(axis_target))
    if magnitude <= deadzone:
        return 0.0
    remapped = (magnitude - deadzone) / (1.0 - deadzone)
    return math.copysign(remapped, axis_target)


def to_axis(axis_target: float, s: Settings) -> float:
    """Map a post-curve target into its desired, pre-slew output axis."""
    x = remap_output_deadzone(axis_target, s.output_deadzone)
    if s.invert_axis:
        x = -x
    return clamp(x)


def travel_px(s: Settings) -> tuple[float, float]:
    raw = max(s.raw_scale * s.mouse_sensitivity, 1e-9)
    left = (
        max(s.full_lock_px, 1.0)
        * s.lock_left
        / (raw * max(s.sens_left, 1e-9))
    )
    right = (
        max(s.full_lock_px, 1.0)
        * s.lock_right
        / (raw * max(s.sens_right, 1e-9))
    )
    return left, right


def format_status(steer: float, axis: float, paused: bool) -> str:
    width = 11
    cells = ["-"] * (2 * width + 1)
    cells[width] = "|"
    idx = width + int(round(clamp(axis) * width))
    if idx != width:
        cells[max(0, min(idx, 2 * width))] = "#"
    state = "paused" if paused else "active"
    return f"\r[{''.join(cells)}] {axis:+6.1%}  steer={steer:+.3f}  ({state})   "


__all__ = [
    "ACCEL_V_REF",
    "CENTERING_SEMANTICS_AXIS_TARGET",
    "CENTERING_SEMANTICS_RAW_POSITION",
    "CENTER_FLOOR",
    "ControlInterval",
    "CURRENT_PRESET_SCHEMA",
    "CURVE_PRESETS",
    "FILTER_SEMANTICS_LEGACY_V1",
    "FILTER_SEMANTICS_V2",
    "INPUT_MODE_CURSOR_FALLBACK",
    "INPUT_MODE_RAW_INPUT",
    "INPUT_MODES",
    "LEGACY_FILTER_REFERENCE_HZ",
    "MAX_CONTROL_INTEGRATION_S",
    "MAX_LATE_AXIS_STEP",
    "PRESET_PATH",
    "PRESET_SCHEMA_V1",
    "PRESET_SCHEMA_V2",
    "PRESET_STATUS_CORRUPT",
    "PRESET_STATUS_IO_ERROR",
    "PRESET_STATUS_LOADED",
    "PRESET_STATUS_MIGRATED",
    "PRESET_STATUS_MISSING",
    "PRESET_STATUS_UNSUPPORTED_VERSION",
    "PRESET_STATUS_VALIDATION_ERROR",
    "PRECISION_BAND",
    "PROFILE_SEMANTICS_LEGACY_V1",
    "PROFILE_SEMANTICS_V2",
    "PresetLoadResult",
    "PresetSaveResult",
    "RAIL_EPS",
    "STEERING_MODES",
    "Settings",
    "SteerState",
    "centering_factor",
    "clamp",
    "format_preset_load_result",
    "format_status",
    "load_preset",
    "load_preset_result",
    "migrate_preset",
    "new_v2_settings",
    "precision_factor",
    "plan_control_interval",
    "remap_output_deadzone",
    "resolve_steering_mode",
    "response_curve",
    "sanitise_settings",
    "save_preset",
    "smoothstep",
    "step_steering",
    "to_axis",
    "travel_px",
]
