"""Phase 4 / Step 23 -- calibration and the layered profile workflow.

This module answers the question the journal filed as P2-09: *"users do not need
to guess how desktop DPI, mouse polling, and a game's controller filters
combine"*. It does that in four deliberately separate pieces, all of them pure:

1. **A measured trace.** :class:`SourceTrace` records what the input source
   actually delivered, and :func:`analyse_trace` turns it into robust statistics
   (travel, rate percentiles, gaps, quiet-sample ratio) rather than trusting a
   single peak sample.
2. **An operator-driven flow.** :class:`CalibrationSession` is a deterministic
   state machine. It never reads a clock, a device, or a file: the caller feeds
   it timestamps and samples, so the whole flow -- including its refusals -- is
   testable without a display, a mouse, or a game.
3. **Conservative suggestions.** :func:`suggest_settings` proposes new values
   *only* for fields with measured evidence, using robust statistics, clamping
   every proposal into the core's validated domain, and limiting how far one run
   may move a value. A profile is never fabricated from assumptions: without a
   measured trace the function refuses and says exactly what is missing.
4. **A layered profile.** :class:`ProfileBundle` stores the machine-level
   input/safety policy (the ``global`` layer) separately from ``game``, ``car``,
   and ``style`` layers, validates which fields each layer may own, and
   imports/exports with schema validation.

Nothing here imports Tk, Win32, pynput, or vgamepad. The only project import is
the pure steering core, for :class:`~steering_core.Settings`, its validation, and
its ``travel_px`` model -- which is reused rather than re-derived, so a
calibration proposal can never disagree with what the engine will actually do.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any, Callable, Iterable, Mapping, Sequence

try:  # package import
    from . import steering_core as core
except ImportError:  # direct script import (tests add src/ to sys.path)
    import steering_core as core  # type: ignore[no-redef]


# ===========================================================================
# CONSTANTS -- every number a calibration decision depends on
# ===========================================================================
#
# These are intentionally module-level and documented: a driver can read the
# policy the flow follows instead of guessing it from behaviour.

#: Version of the calibration provenance payload stored with a profile.
CALIBRATION_SCHEMA = 1

#: Version of an exported profile bundle.
BUNDLE_SCHEMA = 1

#: The only profile semantics a bundle may describe. A bundle is a measured
#: physical-unit document; importing one onto a legacy (cursor-pixel) profile
#: would silently reinterpret units, so that is refused instead.
BUNDLE_SEMANTICS = core.PROFILE_SEMANTICS_V2

# ---------------------------------------------------------------------------
# Trace acceptance
# ---------------------------------------------------------------------------

#: A step needs at least this many samples before any statistic is trusted.
TRACE_MIN_SAMPLES = 6

#: A step needs at least this long a window for its statistics to be robust.
TRACE_MIN_DURATION_S = 0.30

#: A gap longer than this inside a "steady" step means the source stalled.
MAX_STALL_S = 0.50

# ---------------------------------------------------------------------------
# Step definitions
# ---------------------------------------------------------------------------

SETTLE_MIN_S = 2.0
SETTLE_MAX_S = 12.0
SETTLE_TARGET_S = 3.0

SWEEP_MIN_S = 0.60
SWEEP_MAX_S = 8.0
SWEEP_MIN_SAMPLES = 6
#: A sweep must move at least this far, or it is a twitch, not a sweep.
SWEEP_MIN_TRAVEL_UNITS = 40.0
#: ...and reach at least this rate (robust p95), regardless of the noise floor.
SWEEP_MIN_UNITS_PER_S = 50.0
#: ...and be at least this much louder than the measured noise floor.
MIN_SIGNAL_TO_NOISE = 3.0
#: A sweep should mostly go one way; this bounds how much return travel is fine.
SWEEP_MAX_REVERSE_RATIO = 0.25

LOCK_MIN_S = 0.40
LOCK_MAX_S = 15.0
LOCK_MIN_TRAVEL_UNITS = 100.0
#: If the operator reports the game was *already* at full lock, the true travel
#: is below the measured one; this conservative factor reflects that.
LOCK_OVERSHOOT_FACTOR = 0.80

#: The verification pass may correct the range by at most this much per run.
VERIFY_TRIM_FACTOR = 1.15

# ---------------------------------------------------------------------------
# Suggestion policy
# ---------------------------------------------------------------------------

#: Proposed values are clamped into this input-unit window. The GUI slider for
#: the same field spans 100..2000, so a proposal outside it would be invisible
#: to the operator; the cap is documented rather than silent (it is reported).
FULL_LOCK_MIN_UNITS = 100.0
FULL_LOCK_MAX_UNITS = 2000.0

#: One calibration run may move the measured range by at most this fraction of
#: the current value. A larger measured change is capped and reported: the flow
#: converges over repeat runs instead of jumping on one possibly-misread sweep.
CONSERVATIVE_STEP_LIMIT = 0.50

#: Left/right gain trim is bounded to this symmetric fraction per run.
SYMMETRY_CLAMP = 0.12

#: Differences below this relative size are not worth churning a profile for.
MATERIAL_CHANGE_RATIO = 0.05

#: A noise gate is proposed at this multiple of the measured noise floor...
NOISE_GATE_MULTIPLIER = 2.0
#: ...but only when the measured floor is at least this large, and never above
#: this cap. Below the floor, the honest answer is "no gate is needed".
NOISE_FLOOR_MIN_PROPOSE = 4.0
NOISE_GATE_MAX_PROPOSE = 200.0

#: Fields this module will ever propose. Safety and every unmeasured knob are
#: deliberately absent: a calibration flow must not quietly change a guard, a
#: hotkey, or a value it never measured.
SUGGESTIBLE_FIELDS = (
    "full_lock_px",
    "sens_left",
    "sens_right",
    "noise_gate_units_per_s",
)

# ---------------------------------------------------------------------------
# Step vocabulary
# ---------------------------------------------------------------------------

STEP_SETTLE = "settle"
STEP_SWEEP_RIGHT = "sweep_right"
STEP_SWEEP_LEFT = "sweep_left"
STEP_LOCK = "lock"
STEP_VERIFY = "verify"

PHASE_WAITING = "waiting"
PHASE_MEASURING = "measuring"
PHASE_READY = "ready"
PHASE_STUCK = "stuck"
PHASE_CONFIRM = "confirm"
PHASE_DONE = "done"
PHASE_ABORTED = "aborted"

#: Reported outcomes for the in-game observations. The flow cannot see inside
#: the game, so the operator's report *is* the game-filter evidence.
OBSERVED_FULL = "full"
OBSERVED_PARTIAL = "partial"
OBSERVED_SATURATED = "saturated"
LOCK_OBSERVATIONS = (OBSERVED_FULL, OBSERVED_PARTIAL, OBSERVED_SATURATED)

OBSERVED_ABOUT_RIGHT = "about_right"
OBSERVED_TOO_SENSITIVE = "too_sensitive"
OBSERVED_NOT_ENOUGH = "not_enough"
VERIFY_OBSERVATIONS = (
    OBSERVED_ABOUT_RIGHT,
    OBSERVED_TOO_SENSITIVE,
    OBSERVED_NOT_ENOUGH,
)

# ---------------------------------------------------------------------------
# Profile layers
# ---------------------------------------------------------------------------

LAYER_GLOBAL = "global"
LAYER_GAME = "game"
LAYER_CAR = "car"
LAYER_STYLE = "style"

#: Merge precedence, lowest first. A later layer overrides an earlier one, and
#: no two layers may set the same field (that is reported as a conflict, not
#: silently resolved).
LAYER_KINDS = (LAYER_GLOBAL, LAYER_GAME, LAYER_CAR, LAYER_STYLE)

_SAFETY_FIELDS = (
    "hotkey_pause",
    "hotkey_stop",
    "deadman_enabled",
    "deadman_key",
    "focus_guard_enabled",
    "focus_allowlist",
    "safety_fail_mode",
)

#: Which fields each layer may own. The split is the journal's: the machine-level
#: input/safety policy lives apart from the game, the car, and the driver's style.
#: Fields that are *derived* rather than chosen (``center_time_s``,
#: ``auto_return_enabled``) or that change units (``profile_semantics``,
#: ``filter_semantics``, ``centering_semantics``) are deliberately unowned: no
#: layer may set them, so an import can never reinterpret another profile.
LAYER_FIELD_DOMAINS: dict[str, frozenset[str]] = {
    LAYER_GLOBAL: frozenset(
        (
            "input_mode",
            "full_lock_px",
            "raw_scale",
            "mouse_sensitivity",
            "sens_left",
            "sens_right",
            "update_hz",
            "smoothing_tau_ms",
            "noise_gate_units_per_s",
            "centering_hysteresis_units_per_s",
            # Retained schema-v1-compatible mirrors, kept editable so an
            # existing profile can still be expressed as a global layer.
            "smoothing",
            "noise_gate_px",
            "hysteresis_px",
        )
        + _SAFETY_FIELDS
    ),
    LAYER_GAME: frozenset(
        (
            "curve_preset",
            "curve_exp",
            "output_deadzone",
            "max_slew_rate",
        )
    ),
    LAYER_CAR: frozenset(
        (
            "lock_left",
            "lock_right",
            "max_steering_rate",
            "max_reversal_rate",
            "steering_accel",
            "center_strength",
            "center_return_time_ms",
            "idle_grace_ms",
        )
    ),
    LAYER_STYLE: frozenset(
        (
            "precision_zone",
            "precision_gain",
            "invert_axis",
            "steering_mode",
            "saturate_clean",
            "center_curve",
        )
    ),
}

#: Reverse lookup, built once so a field's owner is unambiguous.
FIELD_OWNER_LAYER: dict[str, str] = {
    field_name: kind
    for kind in LAYER_KINDS
    for field_name in sorted(LAYER_FIELD_DOMAINS[kind])
}


def field_owner_layer(field_name: str) -> str | None:
    """Return the layer kind that owns *field_name*, or ``None`` if unowned."""
    return FIELD_OWNER_LAYER.get(field_name)


# ---------------------------------------------------------------------------
# Status vocabulary (shared with the preset loader so callers see one contract)
# ---------------------------------------------------------------------------

BUNDLE_STATUS_LOADED = core.PRESET_STATUS_LOADED
BUNDLE_STATUS_MISSING = core.PRESET_STATUS_MISSING
BUNDLE_STATUS_CORRUPT = core.PRESET_STATUS_CORRUPT
BUNDLE_STATUS_UNSUPPORTED_VERSION = core.PRESET_STATUS_UNSUPPORTED_VERSION
BUNDLE_STATUS_VALIDATION_ERROR = core.PRESET_STATUS_VALIDATION_ERROR
BUNDLE_STATUS_IO_ERROR = core.PRESET_STATUS_IO_ERROR

#: Default directory for exported bundles, next to the repository's profiles.
PROFILE_DIR = Path(__file__).resolve().parents[1] / "profiles"


# ===========================================================================
# TRACES
# ===========================================================================


def _finite(name: str, value: Any) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a number, not a boolean")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a number; got {value!r}") from exc
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite; got {value!r}")
    return number


@dataclass(frozen=True)
class SourceSample:
    """One input delta, stamped with the control-loop time that drained it."""

    t_s: float
    delta: float

    def __post_init__(self) -> None:
        _finite("t_s", self.t_s)
        _finite("delta", self.delta)


@dataclass(frozen=True)
class OutputSample:
    """One sampled output reading: what the engine actually sent."""

    t_s: float
    pos: float
    axis: float

    def __post_init__(self) -> None:
        _finite("t_s", self.t_s)
        _finite("pos", self.pos)
        _finite("axis", self.axis)


@dataclass(frozen=True)
class SourceTrace:
    """The measured input trace for one step (or one whole session)."""

    backend: str
    nominal_hz: float
    samples: tuple[SourceSample, ...] = ()
    notes: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        if not isinstance(self.backend, str) or not self.backend:
            raise ValueError("a trace needs a non-empty backend name")
        _finite("nominal_hz", self.nominal_hz)
        object.__setattr__(self, "samples", tuple(self.samples))
        object.__setattr__(self, "notes", tuple(self.notes))

    # ----- properties ---------------------------------------------------
    @property
    def count(self) -> int:
        return len(self.samples)

    @property
    def duration_s(self) -> float:
        if len(self.samples) < 2:
            return 0.0
        return self.samples[-1].t_s - self.samples[0].t_s

    @property
    def positive_travel(self) -> float:
        return sum(sample.delta for sample in self.samples if sample.delta > 0.0)

    @property
    def negative_travel(self) -> float:
        return -sum(sample.delta for sample in self.samples if sample.delta < 0.0)

    @property
    def total_travel(self) -> float:
        return sum(abs(sample.delta) for sample in self.samples)

    # ----- derived traces ----------------------------------------------
    def shifted(self, t0: float) -> "SourceTrace":
        """Return the same trace with times rebased so it starts at *t0*."""
        offset = _finite("t0", t0) - (self.samples[0].t_s if self.samples else 0.0)
        return replace(
            self,
            samples=tuple(
                SourceSample(sample.t_s + offset, sample.delta)
                for sample in self.samples
            ),
        )

    def window(self, t_from: float, t_to: float) -> "SourceTrace":
        """Return only the samples inside ``[t_from, t_to]``."""
        lo, hi = _finite("t_from", t_from), _finite("t_to", t_to)
        return replace(
            self,
            samples=tuple(
                sample for sample in self.samples if lo <= sample.t_s <= hi
            ),
        )

    def rates(self) -> tuple[float, ...]:
        """Return the measured signed rate between consecutive samples.

        Only intervals that were actually measured are returned. The first
        sample has no predecessor, so inventing a rate for it would let one
        synthetic value inflate a percentile -- exactly the kind of single-sample
        trust this module exists to avoid. A one-sample trace is the sole
        exception: there the nominal period is the only evidence there is.
        """
        if not self.samples:
            return ()
        rates: list[float] = []
        for previous, current in zip(self.samples, self.samples[1:]):
            dt = current.t_s - previous.t_s
            if dt > 1e-9:
                rates.append(current.delta / dt)
        if not rates:
            fallback_dt = 1.0 / self.nominal_hz if self.nominal_hz > 0.0 else 0.0
            if fallback_dt > 0.0:
                rates.append(self.samples[0].delta / fallback_dt)
        return tuple(rates)


@dataclass(frozen=True)
class OutputTrace:
    """The engine's own sent-output trace, sampled while a step ran.

    This is the "actual output trace" the journal asks for: it proves what the
    program sent, independently of what the game then did with it.
    """

    samples: tuple[OutputSample, ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(self, "samples", tuple(self.samples))

    @property
    def count(self) -> int:
        return len(self.samples)

    @property
    def peak_axis(self) -> float:
        return max((abs(sample.axis) for sample in self.samples), default=0.0)

    @property
    def peak_pos(self) -> float:
        return max((abs(sample.pos) for sample in self.samples), default=0.0)

    def time_to_axis(self, threshold: float) -> float | None:
        """Seconds from the first sample until the sent axis first reaches a
        threshold, or ``None`` if it never did."""
        if not self.samples:
            return None
        t0 = self.samples[0].t_s
        for sample in self.samples:
            if abs(sample.axis) >= threshold:
                return max(sample.t_s - t0, 0.0)
        return None


def _percentile(sorted_values: Sequence[float], q: float) -> float:
    """Linear-interpolated percentile of an already-sorted sequence."""
    if not sorted_values:
        return 0.0
    if len(sorted_values) == 1:
        return float(sorted_values[0])
    position = min(max(q, 0.0), 1.0) * (len(sorted_values) - 1)
    lower = int(math.floor(position))
    upper = min(lower + 1, len(sorted_values) - 1)
    weight = position - lower
    return float(sorted_values[lower]) * (1.0 - weight) + float(
        sorted_values[upper]
    ) * weight


@dataclass(frozen=True)
class TraceAnalysis:
    """Robust statistics for one measured trace.

    Every number the flow later reasons about lives here, so a failure message
    can always point at the measurement that caused it.
    """

    samples: int
    duration_s: float
    observed_hz: float
    total_travel_units: float
    positive_travel_units: float
    negative_travel_units: float
    positive_peak_units_per_s: float
    negative_peak_units_per_s: float
    rate_p50_units_per_s: float
    rate_p95_units_per_s: float
    rate_max_units_per_s: float
    stall_count: int
    longest_gap_s: float
    quiet_sample_ratio: float
    warnings: tuple[str, ...] = ()

    @property
    def usable(self) -> bool:
        """Whether this trace carries enough evidence to reason about."""
        return self.samples >= TRACE_MIN_SAMPLES and self.duration_s >= (
            TRACE_MIN_DURATION_S
        )

    def travel_for(self, direction: str) -> float:
        """Travel in one direction: ``"right"`` or ``"left"``."""
        if direction == "right":
            return self.positive_travel_units
        if direction == "left":
            return self.negative_travel_units
        raise ValueError(f"unknown direction {direction!r}")

    def reverse_travel_for(self, direction: str) -> float:
        """Travel *against* the named direction, used to spot a back-and-forth."""
        return self.travel_for("left" if direction == "right" else "right")

    @property
    def direction(self) -> str:
        """``"right"``, ``"left"``, ``"none"`` or ``"both"``."""
        positive = self.positive_travel_units
        negative = self.negative_travel_units
        scale = max(positive, negative, 1e-9)
        if positive <= 1e-9 and negative <= 1e-9:
            return "none"
        if positive / scale >= 0.25 and negative / scale >= 0.25:
            return "both"
        return "right" if positive >= negative else "left"


def analyse_trace(trace: SourceTrace) -> TraceAnalysis:
    """Measure one trace without trusting any single sample."""
    samples = trace.samples
    rates = trace.rates()
    unsigned = sorted(abs(rate) for rate in rates)
    positive_peak = max((rate for rate in rates if rate > 0.0), default=0.0)
    negative_peak = -min((rate for rate in rates if rate < 0.0), default=0.0)
    p50 = _percentile(unsigned, 0.50)
    p95 = _percentile(unsigned, 0.95)

    gaps: list[float] = []
    for previous, current in zip(samples, samples[1:]):
        gaps.append(max(current.t_s - previous.t_s, 0.0))
    longest_gap = max(gaps, default=0.0)
    stall_count = sum(1 for gap in gaps if gap > MAX_STALL_S)
    quiet_threshold = max(p50 * 2.0, 1e-9)
    quiet_ratio = (
        sum(1 for rate in unsigned if rate <= quiet_threshold) / len(unsigned)
        if unsigned
        else 1.0
    )
    observed_hz = (
        (len(samples) - 1) / trace.duration_s if trace.duration_s > 1e-9 else 0.0
    )

    warnings: list[str] = []
    if len(samples) < TRACE_MIN_SAMPLES:
        warnings.append(
            f"only {len(samples)} sample(s); at least {TRACE_MIN_SAMPLES} are "
            "needed for a trustworthy measurement"
        )
    if trace.duration_s < TRACE_MIN_DURATION_S:
        warnings.append(
            f"the step lasted {trace.duration_s:.2f} s; at least "
            f"{TRACE_MIN_DURATION_S:.2f} s is needed"
        )
    if stall_count:
        warnings.append(
            f"{stall_count} gap(s) longer than {MAX_STALL_S:.2f} s suggest the "
            "source stopped delivering input"
        )
    if observed_hz and trace.nominal_hz and observed_hz < 0.5 * trace.nominal_hz:
        warnings.append(
            f"the source delivered {observed_hz:.1f} Hz against a configured "
            f"{trace.nominal_hz:.1f} Hz; polling or capture is slower than the "
            "profile assumes"
        )

    return TraceAnalysis(
        samples=len(samples),
        duration_s=trace.duration_s,
        observed_hz=observed_hz,
        total_travel_units=trace.total_travel,
        positive_travel_units=trace.positive_travel,
        negative_travel_units=trace.negative_travel,
        positive_peak_units_per_s=positive_peak,
        negative_peak_units_per_s=negative_peak,
        rate_p50_units_per_s=p50,
        rate_p95_units_per_s=p95,
        rate_max_units_per_s=max(unsigned, default=0.0),
        stall_count=stall_count,
        longest_gap_s=longest_gap,
        quiet_sample_ratio=quiet_ratio,
        warnings=tuple(warnings),
    )


# ===========================================================================
# THE FLOW
# ===========================================================================


@dataclass(frozen=True)
class CalibrationStep:
    """One operator-facing step of the flow."""

    name: str
    title: str
    prompt: str
    guidance: str
    min_s: float
    max_s: float
    kind: str
    optional: bool = False

    @property
    def needs_observation(self) -> bool:
        """Whether the step finishes on an operator report, not on samples."""
        return self.kind in (STEP_LOCK, STEP_VERIFY)


DEFAULT_STEPS: tuple[CalibrationStep, ...] = (
    CalibrationStep(
        name=STEP_SETTLE,
        title="Measure the noise floor",
        prompt="Take your hand off the mouse and leave it still.",
        guidance=(
            "This measures how much the sensor drifts on its own, which is what "
            "an input noise gate should sit above. Do not move the mouse."
        ),
        min_s=SETTLE_MIN_S,
        max_s=SETTLE_MAX_S,
        kind=STEP_SETTLE,
    ),
    CalibrationStep(
        name=STEP_SWEEP_RIGHT,
        title="Sweep right",
        prompt="Sweep the mouse steadily to the right and keep going.",
        guidance=(
            "Use one continuous movement at the speed you actually drive with. "
            "Do not stop and start; a steady sweep is what gets measured."
        ),
        min_s=SWEEP_MIN_S,
        max_s=SWEEP_MAX_S,
        kind=STEP_SWEEP_RIGHT,
    ),
    CalibrationStep(
        name=STEP_SWEEP_LEFT,
        title="Sweep left",
        prompt="Now sweep steadily to the left, the same way.",
        guidance=(
            "Match the effort you used on the right. This is the measurement "
            "that reveals left/right asymmetry."
        ),
        min_s=SWEEP_MIN_S,
        max_s=SWEEP_MAX_S,
        kind=STEP_SWEEP_LEFT,
    ),
    CalibrationStep(
        name=STEP_LOCK,
        title="Measure your full-lock travel",
        prompt=(
            "Sweep right in one movement until the car reaches full lock, then "
            "stop there. Watch the in-game steering, not the mouse."
        ),
        guidance=(
            "The travel you use is the measurement. Then report what the game "
            "did: full lock, still short of it, or already past it."
        ),
        min_s=LOCK_MIN_S,
        max_s=LOCK_MAX_S,
        kind=STEP_LOCK,
    ),
    CalibrationStep(
        name=STEP_VERIFY,
        title="Verify in the game",
        prompt=(
            "Drive normally for a moment, then report how the steering felt."
        ),
        guidance=(
            "This is the only step that can see the game's own filtering, "
            "because you are looking at it. A correction is bounded and "
            "converges over repeat runs."
        ),
        min_s=0.0,
        max_s=0.0,
        kind=STEP_VERIFY,
        optional=True,
    ),
)


@dataclass(frozen=True)
class StepEvidence:
    """What one finished step measured, plus the words that explain it."""

    step: str
    kind: str
    analysis: TraceAnalysis | None
    measured: Mapping[str, float] = field(default_factory=dict)
    observation: str | None = None
    summary: str = ""

    def as_dict(self) -> dict[str, Any]:
        return {
            "step": self.step,
            "kind": self.kind,
            "observation": self.observation,
            "measured": dict(self.measured),
            "summary": self.summary,
        }


@dataclass(frozen=True)
class StepOutcome:
    """The state of the flow after one ``feed``/``observe`` call."""

    step: str | None
    title: str
    phase: str
    elapsed_s: float
    progress: float
    prompt: str
    guidance: str
    failure: str | None = None
    measured_so_far: Mapping[str, float] = field(default_factory=dict)
    step_index: int = 0
    step_count: int = 0
    complete: bool = False
    aborted: bool = False

    @property
    def needs_observation(self) -> bool:
        return self.phase == PHASE_CONFIRM

    @property
    def blocked(self) -> bool:
        return self.phase == PHASE_STUCK or self.failure is not None

    def status_line(self) -> str:
        """One line suitable for a console or a status bar."""
        if self.aborted:
            return f"Calibration aborted: {self.failure or 'stopped by the operator'}"
        if self.complete:
            return "Calibration complete."
        if self.failure:
            return (
                f"[{self.step_index + 1}/{self.step_count}] {self.title}: "
                f"{self.failure}"
            )
        if self.needs_observation:
            return (
                f"[{self.step_index + 1}/{self.step_count}] {self.title}: "
                f"{self.prompt}"
            )
        return (
            f"[{self.step_index + 1}/{self.step_count}] {self.title}: "
            f"{self.progress:.0%} — {self.prompt}"
        )


class CalibrationSession:
    """A deterministic, operator-driven calibration flow.

    The session owns no clock and no device. The caller supplies ``now_s`` (a
    monotonic time) and the samples that arrived since the previous call, which
    is what makes every branch -- including every refusal -- reproducible in a
    test. It also means the GUI, the CLI, and a test all drive exactly the same
    state machine.
    """

    def __init__(
        self,
        settings: core.Settings,
        *,
        steps: Sequence[CalibrationStep] = DEFAULT_STEPS,
        backend: str = core.INPUT_MODE_RAW_INPUT,
        nominal_hz: float | None = None,
    ) -> None:
        if not steps:
            raise ValueError("a calibration flow needs at least one step")
        names = [step.name for step in steps]
        if len(set(names)) != len(names):
            raise ValueError("calibration step names must be unique")
        self._steps = tuple(steps)
        self._settings = core.sanitise_settings(settings)
        self._backend = str(backend)
        self._nominal_hz = float(
            nominal_hz if nominal_hz is not None else self._settings.update_hz
        )
        self._index = 0
        self._samples: list[SourceSample] = []
        self._step_started_s: float | None = None
        self._last_now_s: float | None = None
        self._last_failure: str | None = None
        self._restart_note: str | None = None
        self._stuck = False
        self._evidence: list[StepEvidence] = []
        self._observations: dict[str, str] = {}
        self._aborted_reason: str | None = None
        self._started_at_s: float | None = None
        self._finished_at_s: float | None = None

    # ----- introspection -------------------------------------------------
    @property
    def steps(self) -> tuple[CalibrationStep, ...]:
        return self._steps

    @property
    def current_step(self) -> CalibrationStep | None:
        if self._finished_at_s is not None or self._aborted_reason is not None:
            return None
        return self._steps[self._index]

    @property
    def complete(self) -> bool:
        return self._finished_at_s is not None and self._aborted_reason is None

    @property
    def aborted(self) -> bool:
        return self._aborted_reason is not None

    @property
    def evidence(self) -> tuple[StepEvidence, ...]:
        return tuple(self._evidence)

    @property
    def observations(self) -> Mapping[str, str]:
        return dict(self._observations)

    @property
    def elapsed_s(self) -> float:
        if self._started_at_s is None or self._last_now_s is None:
            return 0.0
        return max(self._last_now_s - self._started_at_s, 0.0)

    def trace_for_step(self, name: str) -> SourceTrace:
        """Return the raw trace a finished step measured.

        Re-analysis of the exact samples a decision was made from is the point:
        a report can always be re-derived, and a disputed number can be checked.
        """
        for record in self._evidence:
            if record.step == name:
                samples = record.measured.get("_samples")
                if isinstance(samples, tuple):
                    return SourceTrace(
                        backend=self._backend,
                        nominal_hz=self._nominal_hz,
                        samples=samples,
                    )
        return SourceTrace(
            backend=self._backend, nominal_hz=self._nominal_hz, samples=()
        )

    # ----- driving -------------------------------------------------------
    def feed(
        self,
        now_s: float,
        samples: Iterable[SourceSample] = (),
    ) -> StepOutcome:
        """Advance the flow with the samples drained since the last call."""
        now = _finite("now_s", now_s)
        if self._started_at_s is None:
            self._started_at_s = now
            self._step_started_s = now
        if self._last_now_s is not None and now < self._last_now_s:
            raise ValueError("calibration time must not move backwards")
        self._last_now_s = now
        if self._finished_at_s is not None or self._aborted_reason is not None:
            return self._outcome()

        step = self._steps[self._index]
        incoming = tuple(samples)
        if incoming and step.needs_observation:
            # The lock step *is* measured from samples before it asks; the
            # verification step only records the operator's report, so feeding
            # samples into it would be a caller bug rather than useful data.
            if step.kind == STEP_VERIFY:
                raise ValueError(
                    "the verification step takes an observation, not samples"
                )
        self._samples.extend(incoming)

        if step.needs_observation:
            if step.kind == STEP_LOCK and incoming:
                self._last_failure = None
                self._stuck = False
            return self._outcome(stuck=False)

        analysis = self._analyse_current()
        elapsed = self._step_elapsed(now)
        failure = self._check_step(step, analysis, elapsed)
        if failure is None and elapsed >= step.min_s:
            self._finish_sampled_step(step, analysis)
            return self._outcome()
        if failure is not None and elapsed >= step.min_s:
            self._last_failure = failure
        if elapsed >= step.max_s:
            # The step timed out with no usable signal: forget the samples so a
            # retry starts from a clean window. The reason the step could not be
            # completed is kept, and the restart is stated explicitly, because a
            # silent restart would look to the operator like the flow had hung.
            self._restart_note = (
                f"the step was restarted after {step.max_s:.0f} s; try again"
            )
            self._samples.clear()
            self._step_started_s = now
            self._stuck = True
            self._last_failure = failure or "no usable measurement yet"
        return self._outcome()

    def observe(self, observation: str) -> StepOutcome:
        """Record the operator's report for the current observation step."""
        if self._finished_at_s is not None or self._aborted_reason is not None:
            return self._outcome()
        step = self._steps[self._index]
        if not step.needs_observation:
            raise ValueError(
                f"step {step.name!r} is measured from input, not observed"
            )
        value = str(observation).strip().lower()
        allowed = (
            LOCK_OBSERVATIONS if step.kind == STEP_LOCK else VERIFY_OBSERVATIONS
        )
        if value not in allowed:
            raise ValueError(
                f"{value!r} is not a valid report for {step.name!r}; expected "
                + ", ".join(allowed)
            )

        if step.kind == STEP_LOCK:
            analysis = self._analyse_current()
            travel = self._lock_measurement(analysis)
            if travel is None:
                self._last_failure = (
                    "the sweep was too short to measure; sweep further before "
                    "reporting what the game did"
                )
                return self._outcome()
            if value == OBSERVED_PARTIAL:
                # Full lock was not reached: the operator's own sweep defines a
                # lower bound, so the honest answer is to measure a longer one.
                self._samples.clear()
                self._step_started_s = self._last_now_s
                self._last_failure = (
                    f"full lock was not reached at {travel:.0f} units. Sweep "
                    "further (or reduce the in-game steering range) and report "
                    "again — nothing has been written."
                )
                return self._outcome()
            evidence_travel = (
                travel * LOCK_OVERSHOOT_FACTOR
                if value == OBSERVED_SATURATED
                else travel
            )
            summary = (
                f"full lock at {evidence_travel:.0f} input units"
                + (
                    " (conservatively reduced, because the game was already at "
                    "full lock before that point)"
                    if value == OBSERVED_SATURATED
                    else ""
                )
                + f"; the game reported {value}"
            )
            self._record(
                step,
                analysis,
                {"full_lock_units": evidence_travel, "_raw_travel_units": travel},
                observation=value,
                summary=summary,
            )
        else:
            self._record(
                step,
                None,
                {},
                observation=value,
                summary=f"the operator reported the steering felt {value.replace('_', ' ')}",
            )
        return self._advance()

    def retry(self) -> StepOutcome:
        """Discard the current step's samples and start it again."""
        if self._finished_at_s is not None or self._aborted_reason is not None:
            return self._outcome()
        self._samples.clear()
        self._last_failure = None
        self._restart_note = None
        self._stuck = False
        self._step_started_s = self._last_now_s
        return self._outcome()

    def skip(self) -> StepOutcome:
        """Skip the current step if the flow allows it."""
        if self._finished_at_s is not None or self._aborted_reason is not None:
            return self._outcome()
        step = self._steps[self._index]
        if not step.optional:
            self._last_failure = (
                f"{step.title} cannot be skipped: it is the measurement that "
                "makes the resulting profile evidence-based rather than guessed"
            )
            return self._outcome()
        self._record(
            step,
            None,
            {},
            observation=None,
            summary="skipped by the operator",
        )
        return self._advance()

    def abort(self, reason: str = "stopped by the operator") -> StepOutcome:
        """Abandon the flow. Nothing measured is applied by itself."""
        if self._finished_at_s is not None:
            return self._outcome()
        self._aborted_reason = str(reason)
        self._last_now_s = self._last_now_s or 0.0
        return self._outcome()

    # ----- results -------------------------------------------------------
    def report(self) -> "CalibrationReport":
        """Return the measured report (complete or not)."""
        analyses = {
            record.step: record.analysis
            for record in self._evidence
            if record.analysis is not None
        }
        return CalibrationReport(
            complete=self.complete,
            aborted_reason=self._aborted_reason,
            backend=self._backend,
            nominal_hz=self._nominal_hz,
            settings=self._settings,
            steps=tuple(self._evidence),
            analyses=analyses,
            observations=dict(self._observations),
            elapsed_s=self.elapsed_s,
        )

    # ----- internals -----------------------------------------------------
    def _trace(self) -> SourceTrace:
        return SourceTrace(
            backend=self._backend,
            nominal_hz=self._nominal_hz,
            samples=tuple(self._samples),
        )

    def _analyse_current(self) -> TraceAnalysis:
        return analyse_trace(self._trace())

    def _step_elapsed(self, now_s: float) -> float:
        if self._step_started_s is None:
            return 0.0
        return max(now_s - self._step_started_s, 0.0)

    def _noise_floor(self) -> float:
        for record in self._evidence:
            if record.kind == STEP_SETTLE and record.analysis is not None:
                return float(
                    record.measured.get(
                        "noise_floor_units_per_s",
                        record.analysis.rate_p95_units_per_s,
                    )
                )
        return 0.0

    def _lock_measurement(self, analysis: TraceAnalysis) -> float | None:
        """The travel this lock step has measured so far, when it is usable."""
        if not analysis.usable or analysis.direction != "right":
            return None
        if analysis.positive_travel_units < LOCK_MIN_TRAVEL_UNITS:
            return None
        return analysis.positive_travel_units

    def _check_step(
        self, step: CalibrationStep, analysis: TraceAnalysis, elapsed: float
    ) -> str | None:
        """Return an actionable failure message, or ``None`` when acceptable."""
        if not analysis.usable:
            return None  # still measuring; the timeout owns "not enough"
        if step.kind == STEP_SETTLE:
            if analysis.rate_p95_units_per_s > 4.0 * max(
                analysis.rate_p50_units_per_s, 1.0
            ) and analysis.rate_max_units_per_s > 200.0:
                return (
                    "the mouse is still moving. Let go of it completely and "
                    "leave it still for a few seconds."
                )
            return None
        if step.kind in (STEP_SWEEP_RIGHT, STEP_SWEEP_LEFT):
            expected = "right" if step.kind == STEP_SWEEP_RIGHT else "left"
            if analysis.direction == "both":
                return (
                    "the movement went in both directions. Sweep one way in one "
                    "continuous movement; a back-and-forth is not a sweep."
                )
            if analysis.direction not in (expected, "none"):
                return (
                    f"that sweep went {analysis.direction}. Sweep {expected} for "
                    "this step."
                )
            travel = analysis.travel_for(expected)
            if travel < SWEEP_MIN_TRAVEL_UNITS:
                return (
                    f"only {travel:.0f} units of travel were seen; sweep at "
                    f"least {SWEEP_MIN_TRAVEL_UNITS:.0f}."
                )
            signal_floor = max(
                self._noise_floor() * MIN_SIGNAL_TO_NOISE,
                SWEEP_MIN_UNITS_PER_S,
            )
            if analysis.rate_p95_units_per_s < signal_floor:
                return (
                    f"the sweep was too slow to measure: {analysis.rate_p95_units_per_s:.0f} "
                    f"units/s against a required {signal_floor:.0f} units/s. Sweep "
                    "faster, at the speed you actually drive with."
                )
            if (
                analysis.reverse_travel_for(expected)
                > SWEEP_MAX_REVERSE_RATIO * travel
            ):
                return (
                    "that sweep contained too much return movement. Keep going "
                    "one way for the whole step."
                )
            return None
        if step.kind == STEP_LOCK:
            if analysis.direction != "right":
                return "sweep right for this step, then report what the game did."
            if analysis.positive_travel_units < LOCK_MIN_TRAVEL_UNITS:
                return (
                    f"only {analysis.positive_travel_units:.0f} units of travel "
                    f"were seen; sweep at least {LOCK_MIN_TRAVEL_UNITS:.0f} units "
                    "before reaching full lock."
                )
            return None
        return None

    def _finish_sampled_step(
        self, step: CalibrationStep, analysis: TraceAnalysis
    ) -> None:
        if step.kind == STEP_SETTLE:
            noise = analysis.rate_p95_units_per_s
            self._record(
                step,
                analysis,
                {"noise_floor_units_per_s": noise},
                observation=None,
                summary=(
                    f"measured noise floor {noise:.1f} units/s with "
                    f"{analysis.observed_hz:.0f} Hz observed sampling"
                ),
            )
        elif step.kind in (STEP_SWEEP_RIGHT, STEP_SWEEP_LEFT):
            direction = "right" if step.kind == STEP_SWEEP_RIGHT else "left"
            travel = analysis.travel_for(direction)
            self._record(
                step,
                analysis,
                {
                    "travel_units": travel,
                    "direction": direction,
                    "rate_units_per_s": analysis.rate_p95_units_per_s,
                },
                observation=None,
                summary=(
                    f"{direction} sweep of {travel:.0f} units at "
                    f"{analysis.rate_p95_units_per_s:.0f} units/s"
                ),
            )
        else:  # pragma: no cover - every sampled kind is handled above
            self._record(step, analysis, {}, observation=None, summary="measured")
        self._advance()

    def _record(
        self,
        step: CalibrationStep,
        analysis: TraceAnalysis | None,
        measured: Mapping[str, float],
        *,
        observation: str | None,
        summary: str,
    ) -> None:
        payload: dict[str, Any] = dict(measured)
        # Keep the exact samples so the report can be re-derived (and audited)
        # after the fact. They are never serialised: a raw trace is a keystroke
        # log of the operator's hand, which does not belong in a profile file.
        payload["_samples"] = tuple(self._samples)
        record = StepEvidence(
            step=step.name,
            kind=step.kind,
            analysis=analysis,
            measured=payload,
            observation=observation,
            summary=summary,
        )
        self._evidence = [item for item in self._evidence if item.step != step.name]
        self._evidence.append(record)
        if observation is not None:
            self._observations[step.name] = observation

    def _advance(self) -> StepOutcome:
        self._samples.clear()
        self._last_failure = None
        self._restart_note = None
        self._stuck = False
        self._index += 1
        self._step_started_s = self._last_now_s
        if self._index >= len(self._steps):
            self._index = len(self._steps) - 1
            self._finished_at_s = self._last_now_s
        return self._outcome()

    def _outcome(self, *, stuck: bool | None = None) -> StepOutcome:
        stuck_now = self._stuck if stuck is None else stuck
        if self._finished_at_s is not None:
            return StepOutcome(
                step=None,
                title="Done",
                phase=PHASE_DONE,
                elapsed_s=self.elapsed_s,
                progress=1.0,
                prompt="Calibration complete.",
                guidance="",
                complete=True,
                step_index=len(self._steps) - 1,
                step_count=len(self._steps),
            )
        step = self._steps[self._index]
        if self._aborted_reason is not None:
            return StepOutcome(
                step=step.name,
                title=step.title,
                phase=PHASE_ABORTED,
                elapsed_s=self.elapsed_s,
                progress=0.0,
                prompt=step.prompt,
                guidance=step.guidance,
                failure=self._aborted_reason,
                aborted=True,
                step_index=self._index,
                step_count=len(self._steps),
            )
        elapsed = self._step_elapsed(self._last_now_s or 0.0)
        progress = (
            1.0 if step.min_s <= 0.0 else min(elapsed / step.min_s, 1.0)
        )
        if step.needs_observation:
            analysis = self._analyse_current()
            ready = self._check_step(step, analysis, elapsed) is None and (
                step.kind != STEP_LOCK or self._lock_measurement(analysis) is not None
            )
            return StepOutcome(
                step=step.name,
                title=step.title,
                phase=PHASE_CONFIRM if ready else PHASE_MEASURING,
                elapsed_s=self.elapsed_s,
                progress=1.0 if ready else 0.0,
                prompt=step.prompt,
                guidance=step.guidance,
                failure=self._failure_text(),
                measured_so_far=self._measured_so_far(),
                step_index=self._index,
                step_count=len(self._steps),
            )
        phase = (
            PHASE_STUCK
            if stuck_now
            else (
                PHASE_WAITING
                if elapsed < step.min_s
                else PHASE_READY
                if self._last_failure is None
                else PHASE_MEASURING
            )
        )
        return StepOutcome(
            step=step.name,
            title=step.title,
            phase=phase,
            elapsed_s=self.elapsed_s,
            progress=progress,
            prompt=step.prompt,
            guidance=step.guidance,
            failure=self._failure_text(),
            measured_so_far=self._measured_so_far(),
            step_index=self._index,
            step_count=len(self._steps),
        )

    def _failure_text(self) -> str | None:
        """The current failure, with a sticky note when the step was restarted.

        A restart is a state change the operator must notice, so it stays in the
        message until they retry, skip, or advance: otherwise the flow would look
        as though it had silently gone back to waiting.
        """
        if self._restart_note is None:
            return self._last_failure
        if self._last_failure is None:
            return self._restart_note
        return f"{self._last_failure} ({self._restart_note})"

    def _measured_so_far(self) -> dict[str, float]:
        analysis = self._analyse_current()
        measured = {
            "samples": float(analysis.samples),
            "observed_hz": analysis.observed_hz,
            "positive_travel_units": analysis.positive_travel_units,
            "negative_travel_units": analysis.negative_travel_units,
            "rate_p95_units_per_s": analysis.rate_p95_units_per_s,
        }
        return measured


# ===========================================================================
# REPORT
# ===========================================================================


@dataclass(frozen=True)
class CalibrationReport:
    """Everything one calibration run measured, in one auditable object."""

    complete: bool
    aborted_reason: str | None
    backend: str
    nominal_hz: float
    settings: core.Settings
    steps: tuple[StepEvidence, ...]
    analyses: Mapping[str, TraceAnalysis]
    observations: Mapping[str, str]
    elapsed_s: float
    output_trace: OutputTrace | None = None
    output_axis_peak: float = 0.0
    notes: tuple[str, ...] = ()

    # ----- lookups -------------------------------------------------------
    def step(self, name: str) -> StepEvidence | None:
        for record in self.steps:
            if record.step == name:
                return record
        return None

    def measured(self, name: str, key: str, default: float = 0.0) -> float:
        record = self.step(name)
        if record is None:
            return default
        value = record.measured.get(key, default)
        return float(value) if isinstance(value, (int, float)) else default

    def with_output_trace(self, trace: OutputTrace) -> "CalibrationReport":
        return replace(
            self,
            output_trace=trace,
            output_axis_peak=trace.peak_axis,
        )

    # ----- human-facing --------------------------------------------------
    def pending_step_name(self) -> str | None:
        """Name of the first required step with no evidence, if any."""
        for name in (STEP_SETTLE, STEP_SWEEP_RIGHT, STEP_SWEEP_LEFT, STEP_LOCK):
            if self.step(name) is None:
                return name
        if self.step(STEP_VERIFY) is None:
            return STEP_VERIFY
        return None

    @property
    def ready_for_suggestions(self) -> bool:
        return self.complete and self.blocking_reason() is None

    def blocking_reason(self) -> str | None:
        """Why this report cannot produce a profile, or ``None``."""
        if self.aborted_reason:
            return f"the run was aborted: {self.aborted_reason}"
        if not self.complete:
            pending = self.pending_step_name()
            if pending is None:
                return "the run has not finished"
            return (
                f"the run has not finished: the {pending!r} step is still open "
                "(answer it, or skip it if the flow allows)"
            )
        missing = [
            name
            for name in (STEP_SETTLE, STEP_SWEEP_RIGHT, STEP_SWEEP_LEFT, STEP_LOCK)
            if self.step(name) is None
        ]
        if missing:
            return "these steps were not measured: " + ", ".join(missing)
        if self.settings.profile_semantics != core.PROFILE_SEMANTICS_V2:
            return (
                "the profile uses legacy schema-v1 semantics, so its units are "
                "cursor pixels rather than source units. Migrate it to schema v2 "
                "first: calibrating it here would reinterpret its numbers."
            )
        return None

    def summary_lines(self) -> tuple[str, ...]:
        """The measured facts, as short lines a driver can read."""
        lines: list[str] = []
        for record in self.steps:
            lines.append(f"{record.step}: {record.summary}")
        if self.output_trace is not None and self.output_trace.count:
            peak = self.output_trace.peak_axis
            reached = self.output_trace.time_to_axis(0.99)
            detail = f"sent axis peaked at {peak:+.2f}"
            if reached is not None:
                detail += f" (reached full lock {reached * 1000.0:.0f} ms into the step)"
            lines.append(f"output trace: {detail}")
        for record in self.steps:
            if record.analysis is not None:
                lines.extend(
                    f"{record.step}: {warning}"
                    for warning in record.analysis.warnings
                )
        return tuple(lines)

    def measured_facts(self) -> dict[str, float]:
        """The flat, serialisable facts a profile keeps as provenance."""
        settle = self.analyses.get(STEP_SETTLE)
        facts: dict[str, float] = {
            "observed_hz": settle.observed_hz if settle is not None else 0.0,
            "duration_s": self.elapsed_s,
        }
        for name, key in (
            (STEP_SETTLE, "noise_floor_units_per_s"),
            (STEP_SWEEP_RIGHT, "travel_units"),
            (STEP_SWEEP_LEFT, "travel_units"),
            (STEP_SWEEP_RIGHT, "rate_units_per_s"),
            (STEP_SWEEP_LEFT, "rate_units_per_s"),
            (STEP_LOCK, "full_lock_units"),
        ):
            facts[f"{name}.{key}"] = self.measured(name, key)
        if self.output_trace is not None and self.output_trace.count:
            facts["output_axis_peak"] = self.output_trace.peak_axis
        return facts

    def suggested_full_lock_units(self) -> float:
        return self.measured(STEP_LOCK, "full_lock_units")


# ===========================================================================
# SUGGESTIONS
# ===========================================================================


@dataclass(frozen=True)
class FieldSuggestion:
    """One proposed change, with the evidence and the ambiguity named."""

    field: str
    layer: str
    current: float
    proposed: float
    basis: str
    confidence: str
    clamped: bool = False

    @property
    def changed(self) -> bool:
        return abs(self.proposed - self.current) > 1e-9

    def describe(self) -> str:
        direction = "increase" if self.proposed > self.current else "reduce"
        text = (
            f"{self.field}: {self.current:.3g} → {self.proposed:.3g} "
            f"({direction} by {abs(self.proposed - self.current) / max(abs(self.current), 1e-9):.0%})"
        )
        if self.clamped:
            text += " [limited this run]"
        return text + f" — {self.basis}"


@dataclass(frozen=True)
class ProfileSuggestion:
    """A conservative, evidence-bearing proposal for the measured fields."""

    base: core.Settings
    proposed: core.Settings
    fields: tuple[FieldSuggestion, ...]
    provenance: Mapping[str, Any]
    warnings: tuple[str, ...] = ()
    blocked_reason: str | None = None

    @property
    def allowed(self) -> bool:
        return self.blocked_reason is None

    @property
    def changed_fields(self) -> tuple[FieldSuggestion, ...]:
        return tuple(item for item in self.fields if item.changed)

    def summary_lines(self) -> tuple[str, ...]:
        if self.blocked_reason:
            return (f"No profile is offered: {self.blocked_reason}",)
        lines = [item.describe() for item in self.fields]
        lines.extend(self.warnings)
        return tuple(lines)


def _trim_factor(measured: float, reference: float, clamp: float) -> float:
    """Return a gain trim, bounded to ``1 ± clamp`` and never non-positive."""
    if measured <= 0.0 or reference <= 0.0:
        return 1.0
    factor = reference / measured
    return min(max(factor, 1.0 - clamp), 1.0 + clamp)


def suggest_settings(
    settings: core.Settings,
    report: CalibrationReport,
    *,
    step_limit: float = CONSERVATIVE_STEP_LIMIT,
) -> ProfileSuggestion:
    """Turn a finished measurement into a conservative profile proposal.

    The rules, in order of importance:

    * **No measurement, no proposal.** An incomplete or aborted report, or one
      missing the lock observation, produces a blocked suggestion whose reason
      says what is missing. Nothing here invents a starting value.
    * **Only measured fields move.** :data:`SUGGESTIBLE_FIELDS` is the whole
      list; guards, hotkeys, curves, locks, and every other knob are untouched.
    * **Robust statistics and bounded steps.** Percentiles rather than peaks,
      and at most ``step_limit`` of the current value per run, so one bad sweep
      cannot wreck a working profile. Repeat runs converge.
    * **Every proposal is clamped into the range the core accepts**, so a
      suggestion can never be rejected at load time.
    """
    base = core.sanitise_settings(settings)
    blocked = report.blocking_reason()
    if blocked is not None:
        return ProfileSuggestion(
            base=base,
            proposed=base,
            fields=(),
            provenance=calibration_provenance(report, settings=base, fields={}),
            blocked_reason=blocked,
        )

    warnings: list[str] = []
    fields: list[FieldSuggestion] = []
    working = base

    # The measurement is only as strong as what was actually observed.  When no
    # output trace was sampled -- the console flow is input-only by design, and a
    # GUI run has no engine session to read -- say so in the proposal itself, not
    # only in the provenance.  The offer still stands: the source-side facts are
    # real measurements, and the step limit keeps the result conservative.
    if report.output_trace is None or not report.output_trace.count:
        warnings.append(
            "no output trace was sampled in this run, so the game's response to "
            "the sweep is operator-reported rather than machine-observed; the "
            "proposal stays inside the conservative step limit for that reason"
        )

    # ----- 1. left/right symmetry from the two sweeps -------------------
    right_rate = report.measured(STEP_SWEEP_RIGHT, "rate_units_per_s")
    left_rate = report.measured(STEP_SWEEP_LEFT, "rate_units_per_s")
    if right_rate > 0.0 and left_rate > 0.0:
        mean_rate = 0.5 * (right_rate + left_rate)
        for field_name, measured in (
            ("sens_right", right_rate),
            ("sens_left", left_rate),
        ):
            current = float(getattr(working, field_name))
            factor = _trim_factor(measured, mean_rate, SYMMETRY_CLAMP)
            proposed = current * factor
            clamped = abs(factor - 1.0) >= SYMMETRY_CLAMP - 1e-9
            lo, hi = 0.05, 5.0
            proposed_clamped = min(max(proposed, lo), hi)
            material = (
                abs(proposed_clamped - current) / max(abs(current), 1e-9)
                > MATERIAL_CHANGE_RATIO
            )
            if not material:
                continue
            fields.append(
                FieldSuggestion(
                    field=field_name,
                    layer=LAYER_GLOBAL,
                    current=current,
                    proposed=proposed_clamped,
                    basis=(
                        f"measured sweep rates {left_rate:.0f} left / "
                        f"{right_rate:.0f} right units/s; the same gesture should "
                        "steer both ways equally"
                    ),
                    confidence="medium" if clamped else "high",
                    clamped=clamped,
                )
            )
            working = replace(working, **{field_name: proposed_clamped})
        if abs(right_rate - left_rate) / max(mean_rate, 1e-9) > 2 * SYMMETRY_CLAMP:
            warnings.append(
                f"the two sweeps differ by "
                f"{abs(right_rate - left_rate) / max(mean_rate, 1e-9):.0%}, which is "
                "more than the flow corrects in one run; the trim was limited to "
                f"{SYMMETRY_CLAMP:.0%} and repeat runs will converge"
            )

    # ----- 2. the measured full-lock travel -----------------------------
    measured_travel = report.measured(STEP_LOCK, "full_lock_units")
    if measured_travel > 0.0:
        implied_travel = core.travel_px(working)[1]  # right-hand travel
        current = float(working.full_lock_px)
        if implied_travel <= 0.0:
            warnings.append(
                "the current profile implies no travel for full lock, so the "
                "measured travel cannot be related to it; no range change was "
                "proposed"
            )
        else:
            raw_factor = measured_travel / implied_travel
            factor = min(max(raw_factor, 1.0 - step_limit), 1.0 + step_limit)
            proposed = current * factor
            proposed_clamped = min(max(proposed, FULL_LOCK_MIN_UNITS), FULL_LOCK_MAX_UNITS)
            clamped = (
                abs(factor - raw_factor) > 1e-9
                or abs(proposed_clamped - proposed) > 1e-9
            )
            material = (
                abs(proposed_clamped - current) / max(abs(current), 1e-9)
                > MATERIAL_CHANGE_RATIO
            )
            if material:
                basis = (
                    f"the sweep reached the point you called full lock after "
                    f"{measured_travel:.0f} input units, while the profile implied "
                    f"{implied_travel:.0f}"
                )
                lock_record = report.step(STEP_LOCK)
                if lock_record is not None and (
                    lock_record.observation == OBSERVED_SATURATED
                ):
                    basis += (
                        " (you reported the game was already at full lock before "
                        "that point, so the travel was reduced conservatively)"
                    )
                fields.append(
                    FieldSuggestion(
                        field="full_lock_px",
                        layer=LAYER_GLOBAL,
                        current=current,
                        proposed=proposed_clamped,
                        basis=basis,
                        confidence="medium" if clamped else "high",
                        clamped=clamped,
                    )
                )
                working = replace(working, full_lock_px=proposed_clamped)
            else:
                warnings.append(
                    "the measured full-lock travel already matches the profile "
                    "within "
                    f"{MATERIAL_CHANGE_RATIO:.0%}; the range was left unchanged"
                )

    # ----- 3. the noise gate --------------------------------------------
    noise_floor = report.measured(STEP_SETTLE, "noise_floor_units_per_s")
    if noise_floor > 0.0:
        current = float(working.noise_gate_units_per_s)
        if noise_floor < NOISE_FLOOR_MIN_PROPOSE:
            proposed = 0.0
            basis = (
                f"the measured noise floor was only {noise_floor:.1f} units/s, so "
                "no gate is needed"
            )
        else:
            proposed = min(noise_floor * NOISE_GATE_MULTIPLIER, NOISE_GATE_MAX_PROPOSE)
            basis = (
                f"the mouse drifts {noise_floor:.1f} units/s on its own; a gate at "
                f"{NOISE_GATE_MULTIPLIER:.0f}× that rejects the drift without "
                "touching real movement"
            )
        material = abs(proposed - current) / max(abs(current), 1.0) > (
            MATERIAL_CHANGE_RATIO
        )
        if material:
            fields.append(
                FieldSuggestion(
                    field="noise_gate_units_per_s",
                    layer=LAYER_GLOBAL,
                    current=current,
                    proposed=proposed,
                    basis=basis,
                    confidence="high",
                    clamped=False,
                )
            )
            working = replace(working, noise_gate_units_per_s=proposed)

    # ----- 4. the verification pass -------------------------------------
    verification = report.observations.get(STEP_VERIFY)
    if verification in (OBSERVED_TOO_SENSITIVE, OBSERVED_NOT_ENOUGH):
        # "Too sensitive" means full lock arrives too early, so the range (the
        # travel needed for full lock) grows; "not enough" is the mirror.
        factor = (
            VERIFY_TRIM_FACTOR
            if verification == OBSERVED_TOO_SENSITIVE
            else 1.0 / VERIFY_TRIM_FACTOR
        )
        current = float(working.full_lock_px)
        proposed = min(
            max(current * factor, FULL_LOCK_MIN_UNITS), FULL_LOCK_MAX_UNITS
        )
        fields = [item for item in fields if item.field != "full_lock_px"]
        fields.append(
            FieldSuggestion(
                field="full_lock_px",
                layer=LAYER_GLOBAL,
                current=current,
                proposed=proposed,
                basis=(
                    f"you reported the steering felt {verification.replace('_', ' ')}, "
                    f"so the range was corrected by {abs(factor - 1.0):.0%}; repeat "
                    "the flow to converge"
                ),
                confidence="medium",
            )
        )
        working = replace(working, full_lock_px=proposed)

    proposed_settings = core.sanitise_settings(working)
    provenance = calibration_provenance(
        report,
        settings=base,
        fields={
            item.field: item.basis
            for item in fields
            if item.changed
        },
        verification=verification,
    )
    return ProfileSuggestion(
        base=base,
        proposed=proposed_settings,
        fields=tuple(fields),
        provenance=provenance,
        warnings=tuple(warnings),
    )


def apply_suggestion(
    settings: core.Settings, suggestion: ProfileSuggestion
) -> core.Settings:
    """Return *settings* with only the suggestion's changed fields applied."""
    if not suggestion.allowed:
        raise ValueError(
            "this suggestion carries no measured change to apply: "
            f"{suggestion.blocked_reason}"
        )
    changes = {
        item.field: item.proposed for item in suggestion.changed_fields
    }
    if not changes:
        return core.sanitise_settings(settings)
    return core.sanitise_settings(replace(settings, **changes))


def calibration_provenance(
    report: CalibrationReport,
    *,
    settings: core.Settings,
    fields: Mapping[str, str],
    measured_at: str = "",
    verification: str | None = None,
) -> dict[str, Any]:
    """Build the ``calibration`` section a profile stores as provenance.

    A profile that claims to be calibrated records *what was measured*, not just
    the numbers that came out of it, so a later run -- or a reader of the file --
    can tell a measured value from a hand-typed one.
    """
    facts = report.measured_facts()
    lock = report.step(STEP_LOCK)
    provenance: dict[str, Any] = {
        "schema": CALIBRATION_SCHEMA,
        "backend": report.backend,
        "reported_hz": report.nominal_hz,
        "observed_hz": facts.get("observed_hz", 0.0),
        "elapsed_s": report.elapsed_s,
        "noise_floor_units_per_s": facts.get(STEP_SETTLE + ".noise_floor_units_per_s", 0.0),
        "left_peak_units_per_s": facts.get(STEP_SWEEP_LEFT + ".rate_units_per_s", 0.0),
        "right_peak_units_per_s": facts.get(STEP_SWEEP_RIGHT + ".rate_units_per_s", 0.0),
        "left_travel_units": facts.get(STEP_SWEEP_LEFT + ".travel_units", 0.0),
        "right_travel_units": facts.get(STEP_SWEEP_RIGHT + ".travel_units", 0.0),
        "full_lock_units": facts.get(STEP_LOCK + ".full_lock_units", 0.0),
        "full_lock_observed": lock.observation if lock is not None else None,
        "output_axis_peak": facts.get("output_axis_peak", 0.0),
        "fields": dict(fields),
    }
    if measured_at:
        provenance["measured_at"] = str(measured_at)
    chosen_verification = verification
    if chosen_verification is None:
        chosen_verification = report.observations.get(STEP_VERIFY)
    if chosen_verification:
        provenance["verification"] = chosen_verification
    notes = list(report.notes)
    if report.output_trace is None or not report.output_trace.count:
        notes.append(
            "no output trace was sampled, so full lock is operator-reported "
            "rather than machine-observed"
        )
    if notes:
        provenance["notes"] = notes
    return provenance


# ===========================================================================
# LAYERED PROFILES
# ===========================================================================


@dataclass(frozen=True)
class ProfileLayer:
    """One layer of a profile: a named set of fields in one domain."""

    kind: str
    name: str
    settings: Mapping[str, Any] = field(default_factory=dict)
    notes: tuple[str, ...] = ()
    calibrated: bool = False
    evidence: Mapping[str, Any] = field(default_factory=dict)
    unknown: Mapping[str, Any] = field(default_factory=dict)

    def validate(self) -> None:
        """Raise ``ValueError`` unless this layer is well formed.

        Domain enforcement is the point of the layer split: a game layer that
        tries to set a hotkey, or a car layer that tries to change the input
        mode, is a mistake worth refusing loudly rather than merging.
        """
        if self.kind not in LAYER_KINDS:
            raise ValueError(
                f"layer kind {self.kind!r} is not one of {', '.join(LAYER_KINDS)}"
            )
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError("a profile layer needs a non-empty name")
        if not isinstance(self.settings, Mapping):
            raise ValueError(f"layer {self.name!r} settings must be an object")
        for field_name in sorted(self.settings):
            owner = field_owner_layer(field_name)
            if owner is None:
                raise ValueError(
                    f"layer {self.name!r} ({self.kind}) sets {field_name!r}, which "
                    "no layer owns. Derived and unit-defining fields cannot be set "
                    "by a layer."
                )
            if owner != self.kind:
                raise ValueError(
                    f"layer {self.name!r} is a {self.kind} layer but sets "
                    f"{field_name!r}, which belongs to the {owner} layer"
                )

    def payload(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "kind": self.kind,
            "name": self.name,
            "settings": {
                key: self.settings[key] for key in sorted(self.settings)
            },
        }
        if self.calibrated:
            payload["calibrated"] = True
        if self.evidence:
            payload["evidence"] = dict(self.evidence)
        if self.notes:
            payload["notes"] = list(self.notes)
        if self.unknown:
            payload["extensions"] = dict(self.unknown)
        return payload


@dataclass(frozen=True)
class ProfileBundle:
    """A layered profile document: global policy plus optional game/car/style."""

    layers: tuple[ProfileLayer, ...]
    schema: int = BUNDLE_SCHEMA
    semantics: str = BUNDLE_SEMANTICS
    calibration: Mapping[str, Any] = field(default_factory=dict)
    notes: tuple[str, ...] = ()
    unknown: Mapping[str, Any] = field(default_factory=dict)

    def validate(self) -> None:
        if isinstance(self.schema, bool) or not isinstance(self.schema, int):
            raise ValueError("bundle schema must be an integer")
        if self.schema != BUNDLE_SCHEMA:
            raise ValueError(
                f"unsupported bundle schema {self.schema}; this build writes and "
                f"reads schema {BUNDLE_SCHEMA}"
            )
        if self.semantics != BUNDLE_SEMANTICS:
            raise ValueError(
                f"unsupported bundle semantics {self.semantics!r}; a bundle "
                f"describes {BUNDLE_SEMANTICS} units"
            )
        if not self.layers:
            raise ValueError("a bundle needs at least one layer")
        seen: set[tuple[str, str]] = set()
        owned: dict[str, str] = {}
        for layer in self.layers:
            layer.validate()
            identity = (layer.kind, layer.name)
            if identity in seen:
                raise ValueError(
                    f"duplicate {layer.kind} layer named {layer.name!r}; the merge "
                    "order would be ambiguous"
                )
            seen.add(identity)
            for field_name in sorted(layer.settings):
                previous = owned.get(field_name)
                if previous is not None:
                    raise ValueError(
                        f"{field_name!r} is set by both {previous!r} and "
                        f"{layer.name!r}; layers must not overlap"
                    )
                owned[field_name] = layer.name
        if not isinstance(self.calibration, Mapping):
            raise ValueError("bundle calibration provenance must be an object")

    @property
    def global_layer(self) -> ProfileLayer | None:
        for layer in self.layers:
            if layer.kind == LAYER_GLOBAL:
                return layer
        return None

    def layer(self, kind: str) -> ProfileLayer | None:
        for layer in self.layers:
            if layer.kind == kind:
                return layer
        return None

    def payload(self) -> dict[str, Any]:
        self.validate()
        payload: dict[str, Any] = {
            "schema": self.schema,
            "semantics": self.semantics,
            "layers": [layer.payload() for layer in self.layers],
        }
        if self.calibration:
            payload["calibration"] = dict(self.calibration)
        if self.notes:
            payload["notes"] = list(self.notes)
        if self.unknown:
            payload["extensions"] = dict(self.unknown)
        return payload


@dataclass(frozen=True)
class LayerApplication:
    """One layer's contribution to a merge."""

    kind: str
    name: str
    applied: Mapping[str, Any] = field(default_factory=dict)
    unchanged: tuple[str, ...] = ()

    def describe(self) -> str:
        if not self.applied:
            return f"{self.kind} layer {self.name!r}: no change"
        changes = ", ".join(
            f"{key}={self.applied[key]!r}" for key in sorted(self.applied)
        )
        return f"{self.kind} layer {self.name!r}: {changes}"


@dataclass(frozen=True)
class MergedProfile:
    """The settings a bundle produces, plus how it got there."""

    settings: core.Settings
    applications: tuple[LayerApplication, ...]
    provenance: Mapping[str, Any] = field(default_factory=dict)
    warnings: tuple[str, ...] = ()

    def summary_lines(self) -> tuple[str, ...]:
        lines = [item.describe() for item in self.applications]
        lines.extend(self.warnings)
        return tuple(lines)


def merge_bundle(
    bundle: ProfileBundle,
    base: core.Settings,
) -> MergedProfile:
    """Merge a bundle over *base*, refusing a units mismatch.

    A bundle is a measured, physical-unit document. Applying it to a legacy
    cursor-pixel profile would silently reinterpret every number in it, so that
    combination is refused with the migration path named instead.
    """
    bundle.validate()
    settings = core.sanitise_settings(base)
    if bundle.semantics != settings.profile_semantics:
        raise ValueError(
            f"this bundle describes {bundle.semantics} semantics, but the profile "
            f"in use is {settings.profile_semantics}. Migrate the profile to "
            "schema v2 first: importing would reinterpret its units."
        )
    applications: list[LayerApplication] = []
    warnings: list[str] = []
    for layer in bundle.layers:
        applied: dict[str, Any] = {}
        unchanged: list[str] = []
        for field_name in sorted(layer.settings):
            desired = layer.settings[field_name]
            current = getattr(settings, field_name)
            if current == desired:
                unchanged.append(field_name)
                continue
            try:
                settings = core.sanitise_settings(
                    replace(settings, **{field_name: desired})
                )
            except (OverflowError, TypeError, ValueError) as exc:
                warnings.append(
                    f"{layer.kind} layer {layer.name!r}: {field_name} was refused "
                    f"by validation ({exc}); the base value was kept"
                )
                continue
            applied[field_name] = getattr(settings, field_name)
        applications.append(
            LayerApplication(
                kind=layer.kind,
                name=layer.name,
                applied=applied,
                unchanged=tuple(unchanged),
            )
        )
    return MergedProfile(
        settings=settings,
        applications=tuple(applications),
        provenance=dict(bundle.calibration),
        warnings=tuple(warnings),
    )


def bundle_from_settings(
    settings: core.Settings,
    *,
    name: str = "current",
    calibration: Mapping[str, Any] | None = None,
    notes: Sequence[str] = (),
) -> ProfileBundle:
    """Export *settings* as a bundle, split across its owning layers.

    Every implemented field is placed in the layer that owns it, so a later
    import cannot move a hotkey into a car profile or a lock limit into the
    machine-level policy.
    """
    validated = core.sanitise_settings(settings)
    grouped: dict[str, dict[str, Any]] = {kind: {} for kind in LAYER_KINDS}
    for field_name in _SERIALISABLE_FIELD_NAMES:
        owner = field_owner_layer(field_name)
        if owner is None:
            continue
        grouped[owner][field_name] = getattr(validated, field_name)
    layers = [
        ProfileLayer(kind=kind, name=name, settings=grouped[kind])
        for kind in LAYER_KINDS
        if grouped[kind]
    ]
    return ProfileBundle(
        layers=tuple(layers),
        calibration=dict(calibration or {}),
        notes=tuple(notes),
    )


def calibrated_bundle(
    suggestion: ProfileSuggestion,
    *,
    name: str,
    calibration: Mapping[str, Any] | None = None,
) -> ProfileBundle:
    """Export a suggestion as a bundle whose global layer is marked measured."""
    if not suggestion.allowed:
        raise ValueError(
            "cannot export a calibrated bundle from a blocked suggestion: "
            f"{suggestion.blocked_reason}"
        )
    bundle = bundle_from_settings(
        suggestion.proposed,
        name=name,
        calibration=calibration if calibration is not None else suggestion.provenance,
    )
    layers = []
    for layer in bundle.layers:
        if layer.kind == LAYER_GLOBAL:
            layers.append(
                replace(
                    layer,
                    calibrated=True,
                    evidence=dict(suggestion.provenance),
                    notes=(
                        "measured by the calibration flow; the fields below were "
                        "derived from that measurement, not typed in",
                    ),
                )
            )
        else:
            layers.append(layer)
    return replace(bundle, layers=tuple(layers))


#: Every serialisable runtime field, in a stable order, used for exports. The
#: list is explicit rather than derived so a future internal field cannot leak
#: into an exported bundle merely because it exists.
_SERIALISABLE_FIELD_NAMES = (
    "input_mode",
    "full_lock_px",
    "raw_scale",
    "mouse_sensitivity",
    "sens_left",
    "sens_right",
    "update_hz",
    "smoothing_tau_ms",
    "noise_gate_units_per_s",
    "centering_hysteresis_units_per_s",
    "smoothing",
    "noise_gate_px",
    "hysteresis_px",
    "hotkey_pause",
    "hotkey_stop",
    "deadman_enabled",
    "deadman_key",
    "focus_guard_enabled",
    "focus_allowlist",
    "safety_fail_mode",
    "curve_preset",
    "curve_exp",
    "output_deadzone",
    "max_slew_rate",
    "lock_left",
    "lock_right",
    "max_steering_rate",
    "max_reversal_rate",
    "steering_accel",
    "center_strength",
    "center_return_time_ms",
    "idle_grace_ms",
    "precision_zone",
    "precision_gain",
    "invert_axis",
    "steering_mode",
    "saturate_clean",
    "center_curve",
)


# ===========================================================================
# IMPORT / EXPORT
# ===========================================================================


@dataclass(frozen=True)
class BundleLoadResult:
    """The outcome of reading a bundle file, with an explicit status."""

    status: str
    path: Path
    bundle: ProfileBundle | None = None
    detail: str = ""
    source_schema: int | None = None
    notices: tuple[str, ...] = ()

    @property
    def usable(self) -> bool:
        return self.status == BUNDLE_STATUS_LOADED and self.bundle is not None

    def summary(self) -> str:
        if self.usable:
            suffix = " ".join(self.notices)
            return suffix or f"Loaded bundle {self.path}."
        return f"{self.status}: {self.detail}"


def _bundle_string_list(value: Any, *, name: str) -> tuple[str, ...]:
    if value is None:
        return ()
    if isinstance(value, str) or not isinstance(value, Sequence):
        raise ValueError(f"{name} must be a list of strings")
    result: list[str] = []
    for item in value:
        if not isinstance(item, str):
            raise ValueError(f"{name} must contain only strings")
        result.append(item)
    return tuple(result)


_LAYER_KNOWN_KEYS = frozenset(
    ("kind", "name", "settings", "notes", "calibrated", "evidence", "extensions")
)
_BUNDLE_KNOWN_KEYS = frozenset(
    ("schema", "semantics", "layers", "calibration", "notes", "extensions")
)


def bundle_from_payload(payload: Any) -> ProfileBundle:
    """Validate a parsed JSON payload into a bundle, or raise ``ValueError``.

    Validation is deliberately complete before a merge is attempted: an invalid
    bundle must be reported as invalid, never partially applied.
    """
    if not isinstance(payload, Mapping):
        raise ValueError("a bundle root must be a JSON object")
    schema = payload.get("schema", BUNDLE_SCHEMA)
    if isinstance(schema, bool) or not isinstance(schema, int):
        raise ValueError("bundle schema must be an integer")
    if schema != BUNDLE_SCHEMA:
        raise ValueError(
            f"unsupported bundle schema {schema}; this build reads schema "
            f"{BUNDLE_SCHEMA}"
        )
    semantics = payload.get("semantics", BUNDLE_SEMANTICS)
    if semantics != BUNDLE_SEMANTICS:
        raise ValueError(
            f"unsupported bundle semantics {semantics!r}; expected "
            f"{BUNDLE_SEMANTICS!r}"
        )
    raw_layers = payload.get("layers")
    if not isinstance(raw_layers, Sequence) or isinstance(raw_layers, (str, bytes)):
        raise ValueError("bundle layers must be a list")

    layers: list[ProfileLayer] = []
    for index, raw in enumerate(raw_layers):
        if not isinstance(raw, Mapping):
            raise ValueError(f"layer {index} must be a JSON object")
        unknown = {
            key: value
            for key, value in raw.items()
            if key not in _LAYER_KNOWN_KEYS
        }
        extensions = raw.get("extensions")
        if extensions is not None:
            if not isinstance(extensions, Mapping):
                raise ValueError(f"layer {index} extensions must be an object")
            unknown = {**dict(extensions), **unknown}
        settings = raw.get("settings", {})
        if not isinstance(settings, Mapping):
            raise ValueError(f"layer {index} settings must be an object")
        calibrated = raw.get("calibrated", False)
        if not isinstance(calibrated, bool):
            raise ValueError(f"layer {index} calibrated must be true or false")
        evidence = raw.get("evidence", {})
        if not isinstance(evidence, Mapping):
            raise ValueError(f"layer {index} evidence must be an object")
        layers.append(
            ProfileLayer(
                kind=raw.get("kind", ""),
                name=raw.get("name", ""),
                settings=dict(settings),
                notes=_bundle_string_list(raw.get("notes"), name="layer notes"),
                calibrated=calibrated,
                evidence=dict(evidence),
                unknown=unknown,
            )
        )

    calibration = payload.get("calibration", {})
    if not isinstance(calibration, Mapping):
        raise ValueError("bundle calibration must be an object")
    unknown_root = {
        key: value for key, value in payload.items() if key not in _BUNDLE_KNOWN_KEYS
    }
    extensions_root = payload.get("extensions")
    if extensions_root is not None:
        if not isinstance(extensions_root, Mapping):
            raise ValueError("bundle extensions must be an object")
        unknown_root = {**dict(extensions_root), **unknown_root}

    bundle = ProfileBundle(
        layers=tuple(layers),
        schema=schema,
        semantics=semantics,
        calibration=dict(calibration),
        notes=_bundle_string_list(payload.get("notes"), name="bundle notes"),
        unknown=unknown_root,
    )
    bundle.validate()
    return bundle


def parse_bundle_text(text: str, path: Path | None = None) -> BundleLoadResult:
    """Parse bundle JSON text into a loader result (no I/O)."""
    target = Path(path) if path is not None else Path("<memory>")
    try:
        payload = core.parse_json_object(text)
    except (json.JSONDecodeError, ValueError) as exc:
        return BundleLoadResult(
            status=BUNDLE_STATUS_CORRUPT,
            path=target,
            detail=f"Malformed bundle JSON: {exc}",
        )
    try:
        bundle = bundle_from_payload(payload)
    except (OverflowError, TypeError, ValueError) as exc:
        source_schema = (
            payload.get("schema") if isinstance(payload, Mapping) else None
        )
        status = (
            BUNDLE_STATUS_UNSUPPORTED_VERSION
            if "unsupported bundle schema" in str(exc)
            else BUNDLE_STATUS_VALIDATION_ERROR
        )
        return BundleLoadResult(
            status=status,
            path=target,
            detail=str(exc),
            source_schema=source_schema if isinstance(source_schema, int) else None,
        )
    notices: list[str] = []
    if bundle.unknown:
        notices.append(
            "Unknown bundle fields were retained: "
            + ", ".join(sorted(bundle.unknown))
            + "."
        )
    for layer in bundle.layers:
        if layer.unknown:
            notices.append(
                f"Unknown fields in the {layer.kind} layer {layer.name!r} were "
                "retained: " + ", ".join(sorted(layer.unknown)) + "."
            )
    return BundleLoadResult(
        status=BUNDLE_STATUS_LOADED,
        path=target,
        bundle=bundle,
        source_schema=bundle.schema,
        notices=tuple(notices),
    )


def load_bundle(path: Path) -> BundleLoadResult:
    """Read a bundle file with an explicit status rather than an exception."""
    target = Path(path)
    try:
        text = target.read_text(encoding="utf-8")
    except FileNotFoundError:
        return BundleLoadResult(
            status=BUNDLE_STATUS_MISSING,
            path=target,
            detail="Bundle file does not exist.",
        )
    except UnicodeError as exc:
        return BundleLoadResult(
            status=BUNDLE_STATUS_CORRUPT,
            path=target,
            detail=f"Bundle is not valid UTF-8 text: {exc}",
        )
    except OSError as exc:
        return BundleLoadResult(
            status=BUNDLE_STATUS_IO_ERROR,
            path=target,
            detail=f"Could not read bundle: {type(exc).__name__}: {exc}",
        )
    return parse_bundle_text(text, target)


def write_bundle(bundle: ProfileBundle, path: Path) -> Path | None:
    """Write a validated bundle atomically, retaining the previous file."""
    payload = bundle.payload()
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    return core.write_json_atomically(target, payload)


def bundle_text(bundle: ProfileBundle) -> str:
    """Canonical, diffable JSON text for a bundle (sorted keys, 2-space)."""
    return (
        json.dumps(
            bundle.payload(),
            indent=2,
            sort_keys=True,
            ensure_ascii=False,
        )
        + "\n"
    )


# ===========================================================================
# PRECONDITIONS
# ===========================================================================


@dataclass(frozen=True)
class CalibrationGuard:
    """Whether calibration may start, and why not when it may not."""

    allowed: bool
    reason: str

    def require(self) -> None:
        if not self.allowed:
            raise RuntimeError(self.reason)


def calibration_precondition(
    lifecycle_state: str,
    *,
    engine_running_states: Sequence[str] = ("RUNNING", "PAUSED"),
) -> CalibrationGuard:
    """Refuse to calibrate while a session is driving the virtual pad.

    The flow measures the *source*, and it deliberately never creates a gamepad
    of its own. If a session is also running, the same mouse movement would be
    steering the pad at the same time, which would both corrupt the measurement
    and surprise the operator. Stopping first is the honest requirement.
    """
    state = str(lifecycle_state or "STOPPED")
    if state in engine_running_states:
        return CalibrationGuard(
            allowed=False,
            reason=(
                f"a steering session is {state.lower()}. Stop it first: calibration "
                "measures the mouse while it is not steering the virtual pad, and "
                "it never creates a pad of its own."
            ),
        )
    if state in ("STARTING", "STOPPING", "FAULTED"):
        return CalibrationGuard(
            allowed=False,
            reason=(
                f"the engine is {state.lower()}; wait until it is fully stopped "
                "before calibrating"
            ),
        )
    return CalibrationGuard(
        allowed=True,
        reason="no session is running, so the mouse is free to be measured",
    )


__all__ = [
    "BUNDLE_SCHEMA",
    "BUNDLE_SEMANTICS",
    "BUNDLE_STATUS_CORRUPT",
    "BUNDLE_STATUS_IO_ERROR",
    "BUNDLE_STATUS_LOADED",
    "BUNDLE_STATUS_MISSING",
    "BUNDLE_STATUS_UNSUPPORTED_VERSION",
    "BUNDLE_STATUS_VALIDATION_ERROR",
    "BundleLoadResult",
    "CALIBRATION_SCHEMA",
    "CONSERVATIVE_STEP_LIMIT",
    "CalibrationGuard",
    "CalibrationReport",
    "CalibrationSession",
    "CalibrationStep",
    "DEFAULT_STEPS",
    "FIELD_OWNER_LAYER",
    "FULL_LOCK_MAX_UNITS",
    "FULL_LOCK_MIN_UNITS",
    "FieldSuggestion",
    "LAYER_CAR",
    "LAYER_FIELD_DOMAINS",
    "LAYER_GAME",
    "LAYER_GLOBAL",
    "LAYER_KINDS",
    "LAYER_STYLE",
    "LOCK_OBSERVATIONS",
    "LayerApplication",
    "MergedProfile",
    "NOISE_FLOOR_MIN_PROPOSE",
    "NOISE_GATE_MAX_PROPOSE",
    "NOISE_GATE_MULTIPLIER",
    "OBSERVED_ABOUT_RIGHT",
    "OBSERVED_FULL",
    "OBSERVED_NOT_ENOUGH",
    "OBSERVED_PARTIAL",
    "OBSERVED_SATURATED",
    "OBSERVED_TOO_SENSITIVE",
    "PHASE_ABORTED",
    "PHASE_CONFIRM",
    "PHASE_DONE",
    "PHASE_MEASURING",
    "PHASE_STUCK",
    "PHASE_WAITING",
    "PROFILE_DIR",
    "ProfileBundle",
    "ProfileLayer",
    "ProfileSuggestion",
    "STEP_LOCK",
    "STEP_SETTLE",
    "STEP_SWEEP_LEFT",
    "STEP_SWEEP_RIGHT",
    "STEP_VERIFY",
    "SUGGESTIBLE_FIELDS",
    "SYMMETRY_CLAMP",
    "SourceSample",
    "SourceTrace",
    "StepEvidence",
    "StepOutcome",
    "OutputSample",
    "OutputTrace",
    "TraceAnalysis",
    "VERIFY_OBSERVATIONS",
    "VERIFY_TRIM_FACTOR",
    "analyse_trace",
    "apply_suggestion",
    "bundle_from_payload",
    "bundle_from_settings",
    "bundle_text",
    "calibrated_bundle",
    "calibration_precondition",
    "calibration_provenance",
    "field_owner_layer",
    "load_bundle",
    "merge_bundle",
    "parse_bundle_text",
    "suggest_settings",
    "write_bundle",
]
