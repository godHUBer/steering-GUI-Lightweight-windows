# Mouse Steering

Analog steering from a mouse, delivered to games through a **virtual Xbox 360 controller**. Move the mouse, get a smooth, tunable steering axis, with a tuning GUI, a guided calibration flow, and opt-in safety controls.

> **Windows only.** The virtual pad is provided by [ViGEmBus](https://github.com/nefarius/ViGEmBus), which is Windows-only.

## Features

- **Raw Input by default.** Reads relative HID mouse counts via the Windows Raw Input API. It never reads or repositions the system cursor, so there are no cursor warps and no edge-of-screen limits.
- **Cursor fallback mode.** A retained, visibly *degraded* `pynput` cursor-coordinate backend for hosts where Raw Input isn't suitable.
- **Tunable response.** Linear, Squared, Cubed, S-Curve, Hybrid, or custom-power curves, plus per-side sensitivity and lock, a precision zone, steering acceleration, rate and reversal limits, output slew, and output deadzone.
- **Three steering modes.** `release_to_center`, `continuous_spring`, and `manual`.
- **Tuning GUI.** Tabs for **Main**, **Advanced settings**, **Calibration**, and **Diagnostics**. Diagnostics explains why steering is centring or ignoring input and whether the pad is connected. A live indicator banner shows whether the pad is actively steering, held by a guard, or idle.
- **Guided calibration.** Measures your mouse and proposes conservative profile values from real evidence (see [Calibration](#calibration)).
- **Layered profiles.** Export and import profiles as `global` / `game` / `car` / `style` bundles.
- **Operational safety.** Configurable pause/stop hotkeys, a hold-to-enable deadman key, and an opt-in foreground-window allowlist (see [Safety controls](#safety-controls)).
- **Fail-safe runtime.** Every run has a session ID and lifecycle state, so stale workers can't affect a replacement run. Control-loop faults are caught and cleaned up, and the GUI stays open until cleanup is confirmed.
- **Validated presets.** JSON presets are sanitised on load. Corrupt, unsupported, or invalid files are reported instead of silently used.

## Requirements

- Windows 10 or later (Per-Monitor DPI awareness v2 is used where available)
- Python 3 (the bundled bytecode was built with CPython 3.14)
- [ViGEmBus](https://github.com/nefarius/ViGEmBus) driver
- Python packages: `vgamepad`, `pynput`

## Installation

```bash
pip install vgamepad pynput
```

Installing `vgamepad` also installs the ViGEmBus driver. Accept the driver prompt when it appears. If startup reports that the virtual pad couldn't be initialised, confirm that ViGEmBus is installed and working, then retry.

`--help` and the pure core modules work without ViGEm or Windows, since `vgamepad` is loaded lazily.

## Usage

```bash
python src/mouse_steering.py                               # tuning GUI
python src/mouse_steering.py --cli                         # console-only mode
python src/mouse_steering.py --profile profiles/rally.json # load a specific profile
python src/mouse_steering.py --input-mode raw_input        # default backend
python src/mouse_steering.py --input-mode cursor_fallback  # degraded backend
python src/mouse_steering.py --log-level info
python src/mouse_steering.py --dpi-diagnostics             # support output, then exit
python src/mouse_steering.py --help
```

### Default hotkeys

| Key | Action | Setting |
| --- | --- | --- |
| `F8` | Pause / resume | `hotkey_pause` |
| `F9` | Emergency stop | `hotkey_stop` |
| `F10` | Deadman hold (only when enabled) | `deadman_key` |
| `Ctrl+C` | Quit in CLI mode | n/a |

### Command-line options

| Option | Description |
| --- | --- |
| `--cli` | Run the console interface instead of the GUI. |
| `--profile PATH` | Load this preset. It must load successfully, otherwise the launch fails rather than falling back to unrelated defaults. |
| `--input-mode {raw_input,cursor_fallback}` | Override the profile's input backend for this launch. |
| `--raw-input`, `--cursor-fallback` | Compatibility aliases for the matching `--input-mode` values. |
| `--log-level LEVEL` | `debug`, `info`, `warning` (default), `error`, or `critical`. |
| `--dpi-diagnostics` | Print Windows DPI/coordinate diagnostics and exit. |
| `--calibrate` | Measure the mouse and offer a conservative profile. Input-only; never creates a virtual pad. |
| `--calibrate-write PATH` | Save the calibrated profile to `PATH`. By default nothing is written. |
| `--calibrate-live` | Also measure the output trace by running a real steering session during calibration. Opt-in: the virtual pad steers while you measure. |
| `--profile-export PATH` | Write the profile in use as a layered bundle and exit. |
| `--profile-import PATH` | Merge a layered bundle over the profile in use, then launch with the result. |

Some combinations are rejected up front: `--dpi-diagnostics` can't be combined with launch or profile options, `--calibrate-live` and `--calibrate-write` require `--calibrate`, and `--profile-export` can't be combined with `--calibrate`. Abbreviated flags are not accepted.

## Profiles

Settings are stored as JSON presets. With no `--profile`, the default preset `mouse_steering.json` (next to the script) is used.

- **Missing default preset:** starts from intentional native-v2 defaults (Raw Input, explicit units, continuous spring, axis-target return-time centring).
- **Corrupt or unsupported default preset:** prints a warning and starts from fresh v2 defaults *without rewriting the file*.
- **Explicit `--profile` that can't be used:** exits with status 2.
- **Schema v1 profiles** are migrated while preserving their exact control behaviour, so a version bump never changes steering feel.

### Steering modes

| Mode | Behaviour |
| --- | --- |
| `release_to_center` | Steering returns to centre when mouse input stops. |
| `continuous_spring` | Native-v2 default: a spring pulls the axis toward centre continuously. |
| `manual` | No automatic return; the axis stays where you leave it. |

### Layered bundles

`--profile-export` and `--profile-import` use a layered format so you can share and recombine parts of a setup. Each layer may only own certain fields:

| Layer | Owns |
| --- | --- |
| `global` | Input backend, sensitivity, scale, update rate, filtering, and the safety settings |
| `game` | Curve preset and exponent, output deadzone, output slew |
| `car` | Per-side lock, rate/reversal limits, acceleration, centring strength, return time, idle grace |
| `style` | Precision zone and gain, axis inversion, steering mode, clean saturation, centring curve |

Bundles carry measured physical units. Importing one onto a legacy (cursor-pixel) profile is refused, since it would silently reinterpret units.

## Calibration

You shouldn't have to guess how desktop DPI, mouse polling rate, and a game's own controller filtering combine. Run `--calibrate` (or use the GUI's **Calibration** tab) for a guided flow:

1. **Measure the noise floor.** Leave the mouse still.
2. **Sweep right** at your normal driving speed.
3. **Sweep left** the same way (reveals left/right asymmetry).
4. **Measure full-lock travel.** Sweep until the car reaches full lock in-game, then report what the game did.
5. **Verify in the game** *(optional)*. Drive for a moment and report how it felt.

Calibration is deliberately conservative:

- It proposes values only for fields with measured evidence: `full_lock_px`, `sens_left`, `sens_right`, and `noise_gate_units_per_s`.
- It uses robust statistics (percentiles, gap and stall detection) instead of trusting one peak sample.
- It limits how far a single run can move any value, and clamps every proposal into the validated domain.
- Without a usable measured trace it refuses and says what is missing.
- Nothing is written unless you pass `--calibrate-write` (or save from the GUI).

## Safety controls

These guards decide *whether* steering is applied. They never change how it is computed. All are opt-in except the pause and stop hotkeys.

- **Pause/stop hotkeys.** Defaults `F8` and `F9`; configurable, including chords such as `<ctrl>+<shift>+h`.
- **Deadman key.** When enabled, steering is applied only while the key is held.
- **Foreground allowlist.** When enabled, steering auto-pauses if an unlisted window has focus. Entries are `proc:name.exe`, `title:substring`, or a bare rule that matches either. The focus is sampled at 10 Hz. An armed guard with an empty allowlist fails validation at profile load.
- **Fail mode.** `safety_fail_mode` is `fail_closed` (default) or `fail_open`, controlling what happens when the foreground window can't be determined.
- **Input indicator.** A prominent banner reports active, held, inactive, degraded, or fault states.

> **Fair use.** These controls exist so a driver can deliberately gate or kill mouse-to-pad steering. None of them hide from, defeat, or bypass any anti-cheat system. Each game's and platform's rules still apply. Check them before using this tool online. See `docs/GAME_POLICY.md`.

## Project layout

| File | Role |
| --- | --- |
| `mouse_steering.py` | Entry point: argument parsing, the steering engine and runtime sessions, the CLI, and the Tk GUI. |
| `steering_core.py` | Platform-independent settings, validation, curve and steering math, and preset load/save/migration. No Tk, pynput, vgamepad, or ctypes. |
| `raw_input.py` | Windows Raw Input relative-mouse source: a message-only window on its own thread feeding a bounded, timestamped queue that drops the *oldest* events under overload, so a stall never replays stale steering. |
| `safety.py` | Policy for hotkeys, deadman, foreground gating, and the indicator state. Win32 access is lazy and confined to its foreground probe. |
| `calibration.py` | Trace analysis, the deterministic calibration state machine, conservative suggestions, and layered profile bundles. |

Most of the logic is pure and deterministic: no module touches Win32, Tk, or a gamepad at import time, so the core can be tested on any platform without a display, a mouse, or a game.

## Troubleshooting

- **"Could not initialise vgamepad/ViGEmBus".** Make sure `pip install vgamepad pynput` completed and the ViGEmBus driver is installed and working.
- **Steering feels wrong or inconsistent at different scalings.** Run `--dpi-diagnostics` and include its output when asking for help, then try `--calibrate`.
- **Steering stops when you alt-tab.** The foreground allowlist is probably armed. Check `focus_guard_enabled` and `focus_allowlist`, or look at the Diagnostics tab.
- **Using `cursor_fallback`.** It is a degraded compatibility mode. Prefer `raw_input` whenever possible.

## License

_Add your license here._
