# Yaw retune candidates (2026-09-27, packaged 2026-09-28)

Two candidates for the slow release of the damped vectored airship
(`yaw_retune_20260927.md`). Neither is applied to the branch; both are for
PX4-Autopilot at 860ebca856.

| file | what |
|---|---|
| `option_d_restore.diff` | option D, the whole code change: hold the yaw integrator's stick-centred value across a stick deflection |
| `apply_restore.sh` | `git apply` of the diff, then AS_YAWRATE_P 0.5 -> 0.7 (the P default belongs to the gain decision, so it is not in the diff) |
| `apply_ff.py` | option B: AS_YAWRATE_FF feedforward, P 0.65, I 0.07, FF on the three posix airframes; it accepts P 0.5 or 0.7, so it stacks after D |

## Option D, what the diff does

- `lib/rate_control`: `RateControl::setIntegral(axis, value)` (clamped to the
  integrator limit, ignored when not finite or for an axis past 2) and its
  read side `getIntegral(axis) const` (0 for an axis past 2), beside
  `resetIntegral(axis)`; `RateControlTest.SetIntegralIsClampedAndOutputNextCycle`
  reads every write back.
- `airship_manual_input.hpp`: `yawStickDeflected()`, `yawStickReleased()`
  (the band edges; written as `fabsf(x) > 0.f`, exact because `math::deadzone`
  returns an exact 0 inside the band) and `YawIntegratorHold`, the state
  machine: `update(yaw_rate_sp, deadzone, rate_control)` saves the integrator
  through `getIntegral(2)` on the deflection edge, before the loop runs on the
  new setpoint, and puts it back through `setIntegral(2, ...)` on the release
  edge or a crossing of the band; `reset()` when the loop opens. It returns
  `Event::{None, Deflected, Released}` so a test can pin the sequence.
- the module: one member `_yaw_integrator_hold`, one `update()` call before
  `controlYawRate()`, `reset()` next to `resetIntegral()` where the loop
  opens. Nothing is named "trim": on this branch that word is the allocator's
  surface offset (CA_SV_CSn_TRIM).
- docs: the release rule and its MAN_DEADZONE 0 behaviour in the module
  description, AS_YAWRATE_I long and AS_YAWRATE_MAX long. With MAN_DEADZONE 0
  only a stick that reads exactly 0.0 is a release; rc_update on this tree
  applies no per-channel dead zone, so on RC keep MAN_DEADZONE above 0.
- tests, `AirshipManualInputTest`: `StickEventsAreTheDeadzoneEdges` (the
  predicates), `YawIntegratorReturnsToItsCentredValueOnRelease` and
  `YawIntegratorHoldStartsFromZeroWithTheLoop` (the whole sequence against a
  real `RateControl` at the AS_YAWRATE_I / AS_YR_INT_LIM defaults: wind while
  centred, deflect, drift, release, crossing, a second release, the loop
  re-closing with the stick already out).

Against the previous packaging (the `proposal/` copy under
`../options/D/`): that diff's header hunk compared floats with `==` and did
not build under PX4's `-Werror -Wfloat-equal`; a second copy of the header
was overlaid by the script. There is one artifact now. The restore rule
itself is unchanged: the same two predicates, the same snapshot/restore
order before `controlYawRate()`. The state reset moved from "lazily on the
first closed cycle" to "when the loop opens", which sees the same zero state
at every re-close (the state starts at zero and the open branch runs once
per closed period).

## How it was verified (2026-09-28, never applied to the tree)

- `git apply --check option_d_restore.diff` at 860ebca856: clean.
- `-fsyntax-only` with the exact flags of `build/px4_sitl_default` and
  `build/px4_sitl_test/compile_commands.json` on a scratch copy of the
  patched files: `rate_control.cpp`, `airship_att_control_main.cpp` (both
  builds), `rate_control_test.cpp`, `AirshipManualInputTest.cpp` (test build),
  all exit 0. `Tools/astyle/fix_code_style.sh` leaves every file unchanged.
- The two gtest binaries built from the scratch copy against `gtest-all.cc`
  from the build tree: 21/21 and 2/2 pass. Eleven mutants, each of which
  must fail a test:

| mutant | caught by |
|---|---|
| no `setIntegral` on release | both sequence tests |
| no snapshot on deflection | `YawIntegratorReturnsToItsCentredValueOnRelease` |
| snapshot on every cycle | both sequence tests |
| a crossing counts as a deflection (re-snapshot) | `StickEventsAreTheDeadzoneEdges`, the sequence test |
| previous setpoint updated before the predicates | both sequence tests |
| `reset()` keeps the previous setpoint | `YawIntegratorHoldStartsFromZeroWithTheLoop` |
| a crossing releases without a deadzone | `StickEventsAreTheDeadzoneEdges` |
| `setIntegral` without the clamp | `RateControlTest` |
| `setIntegral` without the finite guard | `RateControlTest` |
| `getIntegral` without the axis guard | build error (`-Warray-bounds` at -O2) |
| `setIntegral` without the axis guard | not caught: the write past the vector is undefined behaviour, not an observable |

The `make tests` run on the branch after `apply_restore.sh` is the owner's;
the numbers above come from the scratch build.
