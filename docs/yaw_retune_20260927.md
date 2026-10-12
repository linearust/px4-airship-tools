# The yaw rate loop on the tailed airships: retune record (2026-09-27)

Two gate failures were left after the tail was sized: the damped vectored
airship keeps turning long after the stick is released, and the complete
Cloudship overshoots in Acro. A validated model (yaw_model/) found the causes
and judged the options; this note records what the simulator then measured.

## The model

controller.py ports airship_att_control, RateControl and the allocator's
saturation flags cycle by cycle (250 Hz); against six flight logs the torque
and integrator match to 1.3e-4. planar.RollPlanarPlant carries surge, sway,
roll and yaw with the added-mass block (Munk moment included), hull drag at
the link origin and a replica of the lift-drag plugin. With the current gains
it reproduces all 48 recorded gate verdicts on five plants, and the gz ground
truth of the hulls' drift (from the ulogs) to 0.01-0.05 m/s.

Causes:
- Damped vectored airship, slow release: at release the integrator holds the
  torque that held the turn against the hull's quadratic damping (about 0.09
  of authority), and unwinds only at I x error, about 0.01/s. After a pod
  swing the allocator's steer band also withholds a reversed demand up to
  0.02 without flagging saturation. No gain set fixes this (10-set sweep).
- Complete Cloudship, Acro overshoot: the gate's clean start waits for the
  yaw rate only, the hull keeps drifting at 0.15-0.33 m/s, and the Munk
  moment -4.92 u v cycles once per 180 deg of heading; with the Munk term
  off the model passes.
- Rigid vectored brake: an undamped 1.66 s roll pendulum couples into the
  yaw rate through ixz; P 0.7 brakes inside the window.

Limits: pitch and heave are held (Cloudship-complete margins under 0.15 deg/s
are undecided); noise-free peaks read 0-0.12 deg/s low; the rigid vectored
peak and brake carry 0.3-0.4 deg/s of ripple uncertainty.

## Stage 1: AS_YAWRATE_P 0.7, no code change (chain10, 2026-09-27)

| plant | measured | model |
|---|---|---|
| 2520 rigid vectored | 10 / 10; brake 28.3 -> -5.3; Acro peak 8.36; wind I off 1.51 | 10 / 10; -4.8..-5.4; 8.26-8.40; 1.48-1.53 |
| 2527 complete vectored | 8 / 10; releases end 2.5 / 2.4 deg/s, still falling; wind I off 1.13 | 8 / 10; 2.36-2.61; 1.12-1.17 |
| 2522 vectored fin | 8 / 10; releases end 0.9 / 0.7, still falling | 8-9 / 10; 0.9-1.15 |
| 2510 rigid Cloudship | 9 / 9 on the rerun (the first run read 8 / 9: a wrench publish was dropped and the loop had nothing to hold) | 9 / 9 |
| 2517 complete Cloudship | 9 / 9; Acro peak 9.13 (limit 9.33) | 9 / 9; 8.84-9.09 |

So P 0.7 alone fixes the rigid vectored brake (failing since 2026-09-09) and
the complete Cloudship's overshoot, and the model is confirmed at a gain it
was not built on. The damped vectored releases remain, as it said.

The dropped wrench: `gz topic -p` reports success when the simulator drops
the message. sitl_check.py now judges each wind case by effect (the loop's
torque must reach half the push's size), retries the publish once, and
otherwise reports the case as unmeasured.

## Stage 2: the two candidates for the release

Both run on uncommitted code with the unit tests first (stage2_20260927.txt);
the tree goes back to HEAD 860ebca856 after each.

### Candidate D, restore rule + P 0.7: 48 / 48 (measured 2026-09-27)

| plant | result | measured (model) |
|---|---|---|
| 2520 rigid vectored | 10 / 10 | brake -5.3 (-4.8..-5.4); Acro peak 8.30 (8.26-8.40); releases 4.8 / 4.1 s (3.8-4.8); wind I off 1.50 |
| 2527 complete vectored | 10 / 10 | releases settle in 7.9 / 7.9 s (7.6-8.2); wind I on integrator 0.0215 (needed 0.0185); wind I off 1.15 (1.13-1.17) |
| 2522 vectored fin | 10 / 10 | releases 5.4 / 5.0 s (5.0-5.4); wind I off 1.27 |
| 2510 rigid Cloudship | 9 / 9 | releases 2.8 / 2.8 s (2.8-2.9); wind I on integrator 0.0185 |
| 2517 complete Cloudship | 9 / 9 | Acro peak 8.96 (8.84-9.09, limit 9.33); Acro release 3.2 s (3.2); Stab release 5.1 s (5.1); wind I on integrator 0.0215, rate 0.51 |

Unit tests with the patch: AirshipManualInput 19 cases and rate_control 2
cases, all passing.

- D, restore: remember the yaw integrator when the stick leaves the deadzone,
  put it back when it returns (setIntegral() added to lib/rate_control, two
  helpers, no new parameter), with P 0.7. Model: 52/52 on all five plants.
  Cost: keeps wind rejection only while the wind is constant through the
  turn (a mid-turn wind change costs about 30 deg of heading and 15-21 s to
  hold), a torque step of up to 0.09 at release, a 20 deg swing-back.
- B, feedforward: torque += AS_YAWRATE_FF x rate setpoint (RateControl's
  existing term; PX4's fixed-wing FF "compensates aerodynamic damping"), with
  P 0.65, I 0.07 and FF per plant (2527 1.0, 2522 0.3, 2517 0.05, rigid 0).
  Model: 40/40 on all five plants. Cost: a new parameter whose value is the
  plant's damping torque per unit rate, measured per vehicle; narrow windows.

### Candidate B, feedforward, P 0.65 I 0.07, FF per plant: 48 / 48 (measured 2026-09-28)

| plant | FF | result | measured (model) |
|---|---|---|---|
| 2520 rigid vectored | 0 | 10 / 10 | brake -5.0; Acro peak 8.30; releases 4.7 / 4.7 s; wind I off 1.62 |
| 2527 complete vectored | 1.0 | 10 / 10 | tracking mean 7.11 / 7.03 on 6.67 (FF over-supplies at this rate); releases 8.8 / 8.8 s (8.5-9.2); wind I on integrator 0.0224; wind I off 1.24 |
| 2522 vectored fin | 0.3 | 10 / 10 | releases 7.0 / 6.3 s (6.0-7.6); wind I off 1.36 |
| 2510 rigid Cloudship | 0 | 9 / 9 | releases 3.2 / 2.8 s |
| 2517 complete Cloudship | 0.05 | 9 / 9 | Acro peak 9.02 (9.10 predicted, limit 9.33); Stab release 7.6 s (7.3-7.9) |

## What the measurements say

- AS_YAWRATE_P 0.7 alone is confirmed: it fixes the rigid vectored brake and the
  complete Cloudship's overshoot, changes no parameter name and no code, and
  leaves only the damped vectored releases failing (41 / 48).
- Either mechanism closes the rest (48 / 48), and the model predicted every
  number to within its stated uncertainty on both. The choice is not about
  the gate; it is about the real vehicle:
  - D (restore) needs no new parameter and no measurement on the vehicle, but
    it keeps wind rejection only while the wind stays constant through the
    turn. On a real airship the relative wind changes during every turn, so
    the restored trim is stale after a turn: about 30 deg of heading and
    15-21 s to hold per 5 % of authority in the model. It also steps the
    torque at every release (up to 0.09 of authority, an immediate pod swing)
    and adds a public method to the shared rate-control library.
  - B (feedforward) is PX4's own mechanism for a damped plant (the fixed-wing
    FW_*_FF compensates aerodynamic damping) and keeps the integrator holding
    only the wind trim, so a mid-turn wind change costs nothing extra. Its
    cost is a new parameter, AS_YAWRATE_FF, whose value is the plant's
    damping torque per unit rate: 0 until measured on a vehicle, and right at
    one rate only on a quadratically damped hull. In the gate it runs the
    turn 7 % fast on the damped vectored airship.
- Not measured on hardware: the bench FMU (1965de11ab, behaviour-identical to
  HEAD 860ebca856) still runs P 0.5. THR_MDL_FAC 0 and an unmeasured tilt
  servo mean the real vehicles need their own retune whatever is chosen.

Records: sitl_out/chain10_20260927.txt (stage 1), stage1b_cloudship_rerun_20260927.txt,
stage2_20260927.txt (both candidates), samples_<target>_{chain10,chain10b,D,B}.csv;
patches in yaw_model/candidates/ (option_d_restore.diff, apply_restore.sh, apply_ff.py, README.md). They apply to 860ebca856; the 2026-09-28 architecture pass restructures airship_att_control, so a chosen candidate is re-implemented on the new structure rather than applied as is.

## Decision (user, 2026-09-28): P 0.7 and B, feedforward

"결정 두 가지에 대해 네 권고 대로 진행해": AS_YAWRATE_P 0.5 -> 0.7 and the release
fixed by feedforward, not by the restore rule.

- Decision 1 changes P only, so I stays 0.09 and INT_LIM 0.3. Candidate B had
  been measured at P 0.65 I 0.07; at P 0.7 I 0.09 the model had every plant
  but one already (options/B/final_table.txt, "per-plant FF, P 0.70 I 0.09"):
  all 40/40, three checks undecided, each also undecided or measured passing
  without FF (vec rigid releases, vec complete wind I off 1.14-1.18 against
  SITL 1.15 at P 0.7 alone). The missing one, cs complete at FF 0.05, was run
  2026-09-28 (options/B/ens3.jsonl): 40/40 on every check, none undecided
  (Acro peak 8.90-9.16 against 9.33, Stab release 6.0-6.6 s).
- Values: AS_YAWRATE_FF default 0; posix 2527 1.0, 2522 0.3, 2517 0.05; the
  rigid plants and every unmeasured variant (2521, 2523-2526, 2511-2516) and
  the flight airframes 0. The hold torque per rad/s at 6.67 deg/s (derive_ff)
  is 0.887 / 0.375 / 0.141 on the three.
- Implementation on the 25d2fecb83 structure: AirshipYawRateLoop::setGains(p,
  i, integrator_limit, feedforward) -> RateControl::setFeedForwardGain (the
  MC_YAWRATE_FF pattern), AS_YAWRATE_FF yaml (min 0, max 10, decimal 3),
  DEFINE_PARAMETERS, unit test FeedforwardHoldsTheTurnInsteadOfTheIntegrator
  (AirshipYawRateLoop 8/8), airframe comments (2520 flight, posix 2510) and
  the docs tuning order (FF between P and I). Tools: sitl_check.py now
  records P and FF in its setpoint line; bench_fingerprint.py records FF.
- SITL: sitl_out/chain12.sh -> chain12_20260928.txt (results below).

### SITL on the decision (committed d2647379af, 2026-09-28)

| check | chain12 | chain12b | chain12c |
|---|---|---|---|
| fixed flights (airship_fixed, _am_drag) | 13/13, 13/13 | - | - |
| 2520 rigid vectored, FF 0 | 10/10 | 10/10 | - |
| 2527 complete vectored, FF 1.0 | 10/10 | 8/10 (dropped push) | 10/10, 10/10 |
| 2522 vectored fin, FF 0.3 | 10/10 | 10/10 | - |
| 2510 rigid Cloudship, FF 0 | 9/9 | 9/9 | - |
| 2517 complete Cloudship, FF 0.05 | 8/9 (harness timing) | 9/9 | - |
| tilt range | 8/8 | - | - |

Every run's '지시 각속도' line records the gains it flew (P 0.7, I 0.09, FF as set).

- 2527: releases 7.9 / 8.2 s (model 7.9-8.5), wind I on integrator 0.0216 in
  every good run (model 0.0214-0.0220), wind I off 1.15-1.18.
- 2517 in chain12 failed the Acro peak (9.46 against 9.33) because of the
  harness, not the firmware: the triple wrench clear (gz_link, added the same
  day) cost 2.4 s of wall clock after the wind check, so the Manual check began
  at -14.3 instead of -21.7 deg/s (samples: push end -> Manual 3.56 s against
  1.17 s in stage 2) and every Acro check after it from another state. gz_link
  now sends the first clear only and the repeats from a thread; wrench() joins
  them before the next push. chain12b: Manual from -21.7 again, Acro peak 9.07
  (model 8.90-9.16).
- 2527 in chain12b lost its first wind push outright (0.02 -> 0.01 deg/s in
  5 s, gz topic rc 0) and its I-on case went wrong the same run (integrator
  0.0092, the push effect gone ~27 s in). chain12c ran it twice with every
  publish traced (GZ_LINK_TRACE=1): each gz publish takes 0.85 s wall, the
  repeat clears end >= 3 s before the next push, both runs 10/10 with the
  chain12 numbers. So a gz transport drop, which the first wind check exists
  to catch; the loop's torque followed P e + I throughout.
- Firmware: fw_d2647379af3 via build_fmu.py (metadata check passed) (build_fmu_d2647379af.log).
- Hardware, 2026-09-28 21:25: the user flashed fw_d2647379af3 (props off, ESC power off confirmed).
  `ver git` d2647379af, SYS_AUTOSTART 2520, AS_YAWRATE_P 0.7 and AS_YAWRATE_FF 0.0 as defaults (not
  saved), 1079 parameters used (1078 before). Manual fingerprint fpJ (head_20260928b/): 104 values
  0.000 us against fpC (9535f8abcd) and fpH (25d2fecb838), Manual in all 13 holds, params differ in
  exactly AS_YAWRATE_P and AS_YAWRATE_FF. COM_DISARM_PRFLT read back 10 (unsaved default), disarmed.
  The closed loop itself (P 0.7) is not measured at the pins: on 2520 its output goes through the
  vectored allocation, for which no bench prediction exists.
