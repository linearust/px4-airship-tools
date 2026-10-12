# D1 increment 1: implementation plan (2026-10-07)

Written by the design role (Fable 5.1) for the execution role (Opus 5.5). Design: `d1_universal_architecture_20261001.md`
as amended on 2026-10-07 (section "0단계 결과 반영" and the dated bullets). Evidence: `step0_report_20261002.md`,
`yaw_model/step0c_out_20261001.txt`, `sitl_out/forward_results_20261001.md`. Base: `airship/dev` bccba2ef8c (the 10-07
sync). Report back in `d1_increment1_report_<date>.md` + a memory note; the user reviews the diffs before anything
is pushed.

## 0. Rules that bind every step

- Controller only. `ActuatorEffectivenessAirship*`, `AirshipPod*`, `ControlAllocator*` stay byte-identical; the
  allocator identity harness (`identity/`, 12 sets vs `identity/dump_789e64eb13_12sets.txt`) is the tripwire.
- Every new parameter defaults to OFF so that the firmware at default parameters is bit-identical to bccba2ef8c:
  same torque, thrust, rates setpoint and rate_ctrl_status on the same inputs. Flight airframes 2500/2507/2520 get no
  new values; only the posix gz twins do (step 1d).
- Existing parameter names and meanings are frozen (AS_YAWRATE_P/I/FF/MAX, AS_YR_INT_LIM, MAN_DEADZONE, CA_*).
- No business context in code, commits or docs ([[upstream-privacy-boundary]]). Docs: edit only `docs/en`.
- Commits: conventional format, local, unsigned, no `-s`, trailer `Assisted-by: Claude:claude-opus-5-5` above nothing
  else; never `Co-Authored-By` naming an AI. `make format` on changed C/C++. Do not push.
- SITL runs through `gz_run.sh` (it now exports `PX4_PARAM_SDLOG_DIRS_MAX=0`). Bench runs only with the user's
  "프롭 제거, ESC 전원 차단" confirmation; the user flashes `build_fmu.py`'s staged image.
- No fan-out workflows for mechanical steps. A review workflow is justified once, at the end, over the whole diff.

## 1. The semantics to implement (normative)

Units: yaw rate r in rad/s; torque normalized (−1..1 per axis); FF and k in normalized torque per rad/s, k per m/s
of forward speed; speeds in m/s; the loop runs on every gyro sample as today.

### 1.1 Speed selector (`airship_speed_selector.hpp`, pure header, no uORB; same style as `airship_manual_input.hpp`)

Inputs each cycle: the latest `airspeed` sample (true_airspeed_m_s, confidence, timestamp_sample), the latest
`vehicle_local_position` (vx, vy, heading, v_xy_valid, dead_reckoning, heading_good_for_control, timestamp), now, dt,
and whether the yaw loop is closed. Parameters: mask AS_SPD_SRC (bit 1 airspeed, bit 2 ground), W = AS_SPD_WMAX,
dq = AS_SPD_DP_TOL.

1. Usable: airspeed if bit 1, finite, confidence > 0 and age < 1 s; ground if bit 2, v_xy_valid, not dead_reckoning,
   heading_good_for_control, finite and age < 1 s. Age is `now - timestamp_sample` (resp. `timestamp`) measured every
   cycle, so a topic that stops publishing ages out without a new message.
2. Return hysteresis: a source that became unusable is accepted again only after it has been usable for 1.0 s
   continuously (code constant). Loss is immediate.
3. Intervals (TAS): airspeed `[sqrt(max(0, q − dq)·2/ρ), sqrt((q + dq)·2/ρ)]` with q = ½ρ·TAS², ρ = 1.225 (code
   constant; the TAS is already the sensors module's); ground `[max(u_g − W, 0), max(u_g + W, 0)]` with
   u_g = vx·cos ψ + vy·sin ψ (NED to body x through the EKF heading).
4. Both usable: the intersection; if it is empty, drop the airspeed for this cycle and use the ground interval
   (a blocked tube, wash or a zero error reads either high or low; the ground interval cannot).
5. Filter: both bounds through a first-order low-pass, τ = 1.0 s, state starts at 0, no reset branch
   (`y += (x − y)·min(dt/τ, 1)`).
6. All lost: the lower target is 0 (decays through the filter). The upper target holds its last value while the yaw
   loop is closed and becomes 0 when the loop is open ("held" flag reported).
7. Output struct: u_lo, u_hi (filtered), source (0 NONE, 1 AIRSPEED, 2 GROUND, 3 BOTH), raw airspeed and raw u_g
   (NaN when unusable), held.

### 1.2 Shared authority (`src/lib/control_allocation/actuator_effectiveness/AirshipYawAuthority.hpp`, pure)

`yawAuthority(airframe, pod_grouping, tail, tilt_min, tilt_max, reverse_bits)` returns the at-rest normalized yaw
authority per direction: independent pods `0.5·(1 + ρ_rev)` where ρ_rev = 1 if the tilt range reaches ±180° (either
pod can point backward), else 0 (so ±180° → 1.0, ±90° or fixed → 0.5); tail thruster → 1 in the direction it pushes
and 1 in the other only with its CA_R_REV bit, else 0; no propulsive yaw → 0. Collective thrust is deliberately not
in A (design 2.나, 2026-10-07). Increment 2 adds the credited surfaces here. Unit-tested in the lib's test dir.

### 1.3 Schedule (`airship_yaw_schedule.hpp`, pure function)

Inputs: FF0 = AS_YAWRATE_FF, MAX = radians(AS_YAWRATE_MAX), INT_LIM = AS_YR_INT_LIM, k = AS_YR_FF_SLP, LO/HI =
AS_SPD_LO/HI, A (per direction; use the smaller of the two for the cap when they differ, and record both),
u_lo, u_hi, and `physical` (the torque is in physical units: no yaw surfaces declared; increment 2 adds CS_V > 0).

- If k ≤ 0 or !physical: return {FF0, MAX} exactly, no arithmetic (bit-identity path).
- w(u) = constrain((u − LO)/(HI − LO), 0, 1); if HI ≤ LO treat as a step at LO.
- FF_s = FF0 + w(u_lo)·k·u_lo.
- ρ = 0.5·min(INT_LIM, A/2) (0.5 is a named constant `kIntegratorReserve`).
- cap = MAX; if FF_s > 0: cap = min(cap, (A − ρ)/FF_s); d = k·u_hi − FF_s; if d > 0: cap = min(cap, ρ/d).
  No blend toward MAX; no stall term (design 2.다, 2026-10-07).
- Output {FF_s, cap, A, ρ}.
- Worked values for the tests (2527 posix: FF0 1.0, k 4.01, INT_LIM 0.3, A 1, MAX 15°, LO 1.5, HI 2.5):
  u = 0: {1.0, 15°}; u_lo = u_hi = 4: FF_s 17.04, ρ 0.15, cap = 0.85/17.04 = 0.04988 rad/s = 2.86°/s;
  u_lo = 0, u_hi = 2 (ground-only outdoors at rest, W 2): FF_s 1.0, d = 7.02, cap = 0.15/7.02 = 0.02137 rad/s =
  1.22°/s; u_lo = u_hi = 2: w 0.5, FF_s 5.01, cap = min(0.85/5.01, 0.15/(8.02 − 5.01)) = min(0.1697, 0.0498) = 2.86°/s.
  FF0 = 0 (flight airframes), u_lo = u_hi = 4: FF_s 16.04, cap = 0.85/16.04 = 3.04°/s.

### 1.4 Where the cap applies (both, design 2.다)

- `yawRateSetpoint(stick, deadzone, cap)` — the stick's full scale is the cap (the whole stick stays useful).
- `yaw_rate_sp = constrain(yaw_rate_sp, −cap, +cap)` before `_yaw_rate_loop.update()` (the rover's placement; every
  future setpoint source is limited too). At cap = MAX this is the identity.

### 1.5 Feedforward per cycle

`AirshipYawRateLoop::setFeedForwardGain(ff)` (new, forwards to RateControl::setFeedForwardGain for the yaw axis only)
called every cycle with FF_s; `setGains()` unchanged for P, I, limit.

### 1.6 Restore rule (`AS_YR_RESTORE` = 1)

Exactly options/D's rule as `yaw_model/policies.py` implements it (edge `zero_or_sign`): keep `I_saved` (0 when the
loop opens). With s = sign of the deadzoned stick (−1/0/+1): on 0 → ±1 save I_saved = integrator; on ±1 → 0 set
the integrator to I_saved; on ±1 → ∓1 set the integrator to I_saved and then save again. Needs
`RateControl::setIntegral(const matrix::Vector3f &)` in `src/lib/rate_control` (one setter, clamped to ±lim_int,
unit-tested there) — the only change to that lib. When AS_YR_RESTORE = 0 nothing runs (bit-identity).

### 1.7 Message and logging

`msg/AirshipSpeed.msg`: timestamp, u_lo, u_hi, airspeed_raw, ground_raw [m/s], ff, rate_cap [rad/s], authority,
source (uint8 enum as 1.1), held (bool). Published every cycle in Run() in every mode, before the wrench, from the
selector's output and the schedule's result (FF0/MAX when off). Logger: `add_optional_topic("airship_speed", 100)` in
`logged_topics.cpp` (default profile, 100 ms as rover_rate_status).

### 1.8 Parameters (`airship_att_control_params.yaml`; names final, all reboot_required false)

| name | type | unit | default | min | max | meaning |
|---|---|---|---|---|---|---|
| AS_SPD_SRC | int32 bitmask | - | 0 | 0 | 3 | bit 0 airspeed sensor (TAS), bit 1 EKF body-x ground speed; 0 = no speed source |
| AS_SPD_WMAX | float | m/s | 2.0 | 0 | 10 | wind bound around the ground speed |
| AS_SPD_DP_TOL | float | Pa | 1.05 | 0 | 20 | dynamic-pressure tolerance of the airspeed sensor (one MS4525DO count; SDP3x zero error) |
| AS_SPD_LO | float | m/s | 1.5 | 0 | 10 | FF ramp start |
| AS_SPD_HI | float | m/s | 2.5 | 0 | 10 | FF ramp end (> LO) |
| AS_YR_FF_SLP | float | 1/(m/s) | 0 | 0 | 50 | k: yaw feedforward per m/s of forward speed; 0 = schedule off |
| AS_YR_RESTORE | int32 boolean | - | 0 | 0 | 1 | restore the yaw integrator saved when the stick left the deadzone |

Existing names keep their roles: AS_YAWRATE_FF = FF0 (at-rest FF, max 10 stays), AS_YAWRATE_MAX = the cap's
maximum and the at-rest full scale, AS_YR_INT_LIM = integrator limit and ρ's input.

## 2. Model first (cheap, before any firmware)

1. Update `yaw_model/policies.py` SCHED to the corrected formulas (1.3: FF_s-based interval term, ρ fraction 0.5, no
   blend, no stall term) and keep RESTORE as is; keep the old variant available under another name for comparison.
2. Re-run `policy_compare.py` for HEAD, SCHED, SCHED+RESTORE on the gate legs only (2527, 2522 at 2.5/4/6 m/s, full
   and half stick, 20 s and 300 s, sources perfect / ground_W0 / ground_W2_head / pitot_bias_ms, k x1 / x0.7 / x2).
   Record whether the corrected SCHED+RESTORE keeps the 0c result (release ≤ 2.5° in all, ≤ 3° excursion) and what
   the new caps are. If anything regresses, stop and report before writing firmware.
3. Produce the acceptance table for the SITL legs of step 3: plants 2527 and 2522 (k 4.01 from the files),
   2512 (k from a held-turn measurement in the model, ~12.7), at 2 and 4 m/s full stick and 4 m/s half stick, with
   SCHED+RESTORE and the posix values of 1d: expected r over the stick's last 5 s, integrator at release, heading
   change over the 120 s release, r at +10/+60/+120 s. Tolerances: forward_replay's (r 3 % or 0.1°/s, headings
   5 % or 2°, integrator 0.005).

## 3. Firmware steps, each with its gate

- **1a** `AirshipSpeed.msg`, logger line, `airship_speed_selector.hpp` + gtest (NaN inputs, ageing without new
  messages, confidence 0, mask bits, heading_good / dead_reckoning, wrap and reverse flight, dq intervals at 0/2/4 m/s,
  intersection and the empty-intersection drop, return hysteresis, decay that never stops, upper bound held only while
  the loop is closed). Gate: tests, `make px4_sitl_default`, `build_fmu.py`.
- **1b** Wiring: subscribe `airspeed` and `vehicle_local_position`, run the selector every cycle in every mode,
  publish `airship_speed` before the wrench. Gate: a scratch controller identity harness (script modes, sticks, gyro,
  speed; dump torque, thrust, rates setpoint, rate_ctrl_status as hex floats; `identity/idcmp.sh` against the same
  harness built at bccba2ef8c; never committed) IDENTICAL with and without speed topics; `make tests
  TESTFILTER=Airship`; SITL 2527: switch between GNSS ground speed and `SENS_EN_ARSPDSIM` pitot, inject off / stuck /
  high, lose GNSS, and read the topic in the ULog; `sitl_check.py` 48/48 on the five plants; bench fingerprint
  0.000 µs (user flashes).
- **1c** `AirshipYawAuthority.hpp` + test; `airship_yaw_schedule.hpp` + test (the worked values of 1.3, k = 0
  bit-identity, continuity and monotonicity of the cap in u, ρ for INT_LIM 1.0 and A 0.5, the physical guard);
  `setFeedForwardGain`; `RateControl::setIntegral` + test; the restore rule + test (the three edges, loop-open clear);
  the cap at both places + test; the seven parameters. Gate: unit tests; controller identity IDENTICAL at defaults and
  with the parameters on but AS_SPD_SRC 0; allocator identity 12/12; `sitl_check` 48/48; `airship_fixed_flight_check`
  13/13 on 2500 and 2505; `forward_yaw_check.py` on 2522/2527/2512 at the posix values against the step-2 table
  (plus: integrator never at the limit, cap reached, same with the speed source lost mid-turn); 2520/2510 (no fins,
  k 0) unchanged; bench fingerprint 0.000 µs.
- **1d** posix values: 2527 and 2522 `AS_SPD_SRC 2, AS_SPD_WMAX 0, AS_YR_FF_SLP 4.01, AS_YR_RESTORE 1`; 2512/2517 the
  measured k; flight airframes untouched. Docs (`docs/en/frames_airship/index.md`): replace the six "no airspeed
  scaling" sentences, the pitot configuration (SDP3x, CAL_AIR_CMODEL, mounting outside the pods' wash), the FF
  measurement procedure at two speeds (k from a held turn: yawspeed_integ / rate), AS_SPD_WMAX guidance, the at-rest
  cap outdoors without a pitot. Gate: the step-2 predictions pass in SITL; `variant_check.py`; docs build has no dead
  links.
- **Tools** (additive): `forward_yaw_check.py` records `airship_speed` (u_lo, u_hi, ff, rate_cap, source) per leg
  and adds a verdict "the stick held the cap" when the parameters are on; `sitl_check.py` unchanged.

## 4. Commit plan (one reviewable commit per item)

1. `feat(msg): add airship_speed, the airship controller's forward-speed interval`
2. `feat(airship_att_control): select a forward speed interval from airspeed and ground speed` (+ tests)
3. `feat(airship_att_control): publish airship_speed every cycle`
4. `feat(rate_control): add setIntegral` (+ test)
5. `feat(control_allocation): airship yaw authority per direction` (+ test)
6. `feat(airship_att_control): schedule the yaw feedforward and cap the yaw rate by forward speed` (+ restore rule,
   parameters, tests)
7. `feat(airframes): speed schedule values for the gz airship twins`
8. `docs(frames_airship): forward-flight yaw control, the airspeed sensor and the FF measurement`

## 5. Report

`d1_increment1_report_<date>.md`: per step the gate results with numbers, the SITL table against the step-2
predictions, the identity results, the bench fingerprint, flash/CPU deltas (`perf` on the FMU, image size), and
every deviation from this plan with its reason. Then a memory note (project) pointing at it, and stop for the user's
review of the commits.
