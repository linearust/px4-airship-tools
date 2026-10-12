# Design review of the generic airship family (2026-09-25)

Seven dimension reviewers, a completeness critic with three gap reviews, and three skeptical verifiers per
finding (survives on 2 of 3). 207 agents. 48 findings confirmed, 17 refuted. The findings below are left as
written; the resolution table records what became of each.

## Hardware (Pixhawk 6X, 9535f8abcd, props off, ESC power off; records in head_20260925/)

| test | result |
|---|---|
| T0 image, ROMFS, airframe, allocator | 9535f8abcd; 2500/2507/2520, no 2510; SYS_AUTOSTART 2520; 8 actuators |
| run C, 2520 fingerprint vs run B (2510) | 0.000 us, 13 x 8 |
| T1, 2500 allocation live, Manual fingerprint | 0.000 us vs preset2500 runC; twin signs and pairing hold; motors throttle-only |
| T2, 2500 allocation, CS_K 1, Stabilized yaw | half stick 1532 -> 1679 us at 5.23 us/s (pred 1529/1679/5.24; 1532.5 when the start is predicted at the middle of its window, as `bench_rate_hold.py --loop` does since 2026-09-27); released 1650 us held; full stick 1715 us (43 %); Manual resets to 1500 |
| restore | store = pre-test except runtime values; fingerprint E 0.000 us vs run C |
| .px4 metadata | airframe block lists 2500, 2504, 2507 (airframes.xml from 2026-08-06; build rule depends only on the generator) |

After the fixes (2026-09-27, 1965de11ab flashed by the user, props off, ESC power off; records in head_20260926/):

| test | result |
|---|---|
| T0 image, ROMFS, airframe | 1965de11ab; 2500/2507/2520; SYS_AUTOSTART 2520; parameters equal to the pre-test baseline but for 3 runtime values |
| .px4 metadata | Airship group 2500, 2507, 2520, names equal to the ROMFS (build_fmu.py); 44 non-airship ids advertised without a file in this image (upstream's generator, counted on 2026-09-27) |
| run F, 2520 Manual fingerprint vs run C | 0.000 us, 13 x 8, parameters identical |
| T2 again, 2500 allocation, CS_K 1, Stabilized yaw | half stick 1750 us held 35 s at 0.00 us/s; released 1500 us; full stick 2000 us; Stabilized equals Manual (mode confirmed from the heartbeat); motors idle |
| restore | CS_K 0.5, GRP 1, TLMIN -180, TLMAX 180, reboot; parameters equal to the pre-test snapshot but for 3 runtime values |
| run G, restored bench vs run C | 0.000 us, 13 x 8, parameters identical |

After the architecture pass (2026-09-28, 25d2fecb83 flashed by the user, props off, ESC power off; records in head_20260928/):

| test | result |
|---|---|
| T0 image, airframe | 25d2fecb83; SYS_AUTOSTART 2520; parameters equal to the previous snapshot but for 3 runtime values |
| run H, 2520 Manual fingerprint vs run C | 0.000 us, 13 x 8, parameters identical (refactored bench scripts) |
| T2, 2500 allocation, CS_K 1, Stabilized yaw | 1750 us held, 1500 us released, 2000 us full; passthrough, mode confirmed |
| restore | parameters equal to the pre-test snapshot but for 3 runtime values after a reboot |
| run I, restored bench vs run C | 0.000 us, 13 x 8, parameters identical |

## Resolution (2026-09-25, night)

After the user's delegation of the four decisions (D1 surface-only yaw keeps the stick as torque; D2 one hull
volume and the tail redesigned on it; D3 the classic Cloudship stays at 2519; D4 no upstream PR or issues now,
tooling for the metadata). A closure pass mapped every finding against the tree; the open ones were then fixed
or decided, and a second review of the final diff (4 reviewers, 2 skeptics each) confirmed 11 more low-severity
defects, all fixed. 37 fixed, 9 decided, 1 moot, 1 open.

| # | finding | state | how |
|---|---|---|---|
| 1 | Fin stall basis | fixed | Stall at the plate CLmax 0.7 over slope x K_W(B): 0.1567/0.1649 rad; post-stall to cl 0, cd 1.2 at 90 deg (rev. 3). |
| 2 | 2500 yaw integrator winds at rest | fixed | D1 firmware gate: surface-only yaw keeps the stick as torque in every mode. SITL leg 4b: 5.5 deg held, 0.0 after release. |
| 3 | 2500 turn runs on I/Ki after release | fixed | Same gate; no loop, no integrator on 2500. |
| 4 | 2520 finned plants: loop cannot reach its rate in forward flight | decided | D1: the speed-aware loop is the next increment; the envelope is in the design note. |
| 5 | No check asserts the design requirement | fixed | variant_check fact 14: Helmbold restoring / Munk >= 1.2 from the files (1.203). |
| 6 | Post-stall and broadside forces | fixed | cl 0 and cd 1.2 at 90 deg on every fin; the plugin's reverse-flow switch is stated. |
| 7 | Flap linear to 25 deg | fixed | Flaps +-11 deg (R3), servo range held to the joints by fact 15 (since 2026-09-27 also every flap joint driven by exactly one servo). |
| 8 | 2507 -> 2519 lost its reason | decided | D3: 2519 kept for one decade per vehicle; reason restated in the tree and the naming note. |
| 9 | AS_* defaults cap the 2500 rudder at 43 % | fixed | D1 gate. |
| 10 | Stale airframe metadata in the .px4 | decided | D4: tooling, not a tree change. build_fmu.py deletes airframes.xml and the .px4, checks the image, and stages it under its own commit for the uploader (not `make ... upload`, whose rebuild can repack it around a stale list). |
| 11 | Acro tracking inherits the brake phase | fixed | sitl_check.py: each tracking step waits for rest (bounded) and clears the integrator through Manual; the start state is printed, and since 2026-09-27 a start that did not settle within 60 s or was not read fails the step. |
| 12 | 2500 keeps the vectored gains | fixed | D1 gate makes them unused on 2500; the parameter texts say so. |
| 13 | The 2500 twin cannot hold a turn | fixed | Docs and posix 2500 say why (fins are the only yaw moment; Munk on the _am members) and that the elevator makes heave; flight check leg 6b holds full rudder 10 s. |
| 14 | Forward-flight yaw envelope unstated | decided | In the design note (5.1/3.3/2.2 deg/s at 2.5/4/6 m/s, attached from 2.5 m/s, hysteretic stall). A forward-flight gate case comes with the D1 speed-aware loop. |
| 15 | Two hull volumes | fixed | D2: one 39.2 m^3 spheroid drawn, collided and sized on. |
| 16 | Bare name is the least physical plant | decided | Names frozen. Docs: nothing on the rigid hull limits speed; the Complete member is the one to fly. |
| 17 | 2519 breaks the upstream pairing | decided | D3. |
| 18 | Upstream files promise enforcement by private tools | fixed | Model headers state the rule ("kept identical") without naming variant_check.py; the 2510_gz_cloudship and 2507_cloudship comments no longer cite a SITL gate result (2026-09-27). |
| 19 | "_am must be a full copy" premise | fixed | Text now says a plain merge include cannot change the inertial; experimental:params noted, not adopted. |
| 20 | 30/40 % rate tolerance admits a plant with no Munk moment | fixed | RATE_TOL 10 %, held-leg floor 0.1 deg/s. Since 2026-09-27 the model carries the plugin's fin cl and cd: the records, replayed on the cda they were flown with, match at onset within 2.5 % (yaw) and 4.5 % (pitch); without the Munk moment it predicts 21 % less yaw rate and 11-12 % less pitch rate. |
| 21 | Gate "insensitive to surge" premise | fixed | Docstring states the _fin plants' dependence on drift and that the wind expectation leaves the fins out. |
| 22 | Fin added mass and the Munk coefficient | fixed | The wrong claim is gone. |
| 23 | Rate incidence at c/4; carryover on roll | fixed | Documented in the design note (about 12 % less yaw damping; roll damping overstated). |
| 24 | Comments false for 3-4 commits | decided | D4: history is re-cut when it goes upstream. |
| 25 | Paragraphs added then deleted | decided | D4, as above. |
| 26 | ac41e6c 1.20 m span claim | moot | That sizing no longer exists. |
| 27 | 6e74f20 quotes a superseded run | fixed | Re-run with the corrected reader (chain5, chain6). |
| 28 | Headers name a private tool | fixed | As 18. |
| 29 | Submodule commit scope and subjects | decided | D4, as 24; new commits use imperative subjects. |
| 30 | Yaw-inertia figures and the added-mass gap | fixed | 2510 and 2520 comments give vehicle inertia and name added mass (192 kg m^2; 321 total). |
| 31 | Naming note table stale | fixed | airship_family_naming.md brought to the built family; the lines that pass missed (the classic twin's posix number, the 2520 flight file, the 2026-09-23 revision's numbers, the bench renumber) followed on 2026-09-27. |
| 32 | Cloudship cd at 90 deg | fixed | cd 1.200 at 90 deg. |
| 33 | Cloudship fin gate "unchanged" claim | fixed | Measured: _fin 9/9, _am_fin_drag 8/9 as on 2026-09-24. |
| 34 | Check says Helmbold, files use the plate slope | fixed | Docstrings name each slope where it is used. |
| 35 | CoV 5 mm ahead of the CoM | fixed | Gondola at x 0.3277; CoM and CoV coincide. |
| 36 | Stale Cloudship cd comment | fixed | As 32. |
| 37 | Flap cl added after the stall clip | fixed | Travel +-11 deg keeps full rudder 2 deg below stall on the plugin's own semantics. |
| 38 | Ventral fin below the ground contact | fixed | Ventral 1.28 m, tip 5.3 cm above at level attitude (G1, stated as a drawing constraint). |
| 39 | World wind reaches the fins only | fixed | Stated in the docs and the design note. |
| 40 | Fin colours dropped | fixed | Materials restored. |
| 41 | Mask rule stated as universal | fixed | Docs, CMakeLists and 2510 state the no-fin-bit exception; 2505 is "Generic Airship (Complete)". |
| 42 | Wrong v1.18 badge on independent vectoring | fixed | Badge is "main (PX4 v2.0)". Section images not added. |
| 43 | lta floats instances 0-3 only | fixed | worlds.md says so. |
| 44 | Naming design record out of date | fixed | As 31, with the 2519 reason. |
| 45 | Yaw fit leaves out roll-coupled terms | open | Test-method bias of 3-5 %, inside the check's tolerance; left for the added_mass_check rework. |
| 46 | simulate() misses the added-mass coupling | fixed | Coupling term added; the added-mass members' yaw onsets then matched within 0.6 %, and within 2.1 % once the plugin's fin drag and stall joined the model (2026-09-27). |
| 47 | Post-stall drag 1.5-1.7x its basis | fixed | cd 1.200 at 90 deg; fact 4 checks the stall parameters, the Cloudship's too since 2026-09-27 (low-aspect-ratio plate CLmax 1.25). |
| 48 | Twin allocation block is a copy | fixed | variant_check fact 16 holds each gz twin's CA_* block equal to its flight airframe's. |

On 2026-09-27 the generic fins' cda was re-sized at the lower of 10 deg and the stall, where the plugin switches to
cda_stall (dorsal and side 0.9133 -> 0.8201, ventral 1.0379 -> 0.9805; cd at 90 deg stays 1.200). The generic fin
plants' SITL figures in the rows above were flown on the earlier values; the 2026-09-27 run on the current fin drag (sitl_out/chain9_20260927.txt, records in sitl_out/fixed_flight_20260927_chain9/) passed all four flight twins 13 / 13, the held turns within 0.02 deg/s of the model (rigid -0.03 vs +0.01, drag +0.15 vs +0.17, am +1.89 vs +1.90, am_drag +1.90 vs +1.91 deg/s at the last second's speed), and the added-mass check 9 / 9; the gates kept their baselines: Cloudship 9 / 9, vectored rigid 9 / 10 (the known Acro brake, 27.7 -> -7.6 deg/s), vectored _fin and _am_fin_drag 8 / 10 (release 1.0 and 2.7 deg/s still falling at the window's end), Cloudship _am_fin_drag 8 / 9 (Acro peak 9.54 on 6.67 from rest).
The Cloudship's fins did not change.

## High

### Fin stall basis contradicts the cited data and the family's own stall rule, and hides a loss of directional control above about 17 deg of rudder
`PX4-Autopilot/Tools/simulation/gz/models/airship_fixed/model.sdf:426` (aero-basis)

All generic fins carry alpha_stall 0.3 and cla_stall -1.35, so cl reaches 1.92 on the exposed area at 17.2 deg.

The same lines are in airship_vectored_independent_fin/model.sdf:51-52, _am_fin:52-53 and airship_tail_design.md:63-64,83.

What the cited source says: Ananda, Sukumar and Selig 2015, the design's own slope source, report that the A-3 rectangular flat plate stalls at 'alpha ~ 10 deg'. They put CLmax 'for all models tested in the range of 0.55 to 0.7', with post-stall lift 'relatively constant ... up to 30 deg'. The note quotes the 10 deg (line 63), then uses 17 deg anyway.

What the Cloudship commit 08be662 says: its rule is that a plate's own CLmax times the carryover caps the linear range. Applied here that gives about 0.7 x 1.746 = 1.22, i.e. alpha_stall about 0.19 rad. The file carries 1.57x that, and 2.7x the plate's CLmax. The body upwash that produces K_W(B) 1.27 also makes the panel stall earlier in body angle, not later.

Consequence, from a 2-DOF sway-yaw integration of the file's numbers with u held:
- On the file's curve, a full-rudder (25 deg) steady turn sits at 15.2-15.9 deg fin incidence, speed-independent. That is 90 % of the model's stall.
- With the stall on the cited data (cl capped at 1.22), the same plant departs above 17-18 deg of rudder: beta_cg goes to 34-50 deg and r to 27-41 deg/s instead of 5-6 deg/s. At 3 m/s with hull drag, beta reaches 22 deg in 5 s.
- The departure threshold moves to 21-22 deg only if CLmax is 1.5, and above 25 deg only if CLmax is 1.75 or more.
- Static directional stability in pure sideslip ends at about 25 deg on the file's curve and at about 16 deg on the data.

The gz twin of the 2500 airframe therefore cannot show a loss-of-directional-control mode that the design's own sources predict inside the rudder travel. The flight check's 2.3 s yaw windows never reached the steady turn, so nothing measured has exercised it.

Correct choice: alpha_stall about 0.19 rad (cl about 1.2), a near-flat post-stall to about 30 deg, then falling to 0 at 90 deg. Alternatively, keep 0.30 only with a stated source for an AR 2.8 plate stalling at 17 deg.

### 2500 in any rate mode: the yaw integrator winds to AS_YR_INT_LIM at low airspeed with no anti-windup signal, then turns the hull as airspeed builds; the comment names the cause but not this effect
`ROMFS/px4fmu_common/init.d/airframes/2500_generic_airship:32` (firmware-impact), hardware-testable

Where it happens: in Acro, Stabilized, Altitude and Position, airship_att_control_main.cpp:216-220 closes the yaw PI whenever armed.

Why nothing stops it on 2500: the rudders (CS2/CS3 TRQ_Y 0.5) get delta = u from the pseudo-inverse. With CA_AIRSHIP_CS_K 1 they are credited in full (ActuatorEffectivenessAirship.cpp:172-179). On a surface-served axis the yaw row then publishes the allocator's own residual (cpp:297-300), which is zero until the surfaces clip at |u| > 1. The loop output can never reach that: u <= P*AS_YAWRATE_MAX + AS_YR_INT_LIM = 0.5*0.2618 + 0.3 = 0.431. So updateSaturationStatus() never sets a yaw flag, and the integrator runs freely at zero airspeed.

Computed from the code path:
- At rest, the integrator reaches 0.3 after 12.7 s at full stick and 28.6 s at half stick. That leaves 7.5 deg of rudder standing after the stick is released.
- A 3-state model of airship_fixed_am_drag (the flight check's own formulation) then throttles to 3 m/s with the stick centred. Uncommanded yaw peaks at 1.2 deg/s. Heading moves +55 deg in 60 s and +77 deg in 90 s, and the integrator is still 0.178 at 90 s. With no windup there is zero drift.
- The same happens on every approach and slowdown, not only on the ground. D7 judged the exposure bounded because the allocator anti-windup covers missing authority in flight; on 2500 that assumption does not hold.

What the recent work left out: the posix twin says 'Fly it forward first.', but the flight file only describes the credit. airship_fixed_flight_check.py arms and flies only in Manual (line 494), so the twin built to measure this never closed the rate loop.

Correct choice:
- At minimum, the 2500 comment and docs should state the consequence and the operating restriction: Manual until the rudders have airflow.
- The PX4 precedent for a surface-driven rate loop is fixed-wing: it resets its integrators whenever the vehicle is not in fixed-wing flight (FixedwingRateControl.cpp:304-307) and scales its output by airspeed squared (line 397). The airship needs an equivalent gate for the surface-only yaw axis, or 2500 must keep the yaw passthrough the way FW_ACRO_YAW_EN=0 does.

Pin test: switch the bench FMU to 2500 and arm in Stabilized with props off (the still FMU is exactly the zero-airspeed plant). Hold half yaw stick: the rudder outputs should ramp from about 1529 us toward about 1679 us at about 5.2 us/s. After release they should hold about 1650 us while control_allocator_status reads torque_setpoint_achieved.

### On 2500, the rate modes ask for a yaw rate the rudders cannot reach, so most of any turn happens after the stick is released: exactly I/Ki (up to 191 deg)
`src/modules/airship_att_control/airship_att_control_main.cpp:97` (gap:The 2500 path in rate modes: never run in SITL or at the pins), hardware-testable

**Setup.**
- 2500 has no AS_YAWRATE_* of its own, so it inherits the vectored airship's defaults: P 0.5, I 0.09, I_LIM 0.3, MAX 15 deg/s.
- Gains are set at airship_att_control_main.cpp:97-101 with no feedforward. RateControl::setFeedForwardGain exists but is never called.

**The rudder never reaches the commanded rate.**
- On the twin's numbers, full rudder holds 3.4 / 5.1 / 6.8 deg/s at 2 / 3 / 4 m/s.
- 15 deg/s at 4 m/s would need a 15 m turn radius, 1.5 hull lengths.
- So any sustained stick saturates the integrator: at 3 m/s it reaches 0.3 in 14 s.
- The steady turn is then P*err + 0.3 = 0.41 of rudder: 2.0 deg/s, against 5.1 deg/s in Manual. This is the known 40 % / 43 % result.

**New: what happens on release.**
- The rudder never clips (at most 0.43 < 1), so no anti-windup flag is ever raised (rate_control.cpp:92-99).
- Inside the deadzone the setpoint is exactly 0, so dI/dt = -Ki*r.
- It follows exactly, whatever the plant does: heading change after release = I_release / Ki. That is 0.3/0.09 rad = 191 deg once saturated.
- Model time course at 3 m/s: 67 deg in the first minute, 111 deg by 2 min, 188 deg by 10 min.
- A 3 s full-stick blip at 3 m/s turns the hull 2.4 deg during the input and 42.5 deg after it. Manual gives 15.5 deg during and about 0 after.
- Full counter-stick takes 6.1 s to reverse the rate (0.2 s in Manual).
- At rest the same identity replays whatever the integrator stored once airflow arrives. That covers a stick hold (1650 us held at the pins) or the hull being walked round while armed in any rate mode.

**Scope.**
- Acro, Stabilized, Altitude and Position all use this law.
- Upstream's 2500 is a passthrough in every mode. The 0 us pin identity (preset2500) and airship_fixed_flight_check (:24) are Manual only.
- sitl_check.py cannot be pointed at this vehicle: it exits for anything not in VEHICLES (:41-58), and it turns the hull in place (:15), where 2500 has no authority.

**Correct choice.**
- 2500 sets and justifies its own AS_YAWRATE_MAX: one full rudder can hold, about 3-5 deg/s on the twin.
- Add a feedforward term so the stick reaches the rudder without the integrator. Precedent: FW_YR_FF (default 0.3) with airspeed scaling, FixedwingRateControl.cpp:389-392.
- Or give surface-only airframes an opt-out that restores passthrough. Setting P = I = 0 today zeroes yaw, it does not restore passthrough.
- Then gate it with a forward-flight rate check on airship_fixed_am_drag.

**What the pins can show.** The held 1650 us after a saturating hold, and that nothing else unwinds it. The heading run-on needs SITL at speed.

### On every finned vectored plant the 2520 yaw rate loop cannot reach its commanded rate in forward flight, winds its integrator to the limit with no saturation flag, and keeps the hull turning for minutes after release
`src/modules/airship_att_control/airship_att_control_main.cpp:164` (gap:Yaw control in forward flight on the vectored _fin members: fixed fins, no control surfaces, constant pod authority), hardware-testable

This defect is in firmware that already existed. ac41e6c exposed it but did not introduce it: its fins gave the vectored plant yaw damping of 193.6u N m per rad/s.

**Why the loop cannot reach the setpoint.** AS_YAWRATE_MAX is a constant 15 deg/s (airship_att_control_params.yaml:39-51). The loop's output is P*e + I, with P 0.5 on rad/s and the integrator capped at AS_YR_INT_LIM 0.3. At full stick that output peaks near 0.13 + 0.30 = 0.43 of normalized torque. The pods can deliver about 0.95, so the allocator never saturates.

**Why nothing stops the windup.** The only anti-windup is at lines 164-172: an integrator stops only when the allocator reports a shortfall. The loop's own torque cap is invisible to it. So when the setpoint is unreachable, the integrator sits at +0.3.

**Results.** I ported AirshipPod, shortfall()/saturationSign(), rate_control.cpp and the gz LiftDrag/Hydrodynamics terms onto the _am_fin_drag numbers, with the throttle held at trim.

At 4 m/s, full stick:
- The rate reaches 1.64 deg/s against 15 commanded, using 0.42 of the torque.
- The integrator hits the 0.300 limit at about 15 s.
- Unallocated yaw reads 0 except during the 1.5 s pod swing.

After release at 4 m/s:
- The hull still turns at 1.13 deg/s 2 s later.
- The rate stays above 0.25 deg/s for 250 s.
- The heading changes a further 99 deg in 2 min and 161 deg in 5 min.
- The time constant is (1+0.5k)/(0.09k) ≈ 175 s, with k = 0.0657 rad/s per unit of torque.

At 2 m/s: 3.05 deg/s during the stick, 2.14 deg/s 2 s after release, and 146 deg of drift in 2 min.

Half stick at 4 m/s: 0.87 deg/s against 6.7 commanded, and 59 deg of drift.

For comparison, in hover the same input settles in 25-39 s. That is the known braking failure the user accepted. Forward flight makes the drift last about ten times longer and makes the setpoint unreachable at every speed above about 2 m/s.

Acro and Stabilized share this path, because getControlMode(Attitude) sets rates_enabled and yawRateLoopActive gates on it. Stabilized therefore has no heading hold that would catch the drift.

**Correct fix, a firmware follow-up needed before a finned 2520 flies forward.** Make the rate loop speed-aware:
- limit the rate setpoint to what the pods can hold at the current forward speed, about 0.8 x 0.25/u rad/s for this plant;
- add a speed-scheduled feedforward, about N_r(u)/54 per rad/s, so that P and I carry only trim.

fw_rate_control's airspeed scaling is the in-tree precedent. In the same model this policy:
- doubles the turn at 4 m/s (1.64 to 3.09 deg/s);
- stops the turn within 2 s of release (drift 99 deg to 0 deg);
- keeps the integrator at about 0.

**Options that do not fix it:**
- Raising AS_YR_INT_LIM makes the post-release turn worse.
- Rudders do not cure the windup: the airship_fixed twin under the same 0.42 cap turns 2.7 deg/s at 4 m/s and would hold about 1.9 deg/s after release.

**What the bench can confirm.** The Pixhawk can confirm the controller side, not the plant side. With the FMU on the desk (2520, Acro, throttle low, props off), hold full yaw for 15 s. Port motor and starboard motor should both sit near 0.43, with the starboard tilt at +180 deg. After release they should hold about 0.30 indefinitely, because the gyro reads zero and the integrator never unwinds.

### No check asserts the design requirement; a statically unstable design passes every check
`px4-airship-bench-tools/added_mass_check.py:605` (verification-method)

The requirement is static directional stability, with fin restoring moment at least 1.2 x Munk on Helmbold's slope (airship_tail_design.md:11-22). No check tests it:
- added_mass_check.py:605-611 passes when |K_s(_am_fin) - (fin - munk)| <= 0.15 munk. Both terms come from the SDF, so the check is file against engine. The Cloudship proves this: its fins restore 0.50 of its Munk moment, it is statically unstable, and its net term reads -2.22 for an expected -2.42. It passes 9/9 (chain_20260924.txt:14-25).
- variant_check fact 4 (variant_check.py:379) holds cla equal to fin_basis.cla, a transcription check. Its docstring at :220-224 still says 'Helmbold's slope' while the code uses the plate slope. No fact computes the restoring/Munk ratio.
- The ratio exists only as a print in fin_basis.py:92-109, with 33.332 typed in.
So a 1.20 m span (Helmbold ratio 1.00), or a hull change that raises m_y - m_x, would pass all 13 + 9 + 11 verdicts. 'Stable, as sized' (airship_tail_design.md:150-151, commit ac41e6c) is the engine reproducing the file, not the file meeting the requirement.
Fix: add a variant_check fact that, for each vehicle declared stable, computes sum over vertical fins of 0.5 rho A cla l_f with Helmbold's slope, divides by (m_y - m_x) from the _am block, and requires at least 1.2. In added_mass_check, add a sign verdict K_s(_am_fin) > 0 for the same vehicles, so that a design, and not only a transcription, can fail.

## Medium

### Post-stall and broadside fin forces are wrong at 90 deg and switch on the sign of surge in the hover regime the yaw gate runs in
`PX4-Autopilot/Tools/simulation/gz/models/airship_vectored_independent_fin/model.sdf:53` (aero-basis)

LiftDrag computes cd = cda*alpha_s + cda_stall*(alpha - alpha_s), and cl falls linearly past stall.

The resulting values at 90 deg are wrong:
- **Drag, generic:** cd(90 deg) = 0.903*0.3 + 1.2*1.271 = 1.80, where a rectangular flat plate is about 1.2.
- **Drag, Cloudship:** 1.94*0.35 + 1.2*1.221 = 2.14 (cloudship_fin/model.sdf:55-59). Its comment at line 39 still says 'cd near 1.2-1.3 there'. That was true before 08be662 (1.31); the commit changed cda and alpha_stall without re-deriving cda_stall.
- **Lift, both families:** cl(90 deg) is 0.204 and 0.195 by design ('a plate's ~0.2 at 90 deg'). A flat plate has CL = CN cos(alpha) = 0 at 90 deg by symmetry.

Why it matters: sitl_check.py states that 'every check here turns the hull roughly in place'. That puts the vertical fins at alpha about +-90 deg, with a local lateral velocity of 4.2 r and u close to 0.
- The plugin returns nothing when forward . v <= 0. So the tail's broadside force is a switch on the sign of the surge drift: 229 r^2 N m of yaw damping when u > 0, against the hull's own nRabsR 194 r^2, and zero when u <= 0.
- In the u > 0 state, the spurious cl 0.2 points along +x for both yaw directions (checked against the source's alpha-sign logic). Each vertical fin pushes forward with 0.204 q S, about 0.75 N for the pair at 20 deg/s, which keeps u > 0. The rigid _fin plant has no hull drag to stop that drift.

The hover yaw gate on every _fin/_am_fin plant is therefore measuring a plugin artifact, not a tail.

Correct choice:
- cla_stall = -cl_stall/(pi/2 - alpha_s), so lift is 0 at 90 deg.
- cda_stall = (1.2 - cda*alpha_s)/(pi/2 - alpha_s): 0.731 generic, 0.427 Cloudship.
- State that the lift-drag plugin cannot represent a broadside fin at u <= 0. If the hover gate should see tail cross-flow drag, put it symmetrically in the Hydrodynamics block: about rho*1.2*S*l^3 = 152 r^2 for the vertical pair.

### Flap effectiveness is linear to 25 deg and added outside the stall logic, which overstates control authority
`PX4-Autopilot/Tools/simulation/gz/models/airship_fixed/model.sdf:431` (aero-basis)

control_joint_rad_to_cl is 2.466/rad, applied linearly to +-25 deg. LiftDrag adds it after the stall branch (cl = cl(alpha) + k*delta), with no limit.

- At alpha = 0, full deflection gives cl 1.075 on the exposed area. That is 1.5-2x the 0.55-0.7 CLmax Ananda measured for the whole plate at any angle.
- Combined with incidence, the fin's cl can reach 1.92 + 1.08 = 3.0.
- The thin-airfoil tau of 0.661 and Caughey's viscous factor of 0.8 are small-deflection results. Plain-flap effectiveness falls well below linear past about 10-15 deg. The KTH thesis the research read carries an explicit eta_d for 'the decreasing efficiency of the flap as its deflection increases'; the design dropped it.

Consequence: the note's 'authority at 4 m/s: rudder pair 124 N m, elevator pair 124 N m' (airship_tail_design.md:85) and the flight check's predicted stick responses rest on a linear value at full deflection. The margin over Munk at 5 deg sideslip (46 N m) survives a realistic reduction, so this does not flip a conclusion. It does overstate the high-deflection authority that finding 1's turn depends on.

The cross-check itself is sound: using K_W(B) = k_W(B)+k_B(W) for a flap is right by NACA 1307 eq. 33, and all four flap signs are correct.

Correct choice: set rad_to_cl to the 25 deg secant value, so that cl(25 deg) is at most the plate's CLmax x K_W(B), about 0.9, i.e. about 2.0/rad. Or limit the surfaces to about +-15 deg, where the linear basis holds. Either way, quote the authority on that basis.

### Later commits voided the reason given for renumbering upstream's 2507_gazebo-classic_cloudship; the rename is also typed as a non-breaking refactor
`ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt:73` (commits)

965e8dd44c renames upstream's 2507_gazebo-classic_cloudship to 2519. Upstream/main has this file at 2507. The message's reason is 'so that the generic family's complete variant has its slot', and airship_family_naming.md:132-133 says the same ('complete variant (mask 7) has its slot'). Two commits later, 6e74f20 and 52d183d06f make airship_fixed a family with no _fin member. variant_check fact 8 now reports an error for any _fin member on a vehicle whose fins are its actuators. So the family's complete variant is 2505 (am + drag), and 2506/2507 can never be used. The only reason left is keeping one classic twin inside its vehicle's decade, which is also why the 'ones digit 9 = classic twin' rule exists. The rename affects anyone who launches classic by PX4_SYS_AUTOSTART=2507, and upstream reviewers will see an airframe renumbered for a reason that no longer holds. The same commit is typed refactor(airframes) although it changes two SYS_AUTOSTART numbers: the fork's flight 2510 -> 2520, after which an FMU carrying 2510 boots with no airframe (the message admits this), and upstream's posix 2507. CONTRIBUTING.md defines refactor as neither fix nor feature and asks for '!' on breaking changes. Fix: keep upstream's 2507 file and exempt posix 2507 the way fact 13 already exempts flight 2500/2507 (the '9' rule then goes), or restate the rationale honestly. Mark the renumber with '!' and update the naming note, which also still says '2500 (reserved)' and names '2510_airship_vectored_independent'.

### The shared AS_* defaults cap the 2500's rudder at 43 % in rate modes, silently, halving its already speed-limited yaw authority
`src/modules/airship_att_control/airship_att_control_params.yaml:34` (firmware-impact), hardware-testable

The loop output is P*e + integ. The integrator is clamped at AS_YR_INT_LIM 0.3, and e is at most AS_YAWRATE_MAX = 0.2618 rad/s, so |u| <= 0.431 whatever the stick asks. The pod allocation needs only a few hundredths of torque to hold a rate, so the cap does not bind there. On 2500 the rudder is the only yaw effector and its moment scales with V squared.

Same model at 3 m/s, full yaw stick:
- Manual: rudder 1.00, 3.95 deg/s steady.
- Stabilized: rudder 0.42, 1.66 deg/s, against a 15 deg/s setpoint.
The allocator never reports saturation because |u| < 1, so nothing in logs or telemetry shows that 58 % of the rudder is unreachable. Setting AS_YAWRATE_I 0 (the obvious fix for finding 1) would drop the cap to 0.13.

The defaults were designed and gated only on pod plants (54 N m or 4.48 N m of authority available at rest). The D6 decision to close the loop in Acro was made on the pod basis. 2500 is the fixed-wing-like case, where PX4's own precedent is Acro yaw passthrough (FW_ACRO_YAW_EN 0).

Correct choice: 2500 needs its own treatment, measured on the new airship_fixed twin in Stabilized. Either set-default an AS_YR_INT_LIM of 1 with gains sized to rudder authority and an airspeed gate, or keep 2500 on the passthrough. This recent work built the twin but did not exercise these defaults on it.

Pin test: with 2500 on the bench in Stabilized, full yaw stick should drive the rudder pulses to about 1500 +- 216 us, not +-500.

### The firmware's embedded airframe metadata is stale: the 9535f8abcd .px4 advertises 'Generic Airship (Independent Vectoring)' as 2504, a number with no ROMFS file
`cmake/px4_metadata.cmake:74` (firmware-impact)

px4_generate_airframes_xml depends only on Tools/px_process_airframes.py, not on the airframe files. In the incremental build dir, build/px4_fmu-v6x_airship/airframes.xml is dated 2026-08-06 and was never regenerated through the 2504 -> 2510 -> 2520 renumberings. The ROMFS path does depend on the files (ROMFS/CMakeLists.txt:121-137), so rc.autostart in the same dir (2026-09-25 17:52) lists 2500/2507/2520.

Decoding head_20260924/px4_fmu-v6x_airship_9535f8abcd.px4 shows airframe_xml ids 2500, 2504, 2507, and no 2510 or 2520. The parameter_xml there is current.

Why it matters: QGroundControl caches this block when it flashes a .px4 (FirmwareImage.cc: the airframe XML is decompressed into PX4AirframeFactMetaData.xml). PX4AirframeLoader.cc:51-56 prefers that cached file over its bundled copy. Anyone who flashes this build through QGC and picks the independent airframe gets SYS_AUTOSTART 2504. The next boot then has no airframe file: rcS sets SYS_AUTOSTART 0 and plays the error tune.

The renumbering review should have caught this, since QGC metadata is one of the consumers of the number.

Correct choice: delete airframes.xml, or build clean, before packaging any firmware that is shared. The upstream fix is to add the init.d airframe files to that DEPENDS list; this is a real upstream build bug, latent because CI builds clean.

### The 9/10 -> 8/10 drop blamed on the tail is noise at the threshold plus a non-clean start: the Acro tracking step inherits the failed brake phase
`px4-airship-bench-tools/sitl_check.py:325` (firmware-impact)

The Acro tracking step is not a step from rest. The brake phase ends at -7.6 deg/s with its integrator loaded. commander('mode','acro') while already in Acro keeps yawRateLoopActive true, so there is no reset. The step therefore starts from that state, and the 40 % limit compares a 14 deg/s transient to the 6.67 setpoint.

The same code on the same sized-tail plant gave 9.26 deg/s (PASS, sitl_out/gates_20260924.txt, 20:23) and 9.35 (FAIL, chain_20260924.txt), against a 9.33 limit. Stabilized runs the identical law from a nearly settled start and peaked 8.71 (30.6 %). That is within the limit, and close to the model's clean-step 25.7 % (linear) or 20.7-24.7 % (with pod reversal). The model reproduces the gate sequence's Acro peak rising 24.8 -> 32.8 %.

What actually changed with the tail is settling: 2 % settling time 19.6 -> 33.8 s, zeta 0.69 -> 0.58. The failure worth acting on is the pre-existing release settling on every plant with air (see checked). Retuning P/I to chase this verdict would fit gains to a measurement artefact.

Correct choice: start the Acro step from rest. A Manual hop resets the integrator; then wait for |r| < 1 deg/s. Take the overshoot limit from the plant's predicted damping, as the wind check now does.

### 2500 keeps the yaw-rate gains tuned for the vectored airship; in rate modes they use only about 40 % of the rudder
`ROMFS/px4fmu_common/init.d/airframes/2500_generic_airship:32` (fixed-twin), hardware-testable

2500 sets no AS_* parameters, so it gets AS_YAWRATE_P 0.5, AS_YAWRATE_I 0.09 and AS_YR_INT_LIM 0.3 (`airship_att_control_params.yaml:12/22/34`). These were tuned on the vectored plant at rest, and 9535f8abcd discusses only 2510 and 2520.

The cap is exact. `RateControl::update` gives yaw torque = P·e + I, with |I| clamped to the integrator limit and no feed-forward. At full stick e ≤ 15 deg/s = 0.262 rad/s, so while the vehicle is short of the requested rate the torque is at most 0.5×0.262 + 0.3 = 0.43. The rudders are then far from ±1, so the allocator's residual is zero and no saturation is flagged; anti-windup never engages.

On a fin-steered hull a steady turn needs rudder in proportion to the rate. The P term is only 6-12 % of the fins' yaw damping at 3-6 m/s: K_p·N_δ = 3.87u² against N_r = 2qS·cla·l²/u = 193.6u N m/(rad/s). So the loop runs on its integral. It can hold a rate with zero error only up to 30 % of the full-rudder rate, and Ki 0.09 needs tens of seconds to get there.

The body-axis model of 2505 (the plant the docs say to tune on) gives, with this controller:
- 3 m/s, full stick: holds 2.0 deg/s at command 0.41, where full rudder in Manual holds 4.88.
- 6 m/s, full stick: 3.85 against 9.76.
- 3 m/s, a 5 deg/s command: 1.59 deg/s after 60 s.
- With AS_YR_INT_LIM 1.0, full stick at 3 m/s reaches 4.88 (full rudder).

So Acro, Stabilized, Altitude and Position all turn this vehicle at under half the rate its rudders allow, and never reach the stick's rate. The twin, the first plant on which this could be seen, was flown only in Manual.

Fix: tune AS_YAWRATE_P/I and AS_YR_INT_LIM on 2505 and set them in 2500 (at least the integrator limit at 1.0). Add an Acro leg to `airship_fixed_flight_check.py`.

Bench test: switch the FMU to SYS_AUTOSTART 2500, select Acro, and hold full yaw stick while stationary. Predicted: the rudder outputs step to about 1565 us (P = 0.131), ramp to about 1716 us over about 12.7 s as I reaches 0.3, and stop there. The same stick in Manual gives 2000 us (preset2500 runC shows yaw +0.5 → 1750).

### The default 2500 twin (and 2501) cannot hold a turn: the rudder only sets a sideslip
`ROMFS/px4fmu_common/init.d-posix/airframes/2500_gz_airship_fixed:31` (fixed-twin)

On airship_fixed the only side force is the fins' lift at x -4.2 m; the hull has no drag block and no added mass. In steady flight the moment must be zero, so the net fin force must be zero, which leaves no centripetal force. The yaw rate therefore decays to 0, and full rudder only gives a heading offset of k·δmax/cla = 2.466×0.4363/6.4 = 0.168 rad = 9.6 deg, at any speed.

The records confirm the start of this:
- `fixed_flight_airship_fixed_yaw_+1.csv`: the rate peaks at 5.08 deg/s at 0.48 s and is down to 2.47 at 1.5 s, with the stick still full.
- The model gives 5.19 and 2.82.
- On `_am_drag` the rate holds (5.92 measured, 6.21 model).

Predicted steady full-rudder rates at 3 m/s:
- airship_fixed: 0.00 deg/s (+9.6 deg of heading in total).
- _drag: 0.65 deg/s. Only the quadratic sway drag gives side force, so the turn radius is about 265 m.
- _am: 4.80 deg/s.
- _am_drag: 4.88 deg/s.

Pitch works the same way on the bare plant: a held elevator settles back to zero pitch and gives 0.5 m/s of heave at 3 m/s.

The flight check passes all four plants 11/11 because it judges only the 1.5 s peak (`YAW_S` at line 90, verdict at lines 600-604). The target listed first as 'Generic Airship ... fly it forward first' (`vehicles.md:232-233`, posix comment lines 72-74), which also shares the flight number, is therefore a plant on which the vehicle's main maneuver does not exist.

Fix:
- Say in the 2500 posix comment and in the docs that 2500 and 2501 have no steady turn and no pitch hold (rudder = sideslip ≤ 9.6 deg, elevator = heave), and point to 2505 for flying.
- Add a held-rudder leg (about 10 s, predicted by the body-axis model) to the flight check, so the records tell the members apart.

### The tail requirement and its authority row leave out the fins' yaw damping, so the vectored airship's forward-flight yaw envelope is not stated anywhere, while _am_fin_drag is named the plant to tune the yaw loop against
`px4-airship-bench-tools/airship_tail_design.md:85` (gap:Yaw control in forward flight on the vectored _fin members: fixed fins, no control surfaces, constant pod authority)

**What the note does.** The note sizes the tail for static margin only (lines 11-22). The authority row at line 85 puts the pods' 54 N m next to the Munk moment at 5 deg of sideslip (46 N m) at 4 m/s, which reads as authority to spare.

**What the row leaves out.** In a turn the pods work against the fins' rate damping, not the Munk moment. That damping is 193.6u N m per rad/s, which is 13.5 N m per deg/s at 4 m/s. The note itself records this number as measured (185 for 194, line 149) but never draws the consequence.

**The consequence, from the files.** The steady full-authority turn is r = 0.269/u rad/s:

| plant | speed | turn rate | turn radius |
|---|---|---|---|
| _am_fin_drag, nonlinear | 4 m/s | 3.6 deg/s | 64 m, 6 hull lengths |
| _am_fin_drag, nonlinear | 2 m/s | 6.9 deg/s | |
| _drag, before ac41e6c | any speed | 29-30 deg/s | |

- Below about 2 m/s the plugin's tail stalls at full authority (tail alpha reaches 0.30 rad at 2 m/s). The rate then jumps to 18.7 deg/s and stays there regardless of speed, with a bistable band between 1.8 and 2.0 m/s.
- So the 15 deg/s AS_YAWRATE_MAX is reachable only below about 1.8 m/s.
- With the 2520 loop as tuned, the rate is 1.6 deg/s at 4 m/s.

**Where it is missing.** The envelope appears in none of these:
- the note;
- the _fin, _am_fin and _am_fin_drag model headers;
- docs vehicles.md:282-316.

Yet _am_fin_drag model.sdf:6-8 and vehicles.md:312 call it the plant to tune the yaw loop against. Every yaw check (sitl_check.py:15-18) turns the hull in place, where the fins produce nothing. Tuning there as prescribed tunes against _drag plus added mass. The one dimension along which ac41e6c changed the yaw plant, speed, was never exercised on any vectored plant.

**Correct choice, within this change set.**
- Add a forward-flight requirement row to the note. It should give the full-authority turn rate/radius against speed (0.27/u rad/s above about 2 m/s, and the stall regime below), and the rate the 2520 loop actually reaches.
- State the same envelope in the three model headers and in the docs.
- Add a forward-flight yaw case to the _fin plants' checks: throttle hold at 2 and 4 m/s (throttle stick -0.949 and -0.795), full and half stick for 20 s, then release. Log r, yawspeed_integ and unallocated_torque[2].

**Choices that are not correct.**
- Resizing the fins: neutral fins still give only 0.39/u, because constant pod authority against damping proportional to u makes the turn rate fall as 1/u for any fixed fin.
- Rudders on the vectored _fin members: 2520 declares only two motors and two tilt servos. The family scheme names physics, not actuators. Rudders would make the twin a different vehicle, and they do not fix the windup in finding 1. Adding rudders is a vehicle decision, which the allocator's surface path already supports.

### Added mass and the tail sizing use the drawn hull's 34.36 m^3, but the hull's buoyancy and mass budget use 39.2 m^3
`Tools/simulation/gz/models/airship_base_am/model.sdf:34` (model-files)

The vehicle is neutrally buoyant at 49 kg because the hull collision box displaces 39.2 m^3 (48.02 kg of air). The Lamb block and the 33.3 N m/(m/s)^2 Munk moment that the fins are sized against are instead computed on the drawn 5.25 x 1.25 ellipsoid (34.36 m^3, rhoV 42.09 kg). The only justification given is "a box of the same volume class", which covers a 14 % difference. The drawn hull could lift only 43.07 kg with the gondola, so the drawn hull, the mass and the buoyancy cannot all hold. The Cloudship already fixed the same problem: its collision is the drawn ellipsoid scaled to the real volume. Recomputed on a fineness-4.2 hull of 39.2 m^3: Munk 38.03 (not 33.33), rr about 192 (not 154), yy 41.7 (not 36.5), so total yaw inertia is about 303 against 265 (+14 %). The tail ratio then falls from 1.38 to 1.21 on the data slope, and from 1.22 to 1.07 on Helmbold's slope, which is the stated sizing bound, below the 1.2 requirement. Adding the fins' own added mass, which is unmodelled (+2.7 kg on m_y by the design note), gives 1.13 and 1.00. So the _am/_am_fin/_am_fin_drag and airship_fixed_am* plants have too little added inertia, and the tail meets its requirement only on the smaller hull. The fix is to make the hull geometry single-source, as the Cloudship does: draw and collide one fineness-4.2 ellipsoid of the real volume (radii about 5.486 x 1.306), then recompute the Lamb block, fin station and r/s, and the rigid inertia. Then re-size the span against Helmbold on that hull; my estimate is that it lands at about 1.14 and needs more span. The alternative is to shrink the mass budget to what the drawn hull displaces.

### The bare name, the target users pick, is the least physical plant; 'fly it forward first' accelerates without limit
`docs/en/sim_gazebo_gz/vehicles.md:223` (scheme-docs-upstream)

The grammar makes `<vehicle>` the rigid hull. So `make px4_sitl gz_cloudship` and `gz_airship_vectored_independent` spawn hulls where 'a torque keeps accelerating it and nothing damps a rotation' (vehicles.md:269).

`gz_airship_fixed` has no hull drag, and its fins use LiftDrag's linear-in-alpha drag (airship_fixed/model.sdf:405), which is zero at zero incidence. Yet vehicles.md:233 and posix 2500:74 say 'fly it forward first'. At constant throttle the speed never settles. airship_fixed_flight_check.py:36 models it as u = u0 + a t. The chain3_20260925.txt drag-free (_am) run went from 0.12 to 3.38 m/s at a constant 0.345 m/s^2, and the check has to cut throttle to 0 to hold speed (line 584). Flap authority grows with u^2, so the loop gain depends on how long the user held the throttle.

Users moving from gazebo-classic_cloudship had full airship dynamics (libgazebo_airship_dynamics_plugin). Upstream's bare name means the vehicle (x500), not a reduced plant.

Recommended: the user-facing airframe of each vehicle should default PX4_SIM_MODEL to the complete plant (`_am_fin_drag`, or `_am_drag` for the fixed airship). Either name the reduced plants for what they remove, or keep them reachable only through PX4_SIM_MODEL. The docs' own rule 'tune the loop on the complete one' (line 228) says the complete plant is the reference.

### Moving upstream's 2507_gazebo-classic_cloudship to 2519 no longer has a reason and breaks upstream's flight/posix Cloudship pairing
`ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt:70` (scheme-docs-upstream)

965e8dd44c moved upstream's file 'so that the generic family's complete variant has its slot' (mask 7, 2507). 52d183d06f then dropped the fin bit for the 2500 decade: its complete member is 2505, and 2502, 2503, 2506 and 2507 can never be used. The move now serves only the branch's own '9 = classic twin' rule.

It also undoes the pairing upstream had: flight 2507 Cloudship with posix 2507 Cloudship, the very 'share the flight number' pattern the naming note calls upstream's own. And it breaks `PX4_SYS_AUTOSTART=2507` for classic users.

The in-tree reasoning is now wrong. 2510_gz_cloudship:21-24 says 2507 'shares the generic airship's decade, whose ones digits are that family's physics variants', but 7 is not one of them. CMakeLists:72-73 records the move as history ('moved from 2507 ... at 2519'), which is change-log text in a build file. variant_check.py:629-630 would reject restoring the file.

Recommended: rename it back to 2507_gazebo-classic_cloudship; the contents are unchanged, 100% similarity. Drop the 9 rule, and have the comment say 2507 is the classic Cloudship paired with flight 2507.

### Upstream-facing files promise enforcement by tools outside both repos
`Tools/simulation/gz/models/airship_base_am/model.sdf:9` (scheme-docs-upstream)

The model headers say 'variant_check.py holds it to that' or 'holds equal':
- airship_base_am:9
- airship_fixed_am:5
- airship_vectored_independent_am:6
- cloudship_am:38 and :52
- cloudship:48

That file is in neither PX4-Autopilot nor PX4-gazebo-models; it lives in an unversioned local directory. Two airframes justify gains by 'the SITL gate', which is also out of tree: posix 2510_gz_cloudship:95 ('the SITL gate on this plant passes with them') and flight 2507_cloudship:36.

The family's correctness depends on invariants only that tool checks:
- the _am copies equal their bodies except for the block;
- the fin blocks and the drag blocks are element-identical across members;
- lta.sdf's 80 enable lines cover every model;
- the docs numbers match the airframes.

After upstreaming, nobody runs it and the copies drift silently. Under the upstream text standard, a claim should be reproducible from the tree. Recommended: move the structural facts into the models repo, which already has a CI hook (sdf_parser.yml) and a tests/ directory, or into PX4 Tools/. Otherwise remove the references and cut the invariants down (findings 1 and 5 remove most of them). State gain justifications as a reproducible procedure, not a private gate result.

### The '_am must be a full copy' premise is false: SDF parameter passing reproduces each committed _am from a thin include
`docs/en/sim_gazebo_gz/vehicles.md:224` (scheme-docs-upstream)

The docs, the naming note and the model headers all say `<fluid_added_mass>` forces a full copy, because 'a merge include can only add elements'. That premise costs about 1550 duplicated lines: airship_base_am 254, cloudship_am 587, airship_fixed_am 487, airship_vectored_independent_am 223.

libsdformat has supported `//include/experimental:params` since libsdformat 10, with add, modify, remove and replace. With the installed libsdformat14 14.9.0 I parsed this thin form for each vehicle:
`<include merge="true"><uri>model://<vehicle></uri><experimental:params><link element_id="base_link"><inertial action="replace">(26-line inertial incl. fluid_added_mass)</inertial></link></experimental:params></include>`

Root.to_string() matched the committed file for all three (airship_fixed_am, cloudship_am, airship_vectored_independent_am) once the `_merged__X__model__` frame names were normalized; every such frame has a zero pose on base_link. Put the add directly on `<fluid_added_mass>` instead and the parser rejects it: an action is only allowed on the identified element or its direct children.

Limits of that result: it is a parse-level result only, since gz sim was not run; the namespace is 'experimental'; and mass and inertia still appear twice (26 lines instead of about 250 to 590).

Recommended: try one thin _am in gz with one first-light run and added_mass_check. If it holds, delete airship_base_am and the copies, and fix vehicles.md:224. If not, write down why in the docs.

### The 30%/40% rate tolerance admits a plant with no Munk moment and +-25-30% rudder authority
`px4-airship-bench-tools/airship_fixed_flight_check.py:95` (verification-method)

Measured peaks agree with simulate() within 7%, and within 2% on _am after the onset fix. Rerunning the check's own plant() and simulate() at 3.2 m/s:
- airship_fixed_am with munk = 0: 4.64 against 6.07 (-24%). PASS at 40%.
- Flap slope x0.75 or x1.3: -25% / +30%. PASS.
- Pitch with Munk x2: +20%. PASS.
- Rigid plant, flap slope x0.75: -25%. PASS at 30%.
- The yaw peak barely depends on inertia: I x0.75 or x1.3 moves it only +3..6% / -3..6%.
So on the added-mass plants the leg cannot see Munk at all, and it verifies rudder authority (the 124 N m claim) only to about 25%. Nothing justifies the 30/40 split (RATE_TOL is keyed only on 'added-mass plant').
Fix: add the missing Coriolis term (next finding), tighten to about 10%, and run the legs at a second speed, so the q ~ u^2 scaling is tested too.

### The gate's 'turns in place, insensitive to surge' premise fails on _fin plants, and the wind expectation leaves the fins out
`px4-airship-bench-tools/sitl_check.py:253` (verification-method)

The docstring (:15-18) says nothing in the gate is sensitive to surge. The lift-drag plugin returns when forward.v_cp <= 0, and a pure rotation gives the vertical fins forward.v_cp = 0, so the fins can act only when the hull moves. Yet the _fin records show them carrying yaw:
- Manual: 17.5 deg/s against 27.8 on the rigid plant.
- Wind, I off: 1.16 deg/s against 2.09 (chain_20260924.txt:141-150).
- cloudship_fin wind: 38.9 deg/s against the file-derived 49.4 used as the expectation (21% low) and the rigid 46.85.
This implies a forward drift of order 0.1 m/s that the gate neither records nor controls. If the hull drifts backward, the fins drop out, so the _fin/_am_fin baselines (8/10, 8/9) depend on an unrecorded state.
On the specific wind-check questions:
- rr belongs (the _am_fin run gave 93% of expected, the same fraction as the rigid run).
- Leaving out the drag cross terms is correct.
- The fins' damping is missing.
- The check stays one-sided (> 0.5 x expected), so a plant missing its rr still passes, and abs() on nR/nRabsR hides a sign error.
Fix: log u and v from vehicle_local_position in the gate, and either hold position or report the fin plants as surge-dependent. Make the band two-sided now that the expectation comes from the plant.

## Low

### The unmodelled fin added mass would not raise the steady Munk coefficient; the quarter-chord cp already contains it
`px4-airship-bench-tools/airship_tail_design.md:112` (aero-basis)

The note says the fins' own added mass, pi rho (c/2)^2 per span (2.7 kg for the pair), 'would raise the Munk coefficient from 33.3 to 36'.

In steady flow that couple is already modelled. For a flat plate, the added-mass couple about mid-chord, pi rho (c/2)^2 U^2 alpha, is exactly the moment of the circulatory lift pi rho U^2 c alpha acting at c/4 about mid-chord. Theodorsen's steady limit gives the same: 2 pi rho U^2 b^2 (a + 1/2) alpha at a = 0. Placing the lift-drag cp at the quarter chord therefore already represents it. Only the unsteady terms are genuinely missing: 2.7 kg of sway added mass and about 48 kg m^2 of yaw/pitch added inertia (+18 % on 265 kg m^2).

Consequence: the note implies a smaller real margin (46.1/36 = 1.28) that does not exist. More importantly, anyone who later adds the fin plates to the Lamb block as the note suggests would double-count 2.7 u v, which is 21 % of the net static margin.

Correct choice: say that the steady part is in the c/4 cp and that only the acceleration terms are missing. If the fin added mass is ever modelled, move cp to mid-chord or add only the inertia terms.

### Rotation-induced fin incidence is read at c/4, which under-damps by about 12 %; the (1+r/s)^2 carryover is also applied to roll
`PX4-Autopilot/Tools/simulation/gz/models/airship_vectored_independent_fin/model.sdf:43` (aero-basis)

The plugin evaluates omega x cp at the quarter chord, so the fin yaw/pitch damping is K u l_c4^2 = 193.6 u. The engine measured 185.

Thin-airfoil pitch-rate theory takes the rate-induced incidence at the 3/4-chord point (x = -4.70). That gives K u * 4.20 * 4.70 = 216.7 u, about 12 % more. DATCOM's usual tail factor of about 1.1 on l^2 says the same.

The carryover (1+r/s)^2 is consistent for yaw and pitch rate, because slender-body cross-flow at the fin station is local. For roll-rate-induced incidence it is not: body carryover does not apply to antisymmetric loading. The top fin's 2.6 m lever above the CoM therefore gets an interference factor it should not have, and roll damping is overstated.

Both errors are small next to findings 1-3, and the under-damping errs toward conservatism for weathercock stability.

Correct choice: accept this and document it, or scale cla's damping share. The plugin cannot separate static and rate terms, so the practical fix is a note plus a small nRabsR/mQabsQ-style linear term if the gate is sensitive to it.

### Comments made false by one commit are fixed only 3-4 commits later, and one comment names a model that does not exist yet
`ROMFS/px4fmu_common/init.d-posix/airframes/2510_gz_cloudship:97` (commits)

Three cases. (a) From 52d183d06f, which pins the sized tail, through 38f4cfdff0, 2510_gz_cloudship:95-99 still says '54 N m against 76.5 is 0.71, and the pendulum periods match to 5% ... So the AS_YAWRATE_* defaults transfer'. With izz 108.8 the ratio is 0.50, so the comment's conclusion no longer follows; 9535f8abcd fixes it. The 2520 flight comment is stale over the same range. (b) 965e8dd44c moves the classic Cloudship to 2519 but pins 71444cd, whose cloudship/cloudship_am headers say 'the posix 2507 is upstream's gazebo-classic one'. This stays false through 38f4cfdff0 and is fixed by bdec6cd, pinned in 9535f8abcd. (c) ac41e6c's airship_base header (line 23) and fin comment (line 112, also airship_base_am:116) say 'airship_fixed gives them flaps', but airship_fixed first appears in 6e74f20. Each is a comment, but it is the kind a reader relies on to judge the gains. Fix: when the history is re-cut, fold 9535f8abcd's comment fixes into the commit that pins the tail. Put bdec6cd's '2519' wording with the renumber, or drop the number from the models repo so a PX4 renumber never needs a models commit. Move the airship_fixed mention into 6e74f20.

### Seven airframes get comment paragraphs in 52d183d06f that 38f4cfdff0 deletes three commits later
`ROMFS/px4fmu_common/init.d-posix/airframes/2526_gz_airship_vectored_independent_am_fin:1` (commits)

52d183d06f creates 2501, 2504, 2505, 2522, 2524, 2526 and 2527 with 3-5 line paragraphs. Its message says the '2516/2526 comments no longer claim a rate is kept forever'. 2526 is created in that same commit, so the 'no longer' describes the working tree, not the diff. 38f4cfdff0 then removes those paragraphs from all 13 thin airframes, citing drift that 'the last change had to correct'. Both commits are unpushed, so the history shows text added and removed for no reason. 38f4cfdff0's reason ('a third copy of text already in the vehicle list and the CMakeLists decade comment') also applies to the paragraphs it keeps: 2500_gz_airship_fixed:23-25 restates the mask arithmetic, and 2510_gz_cloudship:18-26 restates the whole decade table and the 2507 -> 2519 history. Fix: create the new airframes thin in the commit that adds them, and cut those two base paragraphs down to what only they know.

### ac41e6c says a 1.20 m span 'met the target only on the data's slope', but by fin_basis.py it did not
`Tools/simulation/gz/models/airship_base/model.sdf:20` (commits)

The ac41e6c message says 'The span was sized on Helmbold's lower slope, on which the ratio is 1.22; 1.20 m met the target only on the data's slope.' airship_tail_design.md:74 says the same ('1.20 m met 1.2 x only on the data's slope, 1.0 x on Helmbold's'). The target is 1.2 x Munk. By fin_basis.cla, at 1.20 m the data slope gives 1.225*1.2*6.301*4.2/33.332 = 1.167 and Helmbold gives 1.002. So 1.20 m misses the target on both slopes. The smallest span that meets 1.2 x on Helmbold is 1.39 m (1.209), which does support choosing 1.40 m. The chosen design is right; the stated reason for rejecting 1.20 m is wrong. Fix: '1.20 m gave 1.17 x on the data's slope and 1.00 x on Helmbold's; 1.39 m is the least span meeting 1.2 x on Helmbold's'.

### 6e74f20 quotes the superseded run, and its 7 % claim for the drag plant rests only on the rejected peak reader
`Tools/simulation/gz/models/airship_fixed/model.sdf:1` (commits)

The message's stick peaks (5.1/4.9 yaw for 5.3/5.0, 3.9/3.4 pitch for 4.0/3.6) come from chain_20260924.txt:38-41. That run used the peak reader the later review found opens after the gz CLI attaches. The corrected run, chain2_20260924.txt (23:18 on 09-24, before the 16:54 09-25 commit), reads 5.1/5.2, 4.9/5.0, 3.9/4.0 and 3.5/3.6. More importantly, the _drag plant was measured only with the old reader and never re-run. Its records start mid-rise: fixed_flight_airship_fixed_drag_yaw_+1.csv begins at r = 3.82 deg/s against a 5.48 peak, and the pitch files begin at 3.3-3.8 against 4.0-4.4 peaks. The other three plants' records start at 0.6-0.8. The rates are still rising at the first sample, so the peaks were probably captured. Still, 'The drag, added-mass and complete plants land within 7 %' cites, for the drag plant, a measurement the review rejected, and its record cannot show the onset. Fix: re-run airship_fixed_drag with the corrected reader and quote the chain2/chain3 numbers.

### Tracked upstream-bound model headers name a private, unversioned tool as what enforces their copy invariants
`Tools/simulation/gz/models/airship_base_am/model.sdf:9` (commits)

ac41e6c and 6e74f20 add 'variant_check.py holds it to that' and similar to airship_base_am:9, airship_vectored_independent_am:6 and airship_fixed_am:5. 71444cd had already done this in cloudship and cloudship_am. variant_check.py exists only in ~/dev/px4-airship-bench-tools, which is in neither repo and not under git. Readers of PX4-gazebo-models are told that something keeps the three 250-490 line copies identical, but nothing they have does. The tool is also unversioned, so the checks these comments and messages rely on cannot be re-run as they stood at an old commit. Only today's tool exists, and it already fails 965e8dd44c's pinned Cloudship fins (1.856 against the 7.254 basis) for a reason that is not a defect of that commit. Fix: state the invariant without naming the tool ('keep identical to airship_base except the <fluid_added_mass> block'), or move the check into the repo, for example a test under Tools/ that CI can run.

### The submodule scope 'models' says nothing, differs from 71444cd's 'airship', and several subjects are not imperative
`Tools/simulation/gz:1` (commits)

The four models commits use '(models)' in a repo that contains nothing but models. The same branch's previous commit is 'feat(airship): ...', and upstream PX4-gazebo-models scopes by model ('feat(advanced_plane): add parachute plugin (#150)'). A scope of '(airship)' or the model name ('airship_fixed', 'cloudship') would say what changed. Several subjects in both repos are noun phrases: 'airship_fixed, the 2500 airframe's gz twin, and members', 'Cloudship fins on the family's lift basis', 'the generic airship families', 'airship decades in the order of the flight numbers', 'airship yaw-gain comments after ...'. CONTRIBUTING.md requires the description 'written in imperative form'. CI checks only the header regex, which they all pass, so this matters mainly at the upstream re-cut, where each subject becomes the squash-merge title. Fix: at the re-cut, use an informative scope and imperative subjects, e.g. 'feat(airship): add airship_fixed, the gz twin of the 2500 airframe'.

### The yaw-inertia figures in the 2520 and posix 2510 comments are base_link's, not the vehicle's, and they point at the smaller of the two tuning gaps
`ROMFS/px4fmu_common/init.d/airframes/2520_airship_vectored_independent:35` (firmware-impact)

The comments quote '76.5 kg m^2 then, 108.8 now'. The same comparison appears in posix 2510_gz_cloudship:97-98 as '0.71 ... 0.50'. Those are base_link izz values only. The vehicle's yaw inertia includes the pod and prop links: 78.75 -> 111.1. variant/added_mass_check itself expects 111.1 and the engine measured 111.4. The authority-to-inertia ratios are therefore 0.69 -> 0.49, not 0.71 -> 0.50.

The framing also leads with the tail's +42 %, which applies to the rigid plant only. On the complete plant, whose added mass the real vehicle always has, the tail is 233 -> 265 kg m^2 (+14 %). The defaults were tuned on a plant 3.4x lighter than that. The second sentence does mention added mass, but a reader will retune for the wrong factor.

These are flight-airframe comments destined for upstream, where verified figures are required. Correct choice: quote the whole-vehicle numbers and name added mass as the dominant difference.

### The naming design note's family table still lists independent vectoring at flight 2510 and 'none yet' for the fixed twin
`px4-airship-bench-tools/airship_family_naming.md:81` (firmware-impact)

The first table (lines 79-82) says the generic independent-vectoring airship is at flight 2510, and that generic fixed has no gz model. Both have been wrong since 965e8dd44c and 6e74f20 (airship_fixed). The second table (lines 123-127) and the commit messages say 2520 and 2500-2505.

This note is the reference variant_check.py's decade table follows, so the two tables disagreeing about the flight number is the kind of stale reference the renumbering review should have caught. Correct choice: 2520 and airship_fixed in the first table.

### cd at 90 deg is 2.14, not the 'near 1.2-1.3' the comment claims, and the in-place gate works in that regime
`PX4-Autopilot/Tools/simulation/gz/models/cloudship_fin/model.sdf:39` (gap:Cloudship fin lift basis (08be662): is the slender-body carryover achieved on a converging stern? Covers both vehicles, but mainly the Cloudship.)

gz-sim8 LiftDrag.cc:432-448 sets cd = (cda*alpha_stall + cda_stall*(alpha - alpha_stall)) past stall. With the new cda 1.94, alpha_stall 0.35 and cda_stall 1.2 (lines 55/57/59, repeated in all four plugins and in cloudship_am_fin:42/58/62): cd(90 deg) = 0.679 + 1.2*1.2208 = 2.144. That is 65% above a flat plate. The comment was true only of the old values: 0.25*0.6 + 1.2*0.971 = 1.315. cl(90 deg) also moved, 0.628 -> 0.195.

sitl_check.py:15-18 turns the hull 'roughly in place'. The fin cp flow is then mostly lateral, and LiftDrag.cc:312 acts only while forward.v > 0. So when the fins act in the gate they are near |alpha| 90 deg, where these post-stall constants set the lateral drag and the damping. cla does not.

Correct choice: cda_stall = (1.2 - 1.94*0.35)/(pi/2 - 0.35) = 0.43 for a plate's cd of about 1.2 at 90 deg. Otherwise change the comment to 2.14, and state that the Cloudship fin-plant gate results depend on this constant.

### The claim that the Cloudship fin-plant gate is unchanged by the fin change has no record behind it; none of the three verdicts is shown to depend on cla
`px4-airship-bench-tools/airship_tail_design.md:176` (gap:Cloudship fin lift basis (08be662): is the slender-body carryover achieved on a converging stern? Covers both vehicles, but mainly the Cloudship.)

The note says the fin plant 8/9 and complete plant 8/9 fail on overshoot 'as before the fin change'.

The records hold a pre-change gate only for cloudship_am_fin_drag: variants_gz_cloudship_am_fin_drag_sitl_check.txt (09-23, cla 1.856 per added_mass_run5-7) gave 8/9, failing Acro tracking with max 10.42 and mean 5.17. The post-change run (chain_20260924.txt:252-271) gives 8/9 on the same check, 10.38 and 5.11. That verdict does not depend on cla.

cloudship_fin (8/9: Stabilized tracking, max 10.05 against 9.34 allowed) and cloudship_am_fin (3/9) were never gated on the old fin block. variants_summary.txt covers cloudship, cloudship_drag and cloudship_am_fin_drag only; the 09-23 fin runs are first-light and added-mass. So 'as before' is unsupported for the fin plant, and whether either verdict depends on cla was never measured.

Physically, the in-place gate exercises the post-stall constants, which changed in the same commit (previous finding), not cla. On _am_fin the larger cla halves the net instability (-4.28 -> -2.42), so a lower cla would not raise its 3/9.

Correct choice: rerun the cloudship_fin and cloudship_am_fin gates on 71444cd's fin block to establish the before, or remove 'as before the fin change' for the fin plant.

### The basis-enforcing check says Helmbold's slope; the files and fin_basis.cla use 2 pi AR/(AR+2)
`px4-airship-bench-tools/variant_check.py:221` (gap:Cloudship fin lift basis (08be662): is the slender-body carryover achieved on a converging stern? Covers both vehicles, but mainly the Cloudship.)

fin_expectations' docstring says the family basis is 'Helmbold's slope on the two panels joined through the hull, times the slender-body carryover'.

Line 241 calls fin_basis.cla(chord, span, root), whose default slope is wing_slope = 2 pi AR/(AR+2) (fin_basis.py:43-45, 73-75). That gives the files' 6.400 (3.665 x 1.746) and 7.254 (2.866 x 2.531). On Helmbold they would be 5.646 and 5.786.

fin_basis.py:15-18 and the design note present Helmbold only as the sizing bound. A reader of the check that holds every fin plugin to the basis is told the files sit on the conservative slope when they sit on the upper one.

Correct choice: change the docstring to the plate slope, with Helmbold named as the sizing bound only.

### Moving the gondola collision also moved the CoV, so the CoV sits 5 mm ahead of the CoM
`Tools/simulation/gz/models/airship_base/model.sdf:66` (model-files)

The header says the gondola moved 0.247 m forward "so that the fins' mass leaves the centre of mass under the centre of volume". But the gondola collision (0.8 m^3 of buoyancy) moved with it. The CoV is now x = 0.8*0.247/40 = +0.00494, while the whole-vehicle CoM is (36*0.247 - 8.9)/49 = -0.00016, a 5.1 mm offset. Over the 1.2045 m pendulum arm that gives a 0.24 deg nose-up static trim, and the header lists the CoV as "2.017 m" with no x offset. The effect is tiny, but gz does not read what the comment says. The fix is to put the gondola at x = 8.9/(36 - 0.98) = 0.254 m, or to state the offset.

### Stale comment: Cloudship fin cd at 90 deg is 2.14, not the 1.2-1.3 claimed
`Tools/simulation/gz/models/cloudship_fin/model.sdf:39` (model-files)

08be662 raised cda from 0.25 to 1.94 and lowered alpha_stall from 0.6 to 0.35 but kept cda_stall 1.2. In gz, cd(90 deg) = cda*alpha_s + cda_stall*(pi/2 - alpha_s) = 0.679 + 1.465 = 2.144. The old values gave 1.315, which is what the comment still describes. The same line sits in cloudship_am_fin. The generic fins carry the same cda_stall and read 1.80 at 90 deg. The fix is cda_stall = (1.25 - cda*alpha_s)/(pi/2 - alpha_s): 0.468 for the Cloudship and 0.770 for the generic fins. Otherwise, correct the comment. The consequence is small: only the horizontal fins in a near-vertical climb or descent reach these angles.

### Flap cl is added after the stall clip, so a fin can reach cl 3.0 against its stated stall cl of 1.9
`Tools/simulation/gz/models/airship_fixed/model.sdf:431` (model-files)

LiftDrag computes the stall-clipped cl first and then adds control_joint_rad_to_cl * delta. At alpha_stall with 25 deg of same-sense flap, that is 1.92 + 2.466*0.4363 = 3.00. The header says the fin stalls at cl 1.9. The flap increment also acts at the whole fin's quarter chord (-4.20) rather than near the flap, which understates its arm by about 5 %. In steady turns the fin alpha and the rudder oppose, so the practical effect is that post-stall recovery authority is overstated. Either state this, or bound the flap authority, for example by lowering the stall cl so that cl_stall + dcl_max matches the plate data the header cites.

### The ventral fin and lower rudder reach 1.3 cm below the gondola's ground contact, which the header does not mention
`Tools/simulation/gz/models/airship_base/model.sdf:145` (model-files)

fin_bottom_visual is centred at z 0.687 with a 1.40 m span, so its tip is at z -0.013. The gondola collision's bottom, which is the ground contact, is at z 0.000. The same holds for flap_bottom (airship_fixed:339). At rest the ventral fin sits inside the ground plane; any nose-up attitude on the ground buries its trailing edge by about 4.95 sin(theta). It is visual only, so gz does not care. But this tail is the sizing for the 2500 vehicle, so the span was chosen with no ground-clearance constraint. The Cloudship header documents its own equivalent (5.5 cm); airship_base should too, or the design should take the clearance as a constraint.

### World wind moves the fins' airflow but not the hull's drag or Munk (latent)
`Tools/simulation/gz/worlds/lta.sdf:170` (model-files)

gz-sim8 LiftDrag subtracts the Wind entity's velocity. Hydrodynamics reads only its /ocean_current topic, and the DART added mass uses inertial velocity. So with wind set through the WindEffects system, which lta.sdf loads, the _fin plants see air-relative flow on the fins but ground-relative flow for hull drag and the Munk moment. A hull holding station in a steady wind would feel full fin weathercocking and no Munk moment. Nothing uses this today: sitl_check injects wind as a hull torque. But the _am_fin_drag model is documented as "the plant to tune against". Its header should say not to use world wind, or the Hydrodynamics current should be fed from the same wind.

### Fin and flap colours dropped: empty <material> renders black
`Tools/simulation/gz/models/airship_base/model.sdf:121` (model-files)

ac41e6c replaced the tail's red (0.6 0.2 0.2) with empty <material></material> elements, and airship_fixed's flaps have them too. SDF's material defaults are ambient and diffuse "0 0 0 1", and gz-sim loads a present material as given. The fins therefore render black rather than taking the scene default. This is visual only: restore the colours or remove the empty elements.

### The mask rule is stated as universal where readers look, but the 2500 decade breaks it and the gaps are unexplained
`ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt:68` (scheme-docs-upstream)

CMakeLists:68-70 is the one place the scheme is written, and the only one variant_check parses. It says 'a gz member's ones digit is its physics mask (1 hull drag, 2 fins, 4 added mass)'. vehicles.md:223 and 225 say 'The bare name is the rigid hull with buoyancy and nothing else' and that the ones digit is 'a mask of the physics the model carries'.

Every airship_fixed member carries four LiftDrag fin plugins, yet bit 2 is clear, and the bare model is not buoyancy-only. The exception is written only in posix 2500:23-25, with a partial version at vehicles.md:235. Nothing says that 2502, 2503, 2506 and 2507 are permanently unused, so a reader of the CMakeLists list sees gaps that look like pending work. The fixed family's all-physics member is also named '(Added Mass, Hull Drag)' while the other two call theirs '(Complete)'.

Recommended: define the digit as a mask of the name's tokens, not the physics carried. Put the airship_fixed exception and the unused digits in the CMakeLists comment, and correct vehicles.md:223 and 225. This goes away if finding 1 is adopted.

### Wrong version badge on independent vectoring; the new sections break docs conventions
`docs/en/frames_airship/index.md:13` (scheme-docs-upstream)

frames_airship/index.md:13 marks 'Generic airship (independent vectoring)' as 'PX4 v1.18'. upstream/release/1.18 has no CA_AIRSHIP_* parameter (0 matches in module.yaml) and no flight 2520. Upstream main is PX4 v2.0 (releases/main.md), and new features there carry `<Badge type="tip" text="main (PX4 v2.0)" />` (for example config/motor_failure_recovery.md:3).

The new gz 'Airship' section (vehicles.md:218) and the LTA world section (worlds.md) have no badge; Ridge, for comparison, carries one. None of the 16 airship vehicle sections has an image, while 14 of upstream's 17 vehicle sections do.

Recommended: use the main (PX4 v2.0) badge, add one per new section, and add at least one image per vehicle.

### The lta world floats only instances 0-3; the docs do not say so
`docs/en/sim_gazebo_gz/worlds.md:50` (scheme-docs-upstream)

lta.sdf:177-264 enables base_link for `<model>` and `<model>_0` through `_3` only: 80 lines for 16 models. px4-rc.gzsim:116 spawns `<model>_<px4_instance>`, so a fifth vehicle from a multi-vehicle run (instance 4) gets no buoyancy force, with no warning, and falls.

worlds.md's LTA paragraph explains the `<model>_<instance>` naming but not the 0-3 limit. The list also grows by 5 lines per model, so every physics variant costs world edits (finding 1). Recommended: state the 0-3 limit in worlds.md, or the multi-vehicle page, and keep the enabled model set small.

### The design record that justifies the numbering is out of date, including its reason for moving 2507
`px4-airship-bench-tools/airship_family_naming.md:124` (scheme-docs-upstream)

Several statements were overtaken by ac41e6c, 6e74f20 and 52d183d06f:
- Line 80: the fixed airship's gz model is 'none yet'.
- Line 81: the independent airship's flight number is 2510.
- Lines 124-126: '2500 (reserved)' and '2522-2527 as the data arrives'.
- Line 133: 'so that the generic family's complete variant (mask 7) has its slot'.
- Lines 168-171: '2520 becomes Cloudship'.
- Lines 200-206: 'Decisions still open', building airship_fixed and the waiting _am/_fin members.

This is the document the scheme is defended from, and its main argument for 2519 is no longer true (see the 2507 finding). Recommended: update it, and record the no-fin-bit decision for the 2500 decade along with what that does to the 2507 argument.

### The yaw fit leaves out roll-coupled terms; its 3-5% residuals are deterministic bias, not noise
`px4-airship-bench-tools/added_mass_check.py:366` (verification-method)

The fit models I r' = K_s u v - K_d u r. In these plants three more terms act:
- The vertical fins' cps are 2.05 +- 1.363 m above the link origin, so roll rate adds -p z_cp to their sideslip.
- The _am block adds -(yp + xq) u p = +68.3 u p N m.
- ixz = 14.7 couples roll into yaw.
The linear model of the files reproduces the rigid _fin fit, 44.2/181.5 against the engine's 44.6/185.4. With the fin roll-rate term removed it returns 46.4/194.8, the file's 46.1/193.6.
The same omission explains the rigid roll period: the check predicts 1.671 s and the engine gives 1.629; with ixz the prediction is 1.630 s.
On the hold: it is not the source of the _am_fin shortfall (11.38 against 12.77). dK_s/dK_d is only -0.03, and the free fit reads K_s 12.85 with K_d 234.6, 27% above the rigid 185 (cond 2.5e3, corr 0.98). Holding K_d is sound physics, but it throws away the one symptom, the free K_d moving, that shows a regressor is missing.
Fix: add the integral of u p as a regressor (p is in the pose stream the check already parses), or fit the full linear lateral model. Add ixz to the roll prediction. After that the 15% tolerances can be tightened.

### simulate() leaves out the anisotropic added-mass Coriolis term in the transverse equation
`px4-airship-bench-tools/airship_fixed_flight_check.py:397` (verification-method)

Derivation: the body sway equation is (m+m_y) v_b' + (m+m_x) u r = Y, and the engine applies it through DART's spatial inertia (the same C_A that gives the measured Munk moment). With vt = v_b + u psi, the world transverse velocity obeys vt' = Y/(m+m_y) + (m_y - m_x)/(m+m_y) u r. The code has only force/m.
The missing coefficient is 0.39 on the generic _am plants. Adding it raises the predicted yaw peak by 4.4% (6.38 -> 6.66 deg/s at 3.36 m/s), moves the peak from 1.18 s to 1.37 s, and raises the pitch peak by 1.5%. The pitch path has the same omission ((m_z - m_x)/(m+m_z) u q).
The rest of the three-state model is correct: alpha = angle - (vt - l r)/u, and the Munk term is correctly signed.
The docstring (:42) says the rise peaks at 0.4 s. That holds on the rigid plants (0.48 s measured). On _am the measured peak is at 1.1 s (fixed_flight_airship_fixed_am_yaw_+1.csv).
This sits under the 7% agreement, but it must be fixed before the tolerance can be tightened.

### Post-stall drag is 1.5-1.7x its stated basis, and no check covers the stall parameters
`PX4-Autopilot/Tools/simulation/gz/models/cloudship_fin/model.sdf:39` (verification-method)

The comment says the post-stall slope puts cd 'near 1.2-1.3' at 90 deg. The plugin computes cd = cda*alpha_stall + cda_stall*(alpha - alpha_stall): 1.94 x 0.35 + 1.2 x 1.221 = 2.14 on the Cloudship, and 0.903 x 0.3 + 1.2 x 1.271 = 1.80 on the generic fins (airship_fixed/model.sdf:426-428 and the _fin files). A flat plate normal to the flow is about 1.2.
variant_check fact 4 checks cla, area, cp, upward and flap slope only, never alpha_stall, cla_stall or cda_stall. Post-stall drag is the fins' only in-place yaw damping, and it is visible in the gate records above.
Fix: cda_stall about 0.47 (Cloudship) and 0.73 (generic) for cd(90) of about 1.2-1.25, with those three parameters added to fact 4.

### The twin's allocation block is a copy of the flight 2500's, and nothing holds the two equal
`PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/2500_gz_airship_fixed:75` (verification-method), hardware-testable

CA_AIRFRAME, CA_AIRSHIP_* and CA_SV_CS0..3 appear twice:
- 2500_gz_airship_fixed:66-84
- 2500_generic_airship:28-44
They are identical today, but no fact in variant_check compares them. The flight check's surface-direction verdicts (stick back gives both elevators +25 deg trailing-edge up and nose up; right yaw stick gives both rudders +25 deg and nose right) verify the copy. They carry over to the flight airframe only while the two blocks stay equal.
Fix: either source a shared block or add an equality fact.
A pin measurement can confirm the link: load 2500 on the bench FMU, apply the same stick inputs, and compare the Servo1-4 PWM direction against the SITL servo_0..3 signs from the rest leg.

## Refuted

- 'Full potential-flow Munk is on the safe side' misreads Munk 184: with fins the hull moment is about equal to potential flow, so there is no hidden margin (aero-basis)
- Thrust does not fall with airspeed, so the twin's top speed and full-throttle flap authority are too high (fixed-twin)
- The ventral fin and the lower rudder reach below the gondola's ground-contact face (fixed-twin; this dimension's version only: the model-files version was confirmed as finding 38 and fixed)
- Stability is shown only by fitted coefficients; no released-sideslip test and no pitch test above the 4.2 m/s crossover (verification-method)
- The conservative margin (1.22 on Helmbold) depends on sizing against the smaller hull volume and on leaving out the fins' own added mass (verification-method)
- The engine's plant is stable in yaw only for sideslip under about 25 deg, then diverges to broadside (model-files)
- The recomputed tensor keeps a solid-ellipsoid hull; the Cloudship uses envelope shell plus gas (model-files)
- 16 posix airframes, 16 index rows and 240 make targets for three vehicles' physics ablations; PX4 already selects a model without an airframe (scheme-docs-upstream)
- The docs order contradicts the order the renumbering was done for (scheme-docs-upstream)
- The flight 2520 comment ships yaw defaults tuned on a plant that no longer exists and not on the one the docs say to tune on (scheme-docs-upstream)
- 52d183d06f's submodule bump changes existing plants without saying so, and breaks the branch's bump-commit pattern (commits)
- At 52d183d06f the scheme check fails because the docs come one commit later; fact 11 also has a substring false-pass (commits)
- The 'vectored rigid 8/10 (was 9/10)' baseline blames the tail for a check that flips on run-to-run noise (commits; this dimension's version only: finding 11, which adds the non-clean start, was confirmed and fixed)
- The generic tail's 1.2x margin is entirely the body's carry-over share, and the Helmbold check never tests that (gap:Cloudship fin lift basis (08be662): is the slender-body carryover achieved on a converging stern? Covers both vehicles, but mainly the Cloudship.)
- Cloudship cla 7.254 is sound in net only through a cancellation nobody wrote down; the quoted 1307 support is out of scope, and the tuning plant's fin damping sits at the top of the range (gap:Cloudship fin lift basis (08be662): is the slender-body carryover achieved on a converging stern? Covers both vehicles, but mainly the Cloudship.)
- The bench FMU has CA_AIRSHIP_CS_K 0.5 saved, so the proposed 2500 pin test would measure a P-only loop. The same setting silently turns 2500's yaw loop P-only in flight (gap:The 2500 path in rate modes: never run in SITL or at the pins)
- bench_yaw.py cannot measure 2500's yaw loop: it reads the port-motor pin, reports a false 'sticks never reached the FMU', and its holds are too short to reach the cap (gap:The 2500 path in rate modes: never run in SITL or at the pins)


After the yaw retune (d2647379af, AS_YAWRATE_P 0.7 + AS_YAWRATE_FF, 2026-09-28 21:25): Manual fingerprint
fpJ 0.000 us, 13 x 8, against run C and run H; parameters differ in exactly AS_YAWRATE_P (0.5 -> 0.7) and
AS_YAWRATE_FF (new, 0). Records head_20260928b/.
