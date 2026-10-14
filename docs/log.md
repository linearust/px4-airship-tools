# Airship bench and equivalence tools (kept outside the PX4 tree)

Which tool answers which question:

| question | tool | needs |
|---|---|---|
| does this firmware still put out the same pulses? | `bench_fingerprint.py`, `identity/` | the FMU |
| does the yaw loop fly? | `gz_run.sh ./sitl_check.py` | gz |
| is the hull's drag written about the right point? | `aero_moment_check.py` | nothing, or gz for the plant |
| is the Cloudship's body the vehicle it claims to be? | `cloudship_geom_check.py` | nothing |
| does the gz Cloudship come up and fly at all? | `GZ_TARGET=gz_cloudship gz_run.sh ./cloudship_first_light.py` | gz |
| do the tilt servos reach the angles the airframe says? | `gz_run.sh ./tilt_range_check.py` | gz |
| does the Cloudship's tilt vector its thrust? | `GZ_TARGET=gz_cloudship gz_run.sh ./cloudship_tilt_check.py` | gz |
| does the mid-air disarm mitigation hold, and does the bug come back without it? | `GZ_TARGET=gz_cloudship gz_run.sh ./cloudship_land_disarm_check.py` | gz |
| is the Cloudship's flight airframe wired and mapped the way it claims, at the pins? | `cloudship_ground_check.py [--drive]` | the FMU on 2507, Logic 2 |
| what does one output function put on every probed pin? | `ground_drive.py FUNCTION V1,V2,.. NAME` | the FMU, Logic 2 |
| did the tilts slew at CA_AIRSHIP_TLT_R, and what did the pins do at the disarm? | `fp_tools/ramp.py FP_DIR [S,P]`, `fp_tools/park.py FP_DIR` (a fingerprint run's export) | nothing |
| are the physics variants what the naming scheme says? | `variant_check.py` | nothing |
| what does the yaw rate loop do on a surface-only (2500) allocation? | `bench_rate_hold.py [--loop]` (refuses unless the 2500 allocation is live: CA_AIRFRAME 16, two equal yaw-only rudders on CS2/CS3; CS_K as wanted; `--loop` predicts the closed loop of firmware before e550ec6597) | the FMU, Logic 2 |
| does the .px4's airframe metadata (QGC's airframe picker) match the ROMFS the image boots? | `build_fmu.py` (build, check, stage to `fw_<commit>/`) or `build_fmu.py --check` (the existing build only) | the PX4 tree (and the toolchain unless `--check`) |
| does the added mass do in the engine what the file says? | `added_mass_check.py [--vehicle <v>]` | gz (its own probe worlds, no px4) |
| does the 2500 twin (fixed thrusters, fin flaps) fly and answer its sticks? | `GZ_TARGET=gz_airship_fixed gz_run.sh ./airship_fixed_flight_check.py` | gz |
| what does a negative SIM_GZ_EC_MIN actually do? | `gz_run.sh ./reverse_thrust_check.py` | gz |
| why is the tail pushing one way only? | `GZ_TARGET=gz_cloudship gz_run.sh ./tail_probe.py` | gz |
| does the yaw integrator park in forward flight, and what does the release do? | `GZ_TARGET=gz_<model> gz_run.sh ./forward_yaw_check.py` (`--plan` needs nothing; `--analyze <ulg> --events <json>` re-measures a record) | gz |
| does the yaw model fly a recorded forward leg the way SITL did? | `yaw_model/forward_replay.py sitl_out/forward_<model>_<stamp>_summary.json [--start both]` | nothing |
| which forward-flight yaw policy holds up against speed-source and k errors? | `yaw_model/policy_compare.py` (record: `yaw_model/step0c_out_20261001.txt`) | nothing |

## head_d020cc4c40/ — the preparation branch at the pins (2026-10-14, fmu-v6x, pr/airship d020cc4c40 vs airship/dev 8af72b9a50)
Same board, same saved parameters, two firmwares, `bench_fingerprint.py` (13 stick vectors x 8 pins, props off, ESC power off):
- `fpM_d020cc4c402`: the preparation branch (upstream main 97a3c7f017 + 7 commits). `fpN_8af72b9a506`: the development branch,
  flashed right after. `fp_tools/fp_diff.py` fpM fpN: max |diff| 0.0 us, 0 of 13 vectors differ, exit 0 (`fp_diff_fpM_fpN.txt`).
- The map with no surfaces: tilts on Servo 1/2 = MAIN1/MAIN2, motors on AUX1/2, MAIN3-6 at 1500 throughout. fpL (10-07, airship/dev
  d51ed755cb) had the tilts on MAIN5/6 and its yaw vectors split with two rudders, because the board still stored CA_SV_CS_COUNT 4
  from the 2500 experiments of 09-25; `fp_diff.py` fpL fpM says CONFIG MISMATCH, as it should (`fp_diff_fpL_fpM.txt`). The count
  was set to 0, the 2520 preset's default, before the fork run, so both runs are the preset. `fp_diff.py` now compares the narrowed
  class's CA_SV_CS_COUNT as 0 whatever the board stores: that class never reads it (the t1 snapshot lists it as unused).
- `fp_tools/ramp.py <dir> 0,1`: 15 slews on each run, all at CA_AIRSHIP_TLT_R 120 deg/s (180 deg in 1.5-1.6 s), 0 off.
- `fp_tools/park.py` (new): at the disarm the one tilt away from rest (MAIN2, 1613 us) is back at 1500 within 0.3 s through the rate
  limiter (the controller publishes a zero wrench when disarmed); the motors drop from the armed idle 1100 to PWM_AUX_DIS 1000 in the
  same 0.1 s window. Same on both firmwares.
- Tilt end stops, `ground_drive.py 201 -1,1,0` and `202 -1,1,0` (disarmed actuator_test, PR firmware): 1000 / 2000 / 1500 us held
  8.0 s each on MAIN1 and on MAIN2, every other pin flat (`stops_servo1.log`, `stops_servo2.log`).
- Parameters. t1 (PR firmware, after the flash) against the 10-07 end state: no shared value moved but CAL_BARO0_OFF; 72 names gone,
  10 the fork alone defines (AS_SPD_*, AS_YR_FF_SLP, AS_YR_RESTORE, CA_AIRSHIP_GRP/TAIL/CS_K) and 62 upstream control-surface and
  airspeed-calibration parameters the narrowed class never reads, which MAVLink therefore does not list (`param status` 1022/1563
  used/total); 1 new, UAVCAN_EC_SIGNED, upstream's since the fork's base. t3 (fork firmware) brings 69 back (AS_SPD_LO and
  AS_YR_RESTORE no longer exist), CA_AIRSHIP_CS_K at its default 1.0: the saved 0.5 went with the PR firmware's first save, and 1.0
  is the value fp_diff assumes for an absent CS_K. t2->t3 and t3->t4: volatile only.
- Files: `t0/t3_identity.log` (`ver git`), `t1..t4` snapshots and `snap_diff_*.txt`, `fpM.log`/`fpN.log`, the two `fp_diff_*.txt`,
  `ramp_*.txt`, `park_*.txt`, the end-stop logs and captures. The board stays on airship/dev 8af72b9a50; both images are staged.
- Footprint, `arm-none-eabi-size` on a scratch worktree of upstream main 97a3c7f017 (submodules referenced from the fork's modules dir,
  ccache) against the preparation worktree at d020cc4c40: fmu-v6x_airship text 1,725,208 -> 1,732,128 (+6,920), data 7,732, bss 106,812
  unchanged; micoair_h743-aio_default (the indoor vehicle's board, airship_att_control on by default) text 1,865,480 -> 1,871,696 (+6,216),
  data 6,540, bss 61,016 unchanged; both targets make exit 0 at d020cc4c40. The fork HEAD 8af72b9a50 fmu-v6x_airship: 1,737,376 / 7,732 / 105,660.

## Parametric families: one vehicle file in, a whole family out (2026-10-07)

**Literature families and the 7 m generic (2026-10-08, overnight_plan_20261007.md).** The generic family is a
7 m outdoor hull of the Aero Drum 7 m RC blimp class (7.0 x 2.0 m, 14.66 m^3, 18.0 kg, 10 N pods); beside it,
fixed families of published vehicles, each a vehicle file: `airship_aurora` 2540 (AURORA / AS800, de Paiva 2006),
`airship_jaxa` 2550 (JAXA 14 m, Kohno 2009), `airship_mpi` 2560 (the MPI 5 m blimp, Price 2022, dimensions read
from its open Gazebo model). `lotte_added_mass_check.py` holds the Lamb added mass on a 107 m^3, fineness-4
spheroid against Li & Nahon 2007's CFD for the Lotte: sway 1.006, surge 0.82, pitch/yaw inertia 1.23 (report only).
So a family's rr, and the P and I the bandwidth rule derives from I + rr, may run some 20 % high on a real hull
(the added inertia is 40-65 % of the total here): a tuning item for the vehicle, not a generator change (2026-10-12).
The generator now also writes the fixed twin's thrusters and flap servo range, and a vehicle file's `docs_note`.

**AS800-class generic (2026-10-07 evening, generic_as800_regeneration_plan_20261007.md; superseded the same
night: the generic is 7 m and this hull is the `airship_aurora` family).** The generic hull was
34 m^3 at fineness 3.5, the AURORA / Airspeed AS800 envelope (de Paiva et al. 2006) as a spheroid, 9.27 x
2.65 m, 42.6 kg (gondola 30.9 kg keeps the envelope at 0.1312 kg/m^2); fins 1.54/1.21 m, flaps +-10 deg, pods
unchanged. `generic_hull_design.DEFAULT` and `vehicles/airship_generic.json` agree; the length scaling of the
turn speeds and pod inertia is anchored at `SCALE_REFERENCE_LENGTH` (10.97 m) so the indoor family did not move,
and the generic pods state their radii of gyration. Gains from the calculator: P 0.421, I 0.0541, FF 0.609
(complete) / 0.205 (fins), slope 2.89 measured (seed 2.82), ramp 1.4/2.9. `yaw_model/policies.fin_slope`
refuses an engine record whose measured inertia is another hull's (the generic's record is the 10.97 m hull's
until `added_mass_check.py` is rerun).

The generic family's design script now takes a vehicle instead of module constants, and a generator
makes a complete gz family from one JSON file. The committed generic family is the proof: regenerated
from its own file it comes back byte for byte (the two pod comments excepted, where a stale 579 N m/rad
and a 2e-04 spelling were corrected).

- `vehicles/<name>.json`: the hull (`generic_hull_design.HullSpec`: volume or length and diameter,
  fineness, hull height over the gondola floor, gondola box and mass, pods' station and mass, fin chord,
  margins, areal density, stability target), the pods (link and prop mass, thrust per pod, rotor speed,
  tilt range and rate), the names and the airframe decade. `airship_generic.json` is the committed
  family; `airship_indoor.json` is the 2 m indoor validation vehicle, its numbers marked ASSUMED where the
  build has not fixed them yet.
- `generic_hull_design.py [--spec FILE]`: the design from a spec, DEFAULT being the generic airship. The
  R3 turns and the envelope are judged at the generic speeds scaled by hull length.
- `apply_generic_hull.py`: the writers take a plan (spec, `Family`, models root); `main()` is the generic
  family in place, as before. Every output comment is formatted from the design, so a new vehicle gets
  the same derivations with its own numbers.
- `airship_family_gen.py vehicles/X.json [--check]`: copies the generic family's files as templates,
  renames them, writes the design and the pods into them, and appends what the tree carries besides:
  the lta world's buoyancy entries, the decade's posix airframes (members as includes; an existing member
  keeps its gains), the posix CMakeLists entries and decade line, the docs sections, table rows and list
  bullet. Idempotent. `--check` regenerates into a scratch copy and diffs it against the tree.
- `airship_gains.py VEHICLE [--write] [--params FILE] [--measure SUMMARY...]`: the yaw loop's values from
  the model files, by giving the vehicle's complete plant the generic complete plant's loop bandwidth
  N P / I and integral time P / I of its anchor, `REF_PLANT` (P 0.7, I 0.09 on 54 N m and 129 + 192 kg m^2,
  the 10.97 m generic hull, frozen as constants when the generic moved to the AS800 class); the feedforward from
  the hull's rotational drag at AS_YAWRATE_MAX, which the firmware applies as the setpoint's share of that rate
  since 2026-10-12 (the drag is quadratic in the rate; the ramp's cap rule, the positive root of
  FF0 r^2 / r_max + k u r = A - rho, follows). The schedule's slope is seeded by what holds a steady turn: the hull
  sideslips until the fins carry the centripetal force, so k = (m + m_x) x_bar - (m_y - m_x) s with s the
  sideslip per r/u (the fins' rotational damping is not it: +9 % on the generic, -21 % on the indoor
  hull). `--measure` replaces the seed with the held-turn torque over u r from forward_yaw_check
  summaries (simulator or hardware) and stores it in `vehicles/<vehicle>.measured.json`, which later
  runs read back while the plant's mass, fin arm and authority stay within 1 %. The ramp ends where
  the capped yaw rate meets 0.8 of the rate that stalls a fin in a turn, its incidence the sideslip plus
  its own rotation, and starts at half that. `--write` puts them between markers in the bare, `_fin`
  and complete airframes; `--params` writes the FMU set for the hardware preset.
- The checks follow the files, not a table: `variant_check.py` reads each body's stability promise from
  its header and recognises `<vehicle>_base` bodies; `sitl_check.py` lists the indoor vehicle's authority
  (relative tolerance against the model); `forward_yaw_check.py` takes every model with a `_fin` token
  as a plant and flies one not in its table at the middle of its airframe's ramp and at 1.6 times the
  ramp's end (where the 10.97 m generic's 2.0 and 4.0 m/s sat on its 1.5 to 2.5 ramp; a length scale put the
  indoor legs below the ramp); `gz_model.family_tokens()` serves them.

On the indoor vehicle as assumed: fins sized by the stability rule come out 0.62 m (dorsal and sides) and
0.22 m (ventral) at a 0.22 m chord; the held-turn slope over the pods' authority is 15.4 per m/s (measured
in SITL; the seed said 15.1) against the generic's 2.9 (the 10.97 m hull's 4.2), so at 1 m/s the pods cannot hold a 15 deg/s turn and
the schedule caps the rate near 3 deg/s there (3.9 at the ramp's end 0.78 m/s); the pods sit 0.3 m below
the centre of mass, so full thrust pitches the nose up about 10 deg against the 0.9 N m/rad pendulum. All
three follow from the assumed hull and should be reread once the envelope and the gondola's installed
mass are known.

## Forward flight, D1 step 0 (2026-10-01)

The first SITL measurement of the forward-flight yaw defect on HEAD 4e1baea97c, with no firmware change; the owner's
report is `step0_report_20261002.md`, every leg's numbers `sitl_out/forward_results_20261001.md`. On 2527, 2522 and
2512 a full stick parks the integrator at AS_YR_INT_LIM with `torque_setpoint_achieved` 1 and no flag, and the
release turns on until I/K_I is spent (191 deg from the limit, whatever the speed). The model reproduces the legs
(`yaw_model/forward.py` + `forward_replay.py`); `policy_compare.py` compares the D1 schedule, the restore rule and
both. Three things a forward run needs that the at-rest gate never did:

- **Logs.** SITL's rcS sets SDLOG_DIRS_MAX 7, and a logger starting with more date folders deletes the oldest;
  the yaw model validates on the 2026-09-25 and 2026-09-27 logs. `gz_run.sh` now exports
  `PX4_PARAM_SDLOG_DIRS_MAX=0` unless the caller sets it, and `forward_yaw_check.py` refuses to fly without it.
- **Height.** The Cloudship trims about 2.5 deg nose down and sinks about 15 m per forward leg; spawn it with
  `PX4_GZ_MODEL_POSE=0,0,100` (the lta world's buoyancy is a uniform density, so height is physically neutral).
- **The unstable plant.** `cloudship_am_fin_drag` (2517) is directionally unstable in forward flight even with the
  loop closed at HEAD gains: the check trims it in Acro, a full stick departs into a stalled spin, and after a turn
  at 2.5 m/s the entry may never become steady again (the check then refuses the leg, as it should).

Measuring a force on a hull whose drag is 35:1 between axes needs three things that are easy to
skip, and each of them produced an angle that passed while the magnitude was wrong by tens of
percent: rotate into the body frame *before* applying any coefficient, average `c v|v|` over the
window rather than evaluating it at the mean speed, and report the magnitude at all so the first two
can be caught. `cloudship_tilt_check.py` recovers the pods' 7.946 N to within 3% at tilts of -90, 0
and +90. Details in `cloudship_sitl.md`.

## The generic family, engineered (2026-09-24)

The generic hull's tail was a drawing; `airship_tail_design.md` sizes it (Munk NACA 184, slender-body
carryover NACA 1307, joined-panel AR, 30 % flaps) so the family could exist: `airship_base` redrawn with
four fin plates and its inertia recomputed, `airship_base_am` (Lamb about the link origin), the vectored
`_fin/_am/_am_fin/_am_fin_drag`, and `airship_fixed` = the 2500 flight airframe in gz (fixed thrusters, the
fins WITH flaps as elevators/rudders on `servo_0..3`, so no `_fin` member) with `_drag/_am/_am_drag`.
Airframes 2500/2501/2504/2505 and 2522/2524/2526/2527; the Cloudship's fins moved to the same basis (cla
1.856 -> 7.254). The basis is `fin_basis.py`: the flat-plate slope 2 pi AR / (AR + 2) on the panels joined through the
hull (what Selig 2015 measured for AR 2-3 plates at this Re) times the slender-body carryover, K_W(B)
alone for a flap (a review verifier solved the slender-body problem numerically: k_W(B) + k_B(W) = K_W(B));
Revision 3 (2026-09-25, `generic_hull_design.py`, written into the models by `apply_generic_hull.py`): one
10.97 x 2.61 m spheroid of fineness 4.2 is drawn, collided, lifted and sized on alike; dorsal and side fins
1.52 m, ventral 1.28 m (the ground at level attitude); the yaw requirement is judged on Helmbold's
attached-flow slope, 1.203 x the Munk moment there and 1.41 x on the plate secant the plugins carry; a fin
stalls where its own incidence (body angle times the hull's upwash K_W(B)) reaches CLmax 0.7, and the flaps
travel +-11 deg so a full-rudder turn on the added-mass plants stays 2 deg below that; below the stall the fins'
drag slope is sized to the induced drag at the stall (2026-09-27; at 10 deg before). `variant_check.py`
knows `_base_am` wrappers, actuator fins, per-hull drag blocks, the upwash stall, the servo range against
the joint limits and each gz twin's allocation block against its flight airframe (the count is in its tool
entry below); `added_mass_check.py --vehicle` runs on any family. `apply_generic_hull.py` refills the comment
paragraphs it writes, so a changed number never leaves a ragged line; three runs give identical files. Since
2026-09-27 it also writes the words that carry a design input (fineness, margins, flap fraction, the drag
reference hull) from the design's constants, refuses an element that lacks the child it writes, and writes
nothing on import.

Checks tightened the same night: `airship_fixed_flight_check.py` judges every rate to 10 % and holds full
rudder 10 s (leg 6b) to see the turn each member keeps, with a 0.1 deg/s floor on that leg that binds only on
the rigid and drag members (0.3 until 2026-09-27). Since 2026-09-27 its model takes each fin's cl and cd from
the lift-drag plugin's own curves, stall included (`fin_basis.plugin_cl`/`plugin_cd`): the 2026-09-25 records,
replayed on the fin drag they were flown with, match its onset peaks within 2.5 % in yaw and 4.5 % in pitch and
its held turns within 0.1 deg/s or 0.8 %, and without the Munk moment it would predict 21 % less yaw rate and
11-12 % less pitch rate. On the current fin drag, the 2026-09-27 run on the current fin drag (sitl_out/chain9_20260927.txt, records in sitl_out/fixed_flight_20260927_chain9/) passed all four flight twins 13 / 13, the held turns within 0.02 deg/s of the model (rigid -0.03 vs +0.01, drag +0.15 vs +0.17, am +1.89 vs +1.90, am_drag +1.90 vs +1.91 deg/s at the last second's speed), and the added-mass check 9 / 9; the gates kept their baselines: Cloudship 9 / 9, vectored rigid 9 / 10 (the known Acro brake, 27.7 -> -7.6 deg/s), vectored _fin and _am_fin_drag 8 / 10 (release 1.0 and 2.7 deg/s still falling at the window's end), Cloudship _am_fin_drag 8 / 9 (Acro peak 9.54 on 6.67 from rest). `sitl_check.py` starts each tracking step at rest with the integrator cleared through Manual, so
an overshoot is the loop's own and not the brake phase's; since 2026-09-27 a step whose start did not settle
within 60 s, or was not read, fails.

Since 2026-10-12 `sitl_check.py` takes its time windows and the drift floor from the plant it flies instead of
the anchor's constants: the tracking window is four P-only time constants I / (N P) of the plant's own yaw
inertia (rigid, or with the added inertia of an _am member), the released hull has to settle inside
1.25 tau ln(sp / 1 deg/s) plus one pod reversal at CA_AIRSHIP_TLT_R, and a push with the integral off has to
drift at half the P-only equilibrium torque / (N P + k_fin u) at the drift u the hull has. The tracking window
is four of the slower of tau and the integral time P / I, the loop is judged against the setpoint the firmware
published (the schedule caps a drifting hull below the ramp at reserve / (k u), and the verdict writes the cap and
the interval down), a hull with drag is first brought below the drift at which that cap would clip the step,
reserve / (1.1 k sp), and the first wrench is republished once when the simulator drops it. A drag-free _fin
member keeps its drift for good, so its hover run is informational: its verdicts come from the forward check.
The anchor's 35 s, 10 s and 1.0 deg/s were its complete
plant's 8.5 s helped by its drag, plus 1.5 s of pod reversal; on the JAXA plant's 60 deg/s servos the
release read 10.1 s and the I-off drift 0.75 deg/s, a slower vehicle and not a loop defect
(professor_decisions_20261012.md D-f).

The SITL gate's wind check (`sitl_check.py`) now expects what 1 N m for 5 s does to the plant it flies:
rigid plus added yaw inertia against the `_drag` member's yaw damping, integrated. A fixed 1.5 deg/s floor
failed the added-mass plants (2.4 x the inertia), and an undamped expectation failed the Cloudship's drag plant.

Three measurement lessons from the day, in `measure-where-the-quantity-is` terms:

- **one regressor where the plant has two.** r against ∫uv alone read the generic fins at a fifth of
  their value: this tail weathercocks in I/(K_d u) = 0.3 s, the rate saturates inside the push and the
  saturation was read as weakness. Fit ∫uv AND ∫ur on the rigid `_fin` (37.4/38.9 static, 155/163
  damping), and on `_am_fin` HOLD the damping the rigid hull just measured, because on a statically
  unstable plant (the Cloudship: fins restore half its Munk moment) the two integrals grow together and
  the free pair put its damping at 6.2 for 3.9.
- **a first flight that skipped buoyancy.** `airship_fixed` dropped the vectored pods' 0.4 kg links and
  kept the hull box sized for them: 0.8 kg light, rising at 0.16 m/s^2 on a drag-free plant, and the
  "at rest" stick tests were taken in a 3 m/s updraft over the fins (a rudder at 25 deg in a vertical
  flow pushes sideways). The flight check now asserts buoyancy first, waits for rest and says so, and
  reads the rotor speed on every stick.
- **a 4 s pitch angle is not an elevator trim.** On the day, 4 s of "trim" read +2.8 deg nose-up with
  the stick forward (`sitl_out/fixed_flight_run1.txt`, the light plant above). The elevator's own lift
  makes heave: on the revision-3 records (2026-09-25) full elevator tilts the path 3-4 deg on the
  rigid and drag members, downward for stick back, while the pendulum brings the pitch back to about
  1 deg of level; only the added-mass members hold a pitch, about 2-3.5 deg across the chain5-8 readings
  at 4 s, the Munk moment against the pendulum.
  The check judges the peak rate of the first second against a three-state simulation of the file's
  numbers and reports the angle without judging it.

## One world, forty-five targets (2026-09-18)

The build generates `gz_<model>` for the default world and `gz_<model>_<world>` for every other, so
the three airship models have 45 make targets between them. Three of them work. Only `lta`,
`underwater` and `kth_marinarium` carry a Buoyancy plugin, it is a WORLD plugin, and it lifts only
the links a world names - so `gz_cloudship_windy` spawns an airship with no lift and it falls, with
no warning anywhere.

`gz_run.sh` now refuses before it boots anything, with its own exit code so a driver script can tell
the three apart:

| | |
|---|---|
| 7 | the check file does not exist |
| 8 | the model does not exist under `models/` |
| 9 | the world has no Buoyancy plugin, or its `<enable>` list does not name this entity |

It also splits the target against the worlds that exist rather than subtracting a guessed suffix,
which is the same data CMake used to generate the names. That is what makes `gz_cloudship_lta` and
`gz_airship_vectored_independent_drag_lta` resolve at all; the old form turned the first into a
model called "cloudship_windy" and refused it for the wrong reason.

After the boot it exits 10 when px4 attached to a gz world it did not start: `px4-rc.gzsim` takes the
first `/world/*/clock` it finds, and the checks would address `/world/$PX4_GZ_WORLD`.

Verified per model and per world: `gz_cloudship` and `gz_cloudship_lta` give the same 7/7, the aero
airframe gives its documented 8/10 and 8/8, and the three refusals above fire before the make.

A negative `SIM_GZ_EC_MIN` does not make a gz motor reversible. The parameter store keeps it -
`param_info_s` carries no bounds - but `MixingOutput` holds the range in a `uint16_t` and then swaps
the pair because min now exceeds max, so -300 against a max of 300 reaches gz as the range
[300, 65236] and the three commanded points land on 300, 32768 and 65236. Measured, twice, on two
vehicles. The route that works is PX4's own `GenericMotorModel` in `duty_cycle` +
`bidirectionalMotor` mode, which the Cloudship's tail now uses; `cloudship_sitl.md` has it in full.

## The four modules the SITL checks are built on (2026-09-18)

A full review found 43 confirmed defects across these files, and the dominant one was not in any
single check: five copies of `verdict()`, five of `px4()`, three of the motor-speed reader with
three different hard-coded motor counts, two of the SDF parser, and gravity written three times with
three different values. So there are four modules now, split by what a check actually touches:

| | |
|---|---|
| `sitl_link.py` | a running px4: clients, parameters, sticks, actuator holds, `finite()` |
| `gz_link.py` | a running gz: topics, poses, motor speeds, wrenches |
| `gz_model.py` | a model.sdf, with no simulator: geometry, drag terms, thrust curves, duty ranges, and the lta world's G and RHO_AIR; since 2026-09-27 also the centre of volume, the lift-drag fins and the servos on their joints, each vehicle's yaw authority, and the airframe directories |
| `report.py` | verdicts, flushed, with one convention, and one exit rule |

Two design bases are shared the same way: `fin_basis.py` (the fin lift basis, and since 2026-09-27 the
lift-drag plugin's own cl and cd) and `hull_basis.py` (2026-09-27: Lamb's coefficients and the added-mass
block about the link origin). The design, `apply_generic_hull.py`, `variant_check.py` and the flight check
take their formulas from these rather than from copies.

The second pass (2026-09-18, after 50 architectural proposals and 63 "leave this alone" notes) took
out what the first one left: three copies of the bridge's bounds check, three centre-of-mass
calculations, an `ok &=` chain threaded through fourteen sites in the moment check, a printer
parameter carried through five signatures with one implementation, and two knobs that could only do
harm - a `wrench(entity=...)` that aimed at a link the message type could never match, and a
`PX4_GZ_MODELS` override that moved the tree the CHECKS read while `gz_env.sh` kept the simulator
on the other one. Code went from 1417 to 1355 lines and the comments from 1017 to 1077, which is
the trade that was wanted: the comments are where the defects are recorded.

Two things it deliberately did NOT do. The gz-side modules still take the entity name from
`sitl_link`, because that name is `px4-rc.gzsim`'s construction and moving it into "a model.sdf
reader" widens that module's charter for no behaviour change. And `aero_moment_check.py` stays one
606-line file: it is 45% prose, `--analytic` already imports nothing but stdlib and `gz_model`, and
its two halves are two tests of one derivation.

Two classes of defect ran through everything and are worth naming, because they are what the split
is for.

**A failed read looked like a reading.** `shell()` swallowed the timeout and returned '', the caller
turned that into NaN, and every guard was a strict comparison - `abs(rate) > 2.0`,
`abs(torque) < 0.02` - which NaN fails. So "could not read" arrived at the verdict as "within
tolerance": the gate's own comment called that "the most believable wrong answer this script can
give" and its guard was written in the one form NaN walks through. A read that did not happen now
returns None all the way up, `finite()` is there for the guards that must insist, and the one
threshold that has to fire on NaN is written as a negated `>=`.

**A number restated is a number that stops describing the vehicle.** The gate carried copies of
AS_YAWRATE_MAX and MAN_DEADZONE; the tail check carried a flat 10 deg/s floor where the model's own
thrust curve implies 32; the buoyancy check carried a drift tolerance the vehicle could not meet.
Every one of them is now read from the model or from the running firmware.

`sitl_link.py` is the one way in to a running SITL, `fmu_link.py` the one way in to the bench FMU.
`python3 sitl_link.py` checks its own parsing against the shapes `param show` actually prints.
Everything else is below, and the `.md` files carry the findings the checks exist to defend.

- `fmu_link.py`: the one MAVLink link to the bench FMU. The by-id port, the baud, the source ids, the priming
  heartbeat the USB autostart needs and the retry live here, because three scripts each carried their own copy and
  they had already drifted (six attempts against eight, two seconds against three). reboot_fmu.py still asks for
  `connect(attempts=8, pause=3)`, which is the one difference that was deliberate.
- `bench_yaw.py`, `bench_fingerprint.py`, `la_mcp.py`: Pixhawk + Logic 2 bench runs (props off, QGC closed).
  `baseline/` holds the 2026-09-09 captures from airship/dev 73909c9c19 (fingerprint 13 vectors x 8 pins, yaw run, SITL gate).
  Three bench facts learned 2026-09-23: pymavlink can throw a TypeError from inside `wait_heartbeat`
  when the port is opened mid-stream (twice in four opens that day), and a fresh connection a moment
  later reads the same stream cleanly, so every link now goes through `fmu_link.connect()`'s retry;
  Logic 2 is another process whose cwd is its read-only AppImage mount, so every path handed to it
  must be absolute; and the FMU sends the virtual `_HASH_CHECK` at index 65535, which is not in
  `param_count`, so `snapshot_params.py` counts announced indices only.
  Since 2026-09-27 `bench_fingerprint.py` and `bench_rate_hold.py` drive the FMU through `bench_yaw.Fc`
  (all four sticks, the FC's main mode from its heartbeat), and `bench_yaw.restore()` zeroes every stick,
  disarms and puts COM_DISARM_PRFLT back even when a run is interrupted right after that parameter was
  cleared.
- `cloudship_ground_check.py`: the Cloudship's flight airframe (2507) on a real FMU, at the pins, with
  nothing armed. Read-only by default: the 11 airframe parameters read back as 2507 sets them (it
  refuses on leftovers from a previous airframe, which an airframe file's set-default can never
  beat -- on the bench CA_SV_CS_COUNT 4 survived the 2510 -> 2507 switch), each of Motor1-3 and
  Servo1 on exactly one probed pin read from PWM_*_FUNC, the tail's disarmed pulse at the centre of
  its range, and the allocator at four actuators with Motor3 reversible. `--drive` then holds
  Servo1 and Motor3 at -1 / +1 / 0 through actuator_test and requires MIN / MAX / centre on the pin
  for the full hold, with every other pin flat at its disarmed value. Bench result 2026-09-23 on
  81f72e4920: 8/8, tilt 1000 / 2000 / 1500 us and tail 1000 / 2000 / 1500 us, each end 8.0 s. The
  matcher is mutation-tested on the saved capture: a non-reversible want, a non-reversible trace,
  a cut hold, swapped ends and a 6 us offset all read as fewer plateaus than wanted. Pass the
  vehicle's own probe map as `GROUND_PROBES`.
- `tail_rtf_probe.py`: real-time factor at rest and under a held tail command through px4, with the
  yaw-rate slope on the simulator's clock, judged by nobody. Written when one first-light run on
  cloudship_am_fin read the tail at a third of its torque with the simulator advancing 10 ms per
  round trip; the probe found RTF 1.00 and +21.3 / -20.9 deg/s^2 on that plant against +24.8 /
  -25.8 on cloudship_am, the difference being the fins' own yaw damping (released at 118 deg/s the
  fins decay the rate at 8.1 deg/s^2; the finless hull holds it). The failing run was a transient
  crawl of the simulator, not the model.
- `ground_drive.py`: the hand-run half of the above, kept because it answers the open question for
  any function on any pin without a verdict in the way. It is also where the plateau segmentation
  lives.
- The bench FMU and the 2026-09-24 renumber: the board's flight airframe 2510_airship_vectored_independent is
  2520_airship_vectored_independent from 965e8dd44c on. Firmware from there has no 2510, so the first boot finds no
  airframe for a stored 2510 and rcS resets SYS_AUTOSTART to 0 (no airship apps, parameters otherwise intact). Done
  on the bench on 2026-09-25, after the user flashed 9535f8abcd, with `head_20260924/renumber_fmu.sh`
  (`apply_params.py SYS_AUTOSTART=2520`, `reboot_fmu.py`, read back 2520 and 8 actuators, snapshot); any other board
  still on 2510 needs the same one command. Nothing else changes: the file is the same file, so no saved value is
  touched, and the pin fingerprints carry no number. Snapshots from before carry 2510; that is history.
- `head_20260923/`: the bench at HEAD 6983e120d8 -- run A on the board's 81f72e4920
  (`fpA_81f72e4920`, 0.0 us against the 2026-09-17 reference, 0 of 13 vectors differ, every tilt
  ramp on time), the HEAD .px4 the user then flashed (`ver_after_flash.log`, `post_flash.json`
  differs from the pre-flash store only in CAL_BARO0_OFF), run B on it (`fpB_6983e120d8`, 0.0 us
  against run A, 0 of 13 differ, ramps on time: the twelve commits since 81f72e4920 change nothing
  at the pins), and the ground-check snapshots: `ground_before`
  (2510), `ground_2507` (the switch changed exactly GRP, TAIL, TLMIN, TLMAX, R_REV, SYS_AUTOSTART),
  `ground_after_drive` (plus the two bench writes CS_COUNT 0 and PWM_MAIN_FUNC2 103),
  `ground_restored` (back on 2510: only CAL_BARO0_OFF differs from `ground_before`).
- `head_20260925/`: the bench after the user flashed 9535f8abcd and `head_20260924/renumber_fmu.sh` moved
  SYS_AUTOSTART to 2520 (`head_20260924/after_renumber.json`: only SYS_AUTOSTART differs from the 2510 store, plus
  runtime values); props off, ESC power off throughout. T0 `t0_identity.log`: 9535f8abcd, ROMFS 2500/2507/2520,
  SYS_AUTOSTART 2520, 8 actuators. Run C (`fpC_9535f8abcd`, `bench_fingerprint.py`): 0.000 us against run B on all
  13 vectors x 8 pins, run parameters identical; the renumber and everything since 6983e120d8 change nothing at the
  pins. T1 `fpD_2500live_9535f8abcd`: the 2500 allocation applied live, Manual, 0.0 us against preset2500 run C on
  the six pins both captured (CA_AIRSHIP_CS_K 0.5 and TLT_R 120 here, 1.0 and 0 there; `t1_before/after.json`
  differ in runtime values only). T2 `t2_rate_hold_9535f8abcd` (`bench_rate_hold.py`, CS_K 1, Stabilized): half
  stick 1532 -> 1679 us at 5.23 us/s, released 1650 us held, full stick 1715 us, the 43 % cap e550ec6597 removes.
  Restore E `fpE_restore_9535f8abcd`: 0.000 us against run C.
- `head_20260926/` (runs of 2026-09-27, props off, ESC power off): the user flashed 1965de11ab, whose image,
  staged in `fw_1965de11ab6/`, passes `build_fmu.py --check` (Airship group 2500/2507/2520). T0 `t0_identity.log`:
  1965de11ab, ROMFS 2500/2507/2520, SYS_AUTOSTART 2520. Run F `fpF_1965de11ab`: 0.000 us against run C, 13 x 8,
  run parameters identical. T2 `t2_rate_hold_1965de11ab` after `t2_apply.log` (GRP 0, TLMIN/TLMAX 0, CS_K 1): half
  stick 1750 us held, released 1500, full stick 2000, the FC in Stabilized by its heartbeat, so Stabilized equals
  Manual. Restore (`restore_apply.log`, `restore_boot.log`): `before.json` and `after.json` differ in the 3 runtime
  values only. Run G `fpG_restore_1965de11ab`: 0.000 us against run C, run parameters identical. After these runs
  `build_fmu.py` began to print the uploader command for the checked copy in `fw_<commit>/` rather than
  `make ... upload`, whose rebuild can repack the .px4 around a stale airframe list.
- `gz_run.sh`, `sitl_check.py`: headless gz SITL gate (`PX4_ROOT` defaults to ~/dev/PX4-Autopilot). `GZ_TARGET`
  picks the make target, so the same checks run against the plain and the hull-aerodynamics model. gz_run.sh traps
  INT/TERM/EXIT, because a killed run used to leave px4 and gz sim holding the ports for the next one. The gate
  returns 3, not 0, when the yaw loop is absent: nothing below that point was measured, which is not a pass.
  Three things it got wrong, all fixed 2026-09-17 and all of the same kind, a check that could not fail:
  it deleted `rootfs/parameters*.bson` where px4 keeps them in `rootfs/fs/`, and verified with the same wrong
  path, so every run had been inheriting the one before it; it reported that a world had been *started* rather
  than that it had come *up*, so a gz that died on launch read as a healthy run whose checks then found an empty
  world; and it gave gz-transport one second to release its discovery socket after a SIGKILL, which is not
  always enough for the next server to bind.
- The two plants disagree on one gate check, in both of the modes it runs in, and the disagreement is the point. On the plain model the yaw
  integral sits near zero, because there is nothing to hold against; releasing the stick stops the hull (4.7 s in
  Acro, 5.1 s in Stabilized on the revision-3 hull, chain7_20260925). On the hull-aerodynamics model the integral
  has to hold the plant's own damping torque (nRabsR 241.56 since revision 3: 0.057 normalized at the 6.45 deg/s the
  loop holds; 0.046 on the 2026-09-17 plant's 193.94), so the rate is still coming down when the 12 s window closes:
  6.5 -> 1.9 deg/s at -0.15 deg/s^2 in Acro, 6.5 -> 1.8 at -0.14 in Stabilized (chain5_20260925; the `_drag` gate
  has not run since). Quadratic damping is weakest exactly where the check looks.
  The check still fails, deliberately, but it now says "계속 내려간다" with the slope rather than "멈추지 않음",
  because a rate that is decaying slowly and one that is stuck are different findings. Expected at d2647379af
  (AS_YAWRATE_P 0.7): 10/10 plain, the Acro 제동 check included (chain12 and chain12b, 2026-09-28); the other
  plants' tallies, and the two runs the harness lost, are in yaw_retune_20260927.md. The 8/10 aero (both
  stick-release checks) is chain5_20260925 at P 0.5 and has not been re-run.
- `sitl_check.py` now knows which vehicle it is flying. A gust only means something as a fraction
  of the authority that has to hold it, so the wind torques scale with `VEHICLES[model]`: the 1 N m
  that is 1.9% of the vectored airship's 54 N m would be 22% of the Cloudship's 4.48, and the
  Cloudship gets 0.0830 instead. The "below the tilt band" case is skipped outright there, because
  it is about how the vectored allocation discounts a demand no pod can turn far enough to meet,
  which a collective tilt with a tail thruster does not do. The vectored airship's own torques and
  check names are unchanged, so its baselines still compare. Since 2026-09-27 the
  script refuses to start when the flown model's table entry differs by more than 0.01 N m from
  the model's own full thrust times arm (`gz_model.yaw_authority`: 54.0 and 4.478 today).
- `tilt_range_check.py`: drives each tilt servo to both ends with `actuator_test` and reads the pod's
  own orientation back out of gz, so the claim that the model's +-pi joint stop clips nothing
  reachable is measured rather than argued -- the two numbers behind it, the stop and the bridge's
  +-180 cap, live in different repositories. Polled against the target rather than slept on, and
  averaged as a direction, because the angle it cares about most is +-180, where samples land on
  both sides of the cut and where a pod that has not started moving looks exactly like one that has
  arrived. An earlier version polled against the previous reading and reported the untouched zero of
  the run's first command as a servo that could not reach its angle. 2026-09-17: 8/8, both pods at
  +-180.0 deg on every command, error 0.00.
- `aero_moment_check.py`, `aero_moment/`: the hull's drag moments about the link origin. gz's Hydrodynamics plugin
  both applies its wrench and reads its velocity at the link origin, which on this hull is 2.05 m off the axis the
  drag acts on, so the set needs six cross terms and not the obvious two. The `--analytic` half reads the geometry
  out of the model files, asserts the hull is neutrally buoyant against its declared mass, walks the
  kinematic graph across the whole merge-included family, and needs no gz;
  the rest pushes the hull through the air and checks the settled attitude and that nothing is still swinging. `aero_moment/README.md` has the derivation and the 2026-09-17 numbers,
  including what a partial fix does instead (roll damping changes sign at 0.4 m/s of sideslip) and the trap in the
  derivation: match the moment about the centre of mass rather than the wrench and the attitude comes out right
  while the translational force stays wrong, which nothing that settles can see.
- `sitl_link.py`: the one way in to a running gz SITL airship, for the same reason `fmu_link.py` is
  the one way in to the bench FMU. `MODEL` follows `GZ_TARGET`/`PX4_SIM_MODEL` with the `_0` spawn
  suffix, so a check run against another airframe addresses that airframe and not a stale name; the
  `gz_` prefix is stripped from whichever of the two is set, since px4 itself is started with the
  prefixed form. `param_show` had been reading every parameter the caller had just written as
  absent, which is the one answer its own docstring says must not be guessed: `param show` prints
  three flag columns with single spaces between them and the pattern allowed only a run of flag
  characters. That is why the gate announced "THR_MDL_FAC 가 0 이다" on every run of an airframe
  that sets it to 1. Fixed 2026-09-18 and confirmed gone; `python3 sitl_link.py` now checks the
  pattern against all four shapes `param show` prints.
- `reverse_thrust_check.py`: whether the gz bridge can drive a thruster backwards at all. A
  reversible motor's command stays in -1..1 and PX4 maps it onto [`SIM_GZ_EC_MIN`, `MAX`], whose
  metadata bounds both at 0, so on paper full reverse is a stopped prop and neutral is half
  throttle. The metadata is advisory -- firmware stores a name, a type and a value and no bounds --
  and gz's motor model signs its thrust off the rotor velocity, so this sets a negative minimum and
  reads the rotor speed back off the topic the bridge publishes on. 2026-09-18: the minimum sticks,
  -300.0 read back from -300. `actuator_outputs` is the wrong place to read it, since it has one
  instance per output group and `px4-listener` gives instance 0, which is not the ESCs here. The
  Cloudship's tail is its only yaw authority and depends on this.
- `cloudship_geom_check.py`: rebuilds the gz Cloudship's body from the gazebo-classic template's
  measurements and checks every fact against the SDF -- mass, centre of mass, the whole inertia
  tensor including ixz, the collision ellipsoid's scale, neutral buoyancy, every fin and prop
  position, both thrust constants, and the kinematic graph: a joint naming a link that is not
  there, two joints claiming the same child, a link with no path to base_link, or a plugin driving
  something that does not exist. Needs no gz. The inertia is recomputed rather than copied,
  because the template's own parallel-axis terms take the wrong offsets.
- `cloudship_first_light.py`: the questions a first run of the gz Cloudship has to answer before any
  yaw-loop check means anything — the model is in the world, it floats, the allocator claimed four
  outputs, the tail can reverse, and a yaw stick turns the hull the way it was asked to. Since
  2026-09-23 it runs on every physics variant: on a plant with no drag block it turns the drift into a
  force through the acceleration (mass plus added heave mass) rather than a terminal velocity, reads
  the tail as an angular acceleration on the simulator's clock rather than a terminal rate, brakes
  each leg instead of waiting for a settling that never comes, and asserts the height first, since a
  hull on the ground drifts at exactly zero. And it measures the tail FROM REST, before its own duty
  test pushes the hull: on the added-mass plants a translating hull carries the Munk moment, the size
  of the tail's torque, and without drag that translation never stops.
- `cloudship_land_disarm_check.py`: both sides of the mid-air disarm bug. AirshipLandDetector reports
  landed the moment Land is entered (AirshipLandDetector.cpp:51-60), so with COM_DISARM_LAND at its
  default the commander disarms an airship in the air. rc.airship_defaults sets it to -1. The check
  arms the Cloudship 20 m up, enters Land, and holds 20 s of PX4 time armed while the detector says
  landed; then sets COM_DISARM_LAND 3 at runtime, repeats, and requires the disarm at 3.0 s for
  reason 6 (landing) still at 20 m, so a detector fix would show up as that side failing. The
  parameter is restored in a `finally`. It stops at the first landed message rather than reading
  the latest one after a sleep: the detector republishes every second, and the first run of this
  check reported that age as a 1.27 s detection latency.
- Physics variants (2026-09-23). Every gz airship is a family named `<vehicle>[_am][_fin][_drag]`, tokens in that
  order: the bare name is the rigid hull with buoyancy only, `_am` the hull's fluid added mass, `_fin` fin lift, `_drag`
  hull drag and rotational damping. All but `_am` are thin merge includes of the model with one token fewer; `_am` is a
  full copy, since `<fluid_added_mass>` lives in the link's `<inertial>` and a merge include can only add elements
  (measured 2026-09-17: the plugin's own added-mass form diverges, the include-params route errors). The airframe's ones
  digit is the physics mask, 1 drag + 2 fin + 4 am, 9 the gazebo-classic twin, and the decades run in the order of the
  flight numbers (2026-09-24): 2500-2505 the generic airship's fixed twin `airship_fixed` (2500 is also its flight
  number; no fin bit, its fins are its flaps), 2510-2517 the Cloudship with the classic at 2519, 2520-2527 the
  independently vectored airship (2520 shared with its flight airframe). Earlier generations: 2504/2505 (to 2026-09-17),
  2510/2511 + 2520-2527 + 2508 (2026-09-17 to 24), `_aero` for `_drag`. The rigid `cloudship` is the allocator's regression
  plant and `cloudship_am_fin_drag` the one to tune the yaw loop on; `cloudship_drag` is exactly the model every
  Cloudship baseline before this date was measured on. `gz_model.family()`/`merged()` give the checks the whole chain.
- `variant_check.py`: seven structural facts that make the scheme more than a convention, no gz needed: the include
  chain, `_am` equal to the base but for its one block, the added-mass numbers recomputed from the collision ellipsoid
  (Lamb), the fin plugins identical across `_fin`/`_am_fin` and at each fin's quarter chord, the drag block identical
  across `_drag`/`_am_fin_drag`, every spawn name in the lta world's buoyancy list, and every model's airframe at
  decade + mask, in the CMake list and in the docs table; then absence, which is what "the bare name is
  rigid" actually claims: each model's merged family carries exactly the fluid physics its tokens name and
  nothing else, and a directory or airframe whose name starts with a vehicle's but is not the scheme's is
  reported rather than skipped. Then (2026-09-23 evening) the family's own rules: a `_base` is a
  plugin-free body wrapped by at least one model and never an airframe; a member's posix `@name` is
  its docs heading verbatim and every airship link in the docs resolves to a heading (the docs build
  ignores dead links); no two gz airframes produce one make target, enumerated the way CMake does;
  25xx numbers are unique per directory and inside the decade table, which the check reads from the
  comment above the 25xx entries of the posix CMakeLists, the one place it is written. Thirteen facts,
  mutation-tested 20/20 across the two passes. Then (2026-09-25) three more: 14, the design requirement
  itself, each statically stable body's two vertical fins restoring at least 1.2 x (the design's target)
  its `_am` block's Munk moment on Helmbold's slope, both read from the files (1.203 today); 15, each flap
  servo's SIM_GZ_SV_MINA/MAXA equal to its flap joint's limits; 16, each gz twin's CA_* block equal to its
  flight airframe's, line for line. Sixteen facts. On 2026-09-27 fact 15 also requires every flap joint a fin
  names to be driven by exactly one servo topic, fact 4 checks the Cloudship's stall on the low-aspect-ratio
  plate's CLmax 1.25, and fact 3's Lamb block and fact 14's target come from `hull_basis.py` and
  `generic_hull_design.py` (fact 4 already used `fin_basis.py`), so those facts hold the files to the design
  rather than re-deriving it. 23 mutated trees each failed the new checker; five of them had passed the one
  before (four servo-coverage holes in fact 15, and a wrong but self-consistent Cloudship stall).
- `airship_family_naming.md`: the naming and numbering design for the whole family, with the PX4
  convention each rule comes from (written vs merely practised) and what a 33-agent review refuted
  in the first draft: renaming the vectored airship (the name is the firmware's own phrase and is
  already published), moving CA_AIRFRAME/COM_PREARM_MODE into rc.airship_defaults (it would change
  upstream's gazebo-classic Cloudship silently), and the x500 era's "Gazebo ..." names.
- `added_mass_check.py`: the file against the engine, in probe worlds without px4. A 1 N m yaw torque for 2 s
  gives the yaw inertia from the yaw angle's curvature (rigid 5.69 vs 5.80 from the SDF; with added mass +4.09 vs the
  block's rr 3.95). A push through the centre of mass gives the Munk moment as the slope of yaw rate against the
  integral of u v: rigid -0.14, added mass -4.77 against -(m_y - m_x) = -4.92, the nose turning AWAY from the velocity,
  which is what destabilising means (the first run expected the opposite sign and called a 3% match a failure). Fins on
  the rigid hull +0.56 against the isolated-wing estimate 0.64; on the added-mass hull they remove 1.07, between the
  static term and static plus the rate damping the yaw rate they see implies (one push cannot separate the two, so the
  static term is taken from the rigid-hull run). A roll release then measures the free roll period,
  the one experiment that touches the block's offset terms: rr and m_y - m_x are the same about any
  point on the yaw axis, so only roll can tell a block taken about the link origin (the spec, 1.88 s)
  from one taken about the centre of mass (2.36 s), with the rigid hull's 1.63 s calibrating the
  buoyancy stiffness. The push's couple is +d x F in the world frame (a first version applied -(d x
  F) and doubled the offset moment; the yaw verdicts hid it because a pitch couple against 35 N m/rad
  of buoyancy stiffness barely reaches yaw).
- `cloudship_sitl.md`: the Cloudship's two airframes, why this branch split them, and what a gz port
  would need. The SITL one is upstream's untouched and runs on gazebo-classic, which is not
  installed here, so it cannot be verified as it stands. The classic model does carry real geometry
  for a port. Two open questions: the tilt range nobody has measured, and whether the gz bridge can
  drive a reversible thruster at all.
- `added_mass_and_fins.md`: why the last two simulator gaps are still open. They are one change, not
  two, because `<fluid_added_mass>` brings the Munk moment with it and the reference design measures
  that at 72 N m of yaw against 19 N m of damping. Records which structural routes are closed (the
  sdformat include-parameter one, by test) and what data the fins are waiting on.
- `identity/`: bit-identity harness for ActuatorEffectivenessAirship. To use: copy the .cpp into
  src/modules/control_allocator/VehicleActuatorEffectiveness/, append
  `px4_add_functional_gtest(SRC ActuatorEffectivenessAirshipIdentityTest.cpp LINKLIBS VehicleActuatorEffectiveness)`
  to that directory's CMakeLists.txt (never commit either), `make tests TESTFILTER=AirshipIdentity`, then
  `IDENTITY_OUT=/tmp/new.txt build/px4_sitl_test/functional-ActuatorEffectivenessAirshipIdentity` and
  `identity/idcmp.sh /tmp/new.txt identity/dump_3b0a61e319_12sets.txt` (signed zero ignored: PX4 builds with -fno-signed-zeros).
  `IDENTITY_SETS=independent` runs only the four independent-pod sets (2504, asym_wide, asym_one_rear, asym_vertical), the ones
  the upstream-bound class can express; a parameter the tree does not define is left alone, so the same source runs on both trees.

  Parameter sets (2026-09-16, 12 sets x 7,399 rows = 88,788 lines): the seven original ones plus
  `asym_wide` (-100..99, independent), `asym_wide_coll` (-100..99, collective + tail + 2 surfaces)
  `asym_one_rear` (-100..60) and `asym_vertical`/`asym_vertical_coll` (-88..92, ends 180 deg apart: only the rear-cone margin hold of 117046d8a7 keeps a straight-back demand there). The three steerTarget defects of 2026-09-11 only showed on wide
  asymmetric ranges; with these sets every steerTarget branch runs under the script (rear-cone
  ranking with moves, out-of-range hold and end switch, seam check and seam hold), see
  `identity/pod_tools.sh coverage`.
- `bench_fingerprint.py` fails the run when idle and full thrust do not separate by 50 us on any pin: a flat table
  means the sticks never reached the FMU, and two flat tables diff as "identical", which is the verdict this tool
  exists to produce. Since 2026-09-27 it also fails when any vector ran outside Manual (the FC's mode from its
  heartbeat, stored per vector as `modes` in fingerprint.json). It warns when COM_RC_IN_MODE leaves RC ahead of
  MAVLink, since a powered receiver outranks the sticks the script sends.
- `identity/pod_tools.sh replay|coverage <dir>`: pod-only replay of the harness script, no PX4 build
  (AirshipPod is pure math; its outputs never feed the next step's demand, so a bit-identical tilt
  trajectory is equivalent to full-harness identity for a change confined to AirshipPod). Pass
  `-mfma -ffp-contract=fast` as extra flags to reproduce the Cortex-M7 contraction on x86. The coverage anchors are
  literal source strings: when they stop matching, the tool EXITS instead of printing a table of zeros, because a
  zero column that means "the tool drifted" is indistinguishable from one that means "this branch never ran". They
  track AirshipPod.cpp at e582116632; re-check them after any edit to steerTarget.

## preset2500/ — 2500_generic_airship migration verified at the pins (2026-09-09, fmu-v6x, airship/dev 37cbde224c)
Same firmware, three configurations, `bench_fingerprint.py` (13 stick vectors x 8 pins, props off):
- runA_fp: the branch's 2500 values applied live (CA_AIRFRAME 16, GRP 0, TLMIN/TLMAX 0) — allocator "Configured actuators: 6"
- runB_fp: the upstream 2500 values on the same firmware (CA_AIRFRAME 9 + CA_ROTOR matrix, Source: Custom)
- runC_fp: booted with SYS_AUTOSTART 2500 so the airframe script set the values (runC_boot.log)
- A vs B vs C: max |diff| 0.0 us. restore_fp: the 2504 bench config re-measured afterwards, 0.0 us vs the same-day reference.
- snapshot_before/after.json: every parameter before and after (only CAL_BARO0_OFF, COM_FLIGHT_UUID, LND_FLIGHT_T_LO moved).
Helpers: `apply_params.py NAME=VALUE ...` (PARAM_SET one at a time with read-back), `snapshot_params.py out.json`,
`fmu_shell.py 'cmd' ...` (MAVLink shell), `reboot_fmu.py` (MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN). All use the
/dev/serial/by-id path: after a reboot the kernel may hand the FMU ttyACM1.
Gotchas learned: (1) a MAVLink-shell `param reset` of ten allocator params at once (CA_AIRFRAME 9->16 with
CA_ROTOR_COUNT 2->0) froze the FMU — USB stayed enumerated, no heartbeat, no hardfault log; only a power cycle
recovered it. Use PARAM_SET one at a time. (2) Switching to an airframe whose `param set-default` equals a SAVED
value erases that saved entry (param_set_default_value -> user_config.refresh); switching back then leaves the
firmware default. Here the 2500 script's CA_SV_CS* defaults wiped the saved 2504 fin block (allocator dropped to
4 actuators) until re-applied. Snapshot before, diff after, always.
