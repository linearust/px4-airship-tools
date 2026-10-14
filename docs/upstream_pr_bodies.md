# Upstream PR bodies (2026-10-13, the two-PR shape of upstream_plan_20261013.md section 10)

Written to be pasted as they are. Verified facts only; bracketed items wait for their measurement
(the indoor flights, the footprint at the PR head, the merged models commit). The PX4 PR opens
after the models PR has merged and its pointer has been moved to the merged commit. Neither PR
opens before the indoor tests are done and the user has reviewed the branches.

## PX4-gazebo-models — feat: add the generic airship with independently tilting pods and the lta world

Adds a lighter-than-air vehicle for gz SITL: `airship_base`, the body (a 7 m hull with its
gondola, tail and sensors, and the hull's fluid added mass on `base_link`), and
`airship_vectored_independent`, the complete plant that merge-includes it and adds two side pods,
each a propeller on a tilt joint, the four fins as lifting surfaces, and the hull's drag and
rotational damping. `worlds/lta.sdf` is the default world plus the Buoyancy system at the air
density (1.225 kg/m³), enabled for this model and its instances 0 to 3, with the 100 km ground
box (#162) and the magnetometer in tesla (#161, `use_units_gauss=false`,
`use_earth_frame_ned=false`, as PX4's `server.config` sets them).

- 5 files, +887 lines; primitive geometry only, no meshes; `model.config` with the author.
- The PX4 twin is airframe 2520 (`make px4_sitl gz_airship_vectored_independent`), submitted to
  PX4-Autopilot with its allocator and yaw rate loop; the two PRs are linked.
- The physics values (added mass by Lamb for the hull's fineness, the fins' lift slope, the hull's
  drag and the Hydrodynamics cross terms about the link origin) were measured in the engine against
  the file values on the members this plant is generated from [added mass 9/9, moments 14/14, fin
  damping within 0.07 %]; the plant is element-wise equal to those members (generator twin check).
- Why a world: the Buoyancy system is world-level and needs every buoyant link enabled by name.
- gz prints `Inertia::verifySpatialTensor` warnings for `airship_base` at load: the fluid added mass is
  written about the link origin, not the centre of mass, so its cross terms do not satisfy the
  rigid-body relation the check expects; the engine measurements above were taken with exactly
  this matrix. The simulation runs with it.
- A screenshot for `docs/assets/simulation/gazebo/vehicles/airship_vectored_independent.png` is
  still to be taken [from a desktop gz session: `gz sim` on the lta world with the model spawned,
  the GUI's screenshot button].
- The hull is a 7.0 m × 2.0 m prolate spheroid (L/D 3.5, 14.66 m³, 18.0 kg neutrally buoyant),
  the 7 m RC blimp class, with this project's two-pod actuation and the assumed pod thrust and
  masses the vehicle file lists; the vehicle that validated the firmware is a 2 m member of the
  same family and is not part of this PR.

## PX4-Autopilot — feat: airship support for two independently tilting pods

### Summary

Airships with two side pods that tilt independently get a control-allocation effectiveness
(`CA_AIRFRAME` 16, Airship), a yaw rate loop in `airship_att_control`, a flight airframe (2520)
and its gz SITL twin, with docs. Upstream's 2500 (Generic Airship) and 2507 (Cloudship) stay on
the Custom allocator; the loop is off by default and closes only where an airframe sets
`AS_YAWRATE_MAX` above zero.

### What it adds (commit order; a split along it is a cherry-pick; numbers at head d020cc4c40)

| commit | files | lines |
|---|---|---|
| rate_control: set and read the integral term of one axis | 2 | +188 (test 167) |
| control_allocator: the airship effectiveness for two tilting pods | 10 | +2375 |
| airship_att_control: a yaw rate loop on the stick | 9 | +1441 −40 |
| airframes: 2520 Generic Airship (Independent Vectoring) | 2 | +44 |
| simulation: the gz twin, posix 2520 | 2 | +77 |
| simulation: gz models pointer | 1 | [merged commit] |
| docs: frames_airship, sim_gazebo_gz vehicles/index/worlds, releases | 5 | +167 −20 |

31 files, +4293 −61, of which 2665 lines are tests. No new message. Parameters: 8
(`CA_AIRSHIP_TLMIN`, `CA_AIRSHIP_TLMAX`, `CA_AIRSHIP_TLT_R`, `AS_YAWRATE_P`, `AS_YAWRATE_I`,
`AS_YR_INT_LIM`, `AS_YAWRATE_FF`, `AS_YAWRATE_MAX`). The controller also reads `MAN_DEADZONE`.
The models PR is 5 files, +887 (commit d3beb42 on the models' main e5997f4).

### Allocation

A pod's thrust is projected onto the tilt angle its servo realizes, so the effectiveness is not a
matrix: the class declares zero columns and computes the actuators in `updateSetpoint()`, as the
helicopter class does, and reports a sign per axis through `control_allocator_status`. The pods'
differential thrust is yaw and roll torque, the tilt projects the common thrust between forward and
vertical; pitch torque and lateral thrust are reported unallocated. The propellers are
non-reversible: the motor commands stay non-negative and thrust reverses by tilting through the
range, with a steering band, end-stop choice and slew inside the model (`CA_AIRSHIP_TLT_R`;
`CA_SVn_SLEW` runs after the model and would desynchronise the projection). Why not Custom: a
linear matrix cannot point the thrust. Why not Tiltrotor VTOL: its tilt is scheduled by the VTOL
transition logic, not solved from the wrench, and spans ±90°.

Units: 1.0 on a torque axis is what the pods deliver at rest at full differential; 1.0 on thrust x
is both pods full forward. Outputs: Motor 1 starboard pod, Motor 2 port pod, Servo 1 and Servo 2
their tilts, always declared; a later layout only appends behind them.

### Yaw rate loop

In the manual modes with rate control (Acro, Stabilized, Altitude, Position) the yaw stick commands
a yaw rate that a PI loop with setpoint feedforward closes on the measured yaw rate. The loop
closes only where `AS_YAWRATE_MAX` is above zero (default 0; 2520 sets 15 deg/s), the opt-in shape
of `FW_ACRO_YAW_EN`. The integrator is saved when the stick leaves the deadzone and put back when
the stick returns or reverses (what a turn gathered against the hull's damping ends with the turn),
stops in a direction the allocator reports it cannot serve, and is cleared when the loop opens.
`AS_YAWRATE_FF` holds a turn at the maximum rate against the hull's quadratic yaw damping, scaled by
the setpoint's share of the maximum.

The manual passthrough changes for every airship, loop or not: the roll stick is roll torque and
the pitch stick also vertical thrust (both unallocated on Custom airframes without such actuators),
the wrench is zero without usable sticks instead of the last sticks, and no wrench is published in
a mode that enables none of manual, rate and attitude control (Offboard with thrust and torque
setpoints owns it). These are stated in the controller commit.

2520 sets `COM_DISARM_LAND -1`: the airship land detector reports landed as soon as Land is
selected, which would disarm the vehicle in the air; a land-detector fix is a separate candidate.

### Tests and checks

- gtest: AirshipPod 25 cases (incl. a 200,000-sample property test of the steering band),
  ActuatorEffectivenessAirship 37, AirshipYawRateLoop 15, AirshipManualInput 13,
  AirshipAttitudeControl 10 (the module through uORB), RateControl +6 (one for the accessors,
  five pinning the behaviour the loop builds on).
- `make tests` (the five airship binaries and rate_control), `check_format`,
  `validate_module_configs`, `module_documentation`, `px4_sitl_default`: clean at d020cc4c40.
  fmu-v6x_airship builds; [micoair_h743-aio exit code at the final head].
- Footprint, `arm-none-eabi-size` fmu-v6x_airship at d020cc4c40: text 1,732,128, data 7,732,
  bss 106,812 [before/after table at the final head, both boards].

### Evidence

- Bit identity: the narrowed class against the author's wider development class on the same
  independent-pod parameter sets: 4 sets, 29,596 rows, IDENTICAL (against airship/dev 27fd5f1ffd,
  unchanged through the trim; re-run at d020cc4c40).
- Bench, a px4_fmu-v6x board, logic analyzer at the output pins, 13 stick vectors in Manual × 8 pins, props off:
  fingerprint 0.0 µs between this branch (d020cc4c40) and the author's development branch (8af72b9a50) at the
  2520 preset, 0 of 13 vectors differ; 15 tilt slews per run at the `CA_AIRSHIP_TLT_R` rate (180° in 1.5–1.6 s at
  120°/s); on disarm the deflected tilt is back at rest within 0.3 s and the motors drop to the disarmed pulse in
  the same 0.1 s window; tilt end stops 1000/2000/1500 µs on both pod servos (actuator_test, disarmed).
- Flight, 2 m indoor vehicle on a MicoAir H743 AIO [exact board variant confirmed on receipt], two
  ±180° pods, optical flow, no wind [Flight Review links]:
  Manual/Stabilized hover; tilt vectoring fore/aft/up/down; differential-thrust yaw with no tilt
  reversal in the steering band; Acro yaw steps (P/I/INT_LIM); full-stick release in Stabilized
  (restore rule; heading drift after release); anti-windup at full throttle + full yaw;
  AS_YAWRATE_FF measured per the docs procedure then flown; Land selected in the air keeps the
  vehicle armed.
- SITL: hover gate 10/10 on posix 2520 from this branch (475267c93a, and again at d020cc4c40);
  sim-vs-flight table from the vehicle's own twin [table].
- Not flown, unit tests only: a fixed mount (`TLMIN` = `TLMAX`), a ±90° range, the 7 m twin's own
  gains, `AS_YAWRATE_FF`'s shipped default 0.

### Not in this PR

Pod grouping (collective tilt), a tail thruster, control-surface credit, the forward-speed
feedforward schedule, an allocator status message, and the migration of 2500 and 2507 to the new
allocator: each follows with the vehicle that validates it.

## Claim → card (the hardware package)

| PR claim | bench/flight card | what the card records |
|---|---|---|
| allocation = development branch at the pins | bench fingerprint, 2520 preset | fp_diff 0.0 µs, IMPLIED keys noted |
| tilt stops and slew = parameters | tilt range card, `--drive` | pins reach ±180 both pods, slope = TLT_R |
| park on disarm | disarm card | pins at park angle after disarm |
| yaw and roll from differential thrust, vertical thrust from the pitch stick | Manual vectoring flight | actuator_motors/servos vs torque/thrust setpoints |
| no reversal dither in the steering band | hover with small yaw demand | tilt reversals counted at the pins: 0 |
| P/I/INT_LIM track the setpoint | Acro step flights at two amplitudes | rates setpoint vs gyro z, settled integrator |
| restore rule | Stabilized full-stick release and reversal | heading drift after release, yawspeed_integ step |
| anti-windup | full throttle + full yaw | unallocated_torque[2] ≠ 0, integrator flat |
| AS_YAWRATE_FF procedure | 360° turn with FF 0, then with the value | integrator near zero in a held turn |
| COM_DISARM_LAND -1 | Land in the air | vehicle_status stays armed |
| the flown code is the PR code | staged image name | fw_<commit> from the preparation worktree |
