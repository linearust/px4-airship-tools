# Upstream PR bodies (draft, 2026-10-13)

Draft bodies for the PR set in `upstream_pr_plan_20261007.md`, written to be pasted as they are.
Verified facts only; numbers come from the gates in `docs/` and the gtests. Fill the bracketed
items when their measurements exist (the hardware flights; the @maintainer decision). Re-run
`arm-none-eabi-size` on micoair_h743-aio_default at the PR head before pasting the footprint
table: parameters.json.xz is in the ROMFS, so a module.yaml change moves the text size.

## M1 — PX4-gazebo-models: lta world, airship_base, the vectored and fixed generic families, Cloudship

Adds a neutral-buoyancy world (`lta`) and three airship model families for gz SITL: a 7 m
generic hull with independently tilting pods (`airship_vectored_independent`), the same hull
with fixed thrusters and four fin flaps (`airship_fixed`), and the Cloudship (4.7 m, collective
tilt, tail thruster). Each is a complete plant: buoyancy, Lamb added mass for the hull's
fineness, Hydrodynamics drag cross terms about the link origin, fin lift-drag plugins.

- Added mass and aerodynamic moments were measured in the engine against the file values
  (added mass 9/9, moments 14/14, fin damping within 0.07 %); the Cloudship's added mass is
  Lamb's for fineness 2.94 (the template factors were fineness-6).
- The Hydrodynamics plugin reads velocity and applies its wrench at the link origin, so an
  off-axis origin needs the six cross terms; the two obvious ones alone destabilize the hull.
- The magnetometer noise is in tesla and the world loads the Magnetometer system with
  `use_units_gauss=false` and `use_earth_frame_ned=false`, as `gz_bridge`'s server.config does
  (a world that spells out its own systems otherwise gets the legacy gauss output and a wrong
  heading).
- The Buoyancy plugin needs every buoyant link enabled by name; the world lists them.

## M2 — PX4-gazebo-models: the indoor airship family

[After the 2 m vehicle flies: the family generated from its measured dimensions, with the
held-turn slope measured on the vehicle.]

## P1 — control_allocator: airship effectiveness (CA_AIRFRAME 16)

Replaces the single-configuration airship effectiveness with one that covers independently or
collectively tilting pods, an optional tail yaw thruster and credited control surfaces
(`ActuatorEffectivenessAirship`, `AirshipPod`, `AirshipYawAllocation`), and publishes what the
yaw rate controller needs to know about the configuration in `airship_allocator_status`.

Reviewer questions, answered:

- **Why a new message (`airship_allocator_status`)?** The attitude controller must not read
  another module's parameters (parameters guideline). Whether the propulsion gets yaw torque to
  make, and how much at rest, is the allocator's result; the controller takes it on uORB.
- **Why does the controller not read `control_allocator_status` for this?** That topic
  reports allocation residuals per cycle; the propulsive-yaw verdict and the at-rest authority
  per direction are configuration facts that change on a parameter update, and the controller
  needs them before its first cycle (the allocator reports at construction).
- **The pods are non-reversible; why a steer band?** A pod whose thrust cannot reverse steers
  through the tilt range to point its force; the band keeps a demand the pod cannot realize
  from flipping it back and forth (the dither a 200k-sample property test pins).
- **2500 and 2507 move to CA_AIRFRAME 16.** 2500 (fixed thrusters, four flaps) and 2507
  (Cloudship) were verified bit-identical at the FMU pins through the migration (0.0 us
  fingerprint regression on fmu-v6x) and on the gz twins; no flight on the Cloudship itself.
  If the reviewers prefer, 2507 stays on 9 and moves in P5.
- **`@maintainer` on 2507.** [Keep only if the author takes the Cloudship on; otherwise drop.]
- **Flash and RAM.** micoair_h743-aio_default, `arm-none-eabi-size` of the elf, upstream main
  (a4697ed08e) against this branch (48b45f3e5f, the whole airship set; 1d2aef0e5c carries the same firmware):

  | | text | data | bss |
  |---|---|---|---|
  | upstream main | 1865120 | 6540 | 59864 |
  | this branch | 1876512 | 6540 | 59864 |
  | delta | +11392 (+0.6 %) | 0 | 0 |

Validation: unit tests (`AirshipPod`, `AirshipYawAllocation`, functional
`ActuatorEffectivenessAirship`), the allocator identity harness (12 sets, IDENTICAL), fmu-v6x
bench fingerprints at the pins (logic analyzer), gz twins.

## P2 — airship_att_control: the yaw rate loop

The yaw stick commands a yaw rate closed by a PI loop with setpoint feedforward wherever the
propulsion gets yaw to make; roll, pitch and thrust stay stick passthrough. The feedforward is
the hull's damping torque at `AS_YAWRATE_MAX`, scaled by the setpoint's share of the maximum
(the damping is quadratic in the rate), so the torque that holds a turn comes from the setpoint
and goes with it on release instead of being built up in the integrator. The integrator is
saved when the stick leaves the deadzone and put back when it returns or reverses.

- **Why no enable parameter (as FW_ACRO_YAW_EN)?** The loop follows the allocator's report
  alone: on a frame whose yaw comes from control surfaces the stick stays torque, because the
  loop does not follow how a surface's effectiveness changes with airspeed; at rest its
  integrator would wind to the limit with no shortfall reported. A parameter that forced it on
  would reintroduce exactly that.
- **Why quadratic feedforward?** A constant gain that held one rate held smaller ones with
  r_max / r times the torque before; the loop overshot them (model and SITL, 48/48 gate
  verdicts).
- **Why restore the integrator at every stick input?** Without it the integrator parked at its
  limit during a turn and released over 37-145 deg of heading (SITL step 0); with it ≤ 6.5 deg.
  The price is a yaw trim that changed during the turn coming back stale, which the loop
  relearns at `AS_YAWRATE_I`.
- **`rate_control::setIntegral`/`getIntegral`.** The restore needs to set one axis's integral;
  the getter mirrors it. Both are unit tested in the library.
- **Modes.** Acro, Stabilized, Altitude and Position close the loop; the airship has no attitude
  or position controller, so the last two are the manual passthrough with the yaw loop.
- **Land detector.** `AirshipLandDetector` reports landed as soon as Land mode is selected,
  which with `COM_DISARM_LAND` > 0 disarms the airship in the air; `rc.airship_defaults` sets
  -1. Reproduced in SITL (`cloudship_land_disarm_check.py`, 10/10 both ways). A sensor-based
  landed verdict for an airship is a separate feature.

Validation: the module's five gtest binaries, 65 cases (yaw rate loop 13, schedule 13, speed selector 11, manual input 14, functional 14), SITL gates on
the generic and Cloudship plants (hover 10/10 and 9/9), [2 m vehicle flights].

## P3 — airship_att_control: the forward-speed schedule

Fins damp yaw in proportion to the forward speed. The schedule adds feedforward per m/s
(`AS_YR_FF_SLP`) on the lower bound of a forward speed interval and caps the yaw rate where the
propulsion can still hold the turn with integrator in reserve. The interval comes from an
airspeed sensor widened by its pressure tolerance, the estimator's body-x ground speed widened
by a wind bound, or their intersection (`AS_SPD_SRC`), filtered at 1 s.

- **Why a second message (`airship_speed`)?** The interval, its sources, the cap and the gain
  applied have no place in an existing topic, and the forward-flight analysis judges the loop
  by them. It is an optional logged topic (`add_optional_topic`), absent from other vehicles'
  logs.
- **`AS_SPD_DP_TOL` has no `unit:`.** Pascal is not in the parameter metadata unit list; the
  short description carries [Pa].
- **Why an interval and not a value?** An uncertain speed gets less feedforward and a lower
  cap; a wrong airspeed reading the ground interval rules out is dropped for that cycle.
- **Ground speed without `heading_good_for_control`.** Velocity and heading are one frame, so
  a heading error cancels for a body-frame source (optical flow) and costs u (1 - cos delta)
  for GNSS, 1.5 % at 10 deg; requiring the flag would keep the source off indoors.

Validation: gtests, forward-flight SITL on the fin plants (42/42 on the generic complete
plant), [2 m vehicle with fixed fins].

## P4 — gz SITL airframes, docs, submodule pointer

The posix airframes of the three families' complete plants, the vehicle pages and the `lta`
world description; the submodule pointer to M1 (and M2).

## P5 — the rest

- `2519_gazebo-classic_cloudship` (or 2507 if the rename is dropped): the `@output` header named
  MAIN1 the tilt while `PWM_MAIN_FUNC` drives Motor 1 on it; fixed to the mapping. The classic
  model places motor 0 at Gazebo +y, so the classic sim's differential-thrust yaw sign is
  unverified (a model item).
- `module.yaml`: "Pod Tilt Servo count 1 + Port Pod Tilt Servo count CA_AIRSHIP_GRP" shows one
  unused servo in the GCS when the range is 0; a limit of the metadata, noted in the yaml.
- `CA_AIRSHIP_GRP` is read on a parameter update with no armed check: switching grouping while
  armed can leave the pods apart for a moment; change it disarmed (documented).
