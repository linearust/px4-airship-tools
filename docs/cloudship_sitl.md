# Cloudship in the simulator: what is there, what split, and what would be needed (2026-09-17)

## The two airframes came apart because of us

Upstream ships the Cloudship twice and, until this branch, identically as far as allocation goes:
`init.d/airframes/2507_cloudship` and `init.d-posix/airframes/2507_gazebo-classic_cloudship` both
set `CA_AIRFRAME 9` with the same three-rotor geometry, one control surface and `CA_R_REV 7`. This
branch rewrote the flight one onto the airship effectiveness (`CA_AIRFRAME 16`, `CA_AIRSHIP_GRP 0`
collective, `CA_AIRSHIP_TAIL 1`, tilt +-90, `CA_R_REV 4`) and left the SITL one exactly as upstream
has it. `git diff upstream/main` on the posix file is empty.

So the split is ours, it is one file wide, and a reviewer of any upstream PR will see it.

## It cannot be verified here

The SITL Cloudship runs on gazebo-classic. There is no `gazebo` or `gzserver` on this machine and
no gazebo11 package; only the model files are present, in the sitl_gazebo-classic submodule.
Converting that airframe to match the flight one is a three-line edit and an unverifiable one, and
the vehicle it describes is the one already reported as not behaving. So the choice is between
converting it blind, reverting the flight-side change, or building the Cloudship a simulator that
does run here.

## A gz port has real numbers to work from

`models/cloudship/cloudship.sdf.jinja` is parameterized, and the parameters are measurements rather
than placeholders:

| | |
|---|---|
| hull | 4.7 m long, 1.6 m max diameter, 5.97 m^3, centre of volume 2.22 m from the nose |
| gondola | 0.4 x 0.17 x 0.1, 2 m from the nose, 0.8486 m below the centre of volume |
| payload | 1.05 kg, 2.4 m from the nose, 0.784 m below |
| fins | four, root chord 0.465, span 0.39, thickness 0.01, from 3.635 m, 0.5634 m off the axis |
| masses | hull 1.7, helium 1.045, gondola 3.3, payload 1.05, fins 4 x 0.055 = 7.315 kg |
| lift | 5.97 m^3 x 1.225 = 7.313 kg, so neutral to 2 g |
| thrusters | two, +-0.45 m in y on a tilting rod at the gondola, 13" props, 1.62 kg each, tau 0.05 |
| tail | 5" prop, 0.24 kg, tau 0.025 |
| added mass | k_axial 0.04662, k_transverse 0.9147, k_rotation 0.75567, plus a fin term |

That is a 4.7 m, 7.3 kg vehicle, a seventh of the generic airship this branch already models, and
enough to build a faithful gz model rather than an invented one.

## What was built (2026-09-17)

`Tools/simulation/gz/models/cloudship` and
`ROMFS/px4fmu_common/init.d-posix/airframes/2510_gz_cloudship`. Every length and mass is the classic
template's measurement; the inertia is recomputed, because that template's parallel-axis terms take
the wrong offsets (its gondola `ixx` is built from the x offset and its `izz` from the z one). The
frame follows `airship_base`: x forward, y port, z up, origin on the ground under the gondola.

| | |
|---|---|
| base_link | 7.0148 kg at (0.0155, 0, 0.4183), ixx 2.3196, iyy 6.8533, izz 5.6703, ixz 0.3598 |
| whole vehicle | 7.3148 kg, centre of mass z 0.4055, centre of volume z 0.8976, restoring 35.28 N m/rad |
| buoyancy | hull ellipsoid scaled by 0.981925 to the real 5.97 m^3, plus the gondola's 0.0068: lift 7.31480 kg against 7.3148 |
| thrusters | 7.0632e-06 * 1500^2 = 15.89 N each, opposite turning directions so their couples cancel |
| tail | 1.0464e-06 * 1500^2 = 2.35 N, 1.671 m aft, 3.93 N m of yaw |
| drag | the same reference set and the same six cross terms, scaled: r_ref 0.74284 m, A 1.73354 m^2, S 19.3908 m^2, L 4.7, rotational scale 4.2394 N m, offset h 0.8986 m |

The number was 2508, then 2520 (2026-09-23), and is 2510 since 2026-09-24 (below), because 2507 in `init.d-posix` was upstream's gazebo-classic build of the same
airship, which this leaves alone. The two simulators sit next to each other and only the flight
airframe has one number.

The hull collision is an ellipsoid, which is worth noting because the generic airship's is a box
"sized for volume rather than shape". The Buoyancy plugin measures ellipsoids as happily as boxes,
so that choice on the older model could be revisited; it would change that vehicle's displacement
from 40 to 34.36 m^3 and every gate baseline with it, so it has not been.

## It did not come up under gz_run.sh, and the script was why

Two attempts, both the same. px4 finds the airframe (`SYS_AUTOSTART=2508`), starts gz on lta.sdf,
and one second later the log says `Subprocess killed` followed by thirty `Waiting for Gazebo
world...` and `ERROR [init] Timed out waiting for Gazebo world`. The same queue runs
`gz_airship_vectored_independent` and `..._drag` through the same script on the same world minutes
earlier and later, and both come up.

What is ruled out:

- **the model.** Loaded standalone in its own world it parses with nothing but the `gz_frame_id`
  warnings every PX4 model produces, and when px4 was pointed at an already-running world it
  spawned `cloudship_0` into it and the bridge attached: `world: probe_974555, model: cloudship_0`.
- **the world.** lta.sdf starts fine for the other two airframes after the Cloudship's buoyancy
  lines were added to it.
- **the model name.** `px4-rc.gzsim` takes `${PX4_SIM_MODEL#*gz_}`, so `gz_cloudship` resolves to
  `cloudship` and `cloudship_0`, which is what spawned.

The A/B settled it (2026-09-18). Both airframes, one after the other, nothing else running,
`GZ_VERBOSE=3`, eight seconds of settle between them: **both came up**, each reaching `Gazebo world
is ready`, `world: lta, model: <name>_0` and `Startup script returned successfully`. The Cloudship
is not special. Both earlier failures followed a gz that had died seconds before, and `gz_run.sh`
was tearing down and relaunching inside that window; it now waits six seconds after the cleanup and
waits for the world to be *ready* rather than *started*, and the Cloudship comes up every time.

Two defects in the script were found on the way and are worth keeping separate from this one,
because each of them had been silently wrong for every run:

- it deleted `rootfs/parameters*.bson` where PX4 keeps them in `rootfs/fs/`, and its own
  verification checked the same wrong path, so it always passed and every run inherited the
  previous run's parameters.
- it waited for the log line that says gz was *started*, not the one that says the world is
  *ready*, on a block-buffered log: a good run could warn and a dead one could look healthy.

## It hangs nose down, and that is the vehicle

Measured in gz with nothing applied, the hull settles 2.61 degrees nose down and exactly level in
roll. That is not a modeling error: the gondola's 3.3 kg sits 0.22 m forward of the centre of
volume while the payload and fins
sit aft, which puts the whole vehicle's centre of mass 22 mm ahead of its centre of buoyancy. The
classic template's own centre-of-gravity expression gives the same 24 mm. A 22 mm offset against a
0.49 m righting arm is `atan(0.02236 / 0.4921)` = 2.60 degrees, and the hull hangs there.

Worth knowing for two reasons. It is what any check of this vehicle has to subtract before it can
read a pitching moment at all, and it is a real handling property: the Cloudship cruises nose down
unless something trims it.

## What it does, measured (2026-09-18)

| | |
|---|---|
| SITL gate | **9/9**, every check that applies. It passes "Acro 가 남은 회전을 잡는다", which the 49 kg vectored airship fails on both of its models |
| neutral buoyancy | drifts 0.0223 m/s after settling, which is an imbalance of 0.002 N against 71.7 N of weight |
| tail, full reverse | -53.9 deg/s; full forward +58.0 deg/s, at duties 1100 and 1900 with reversible_flags 4 |
| tilt servo | +90.0, -90.0 and -0.0 deg commanded and reached, inside a +-180 stop |
| thrust vectoring | tilt 0 -> +90 turns the thrust +87.7 deg (0.1 -> 87.8) |
| collective symmetry | roll rate under 0.21 deg/s through every burst, at every tilt |
| hull drag wrench | 3 of 4, the fourth being the deliberately stripped control case below |

The gains were not touched. The airframe's claim that rc.airship_defaults transfers - a seventh of the
size but the same angular acceleration per unit of command, 0.69 rad/s^2 against 0.71 - is what the
gate result says.

### Two ways to measure a tail that does not exist

Both of these passed something before they measured anything, and both are the same mistake in
different clothes: a reading taken where the quantity was not.

The reverse-thrust check commanded the tail and read the hull's yaw rate once, after a ten second
sleep. It reported `+1 -> +50.0 deg/s, -1 -> -0.5 deg/s`, which reads exactly like a tail that pushes
one way only. `tail_probe.py`, sampling the duty, the yaw rate and reversible_flags together once a
second, showed the tail reversing to -53.9 deg/s within two seconds of the command, three times over.
The override lapses on actuator_test's own timeout, which is counted in the simulator's clock while
the sleep is counted in this machine's; a single sample at the end of a sleep cannot tell a tail that
never pushed from one that pushed and was let go, because both read as a hull coasting down.
`tilt_range_check.drive_to` had already learned this and says so in its own comment. The check now
re-issues every pass and takes the extreme over the window.

The vectoring check measured the velocity change over a burst and read 20 degrees shallow at tilt
-90. It was not wrong about the velocity: this hull has 0.0946 N s^2/m^2 of axial drag on 7.3 kg, so
a burst leaves it running for a minute, and the third burst was still carrying the second's speed.
The velocity change was thrust plus the drag on that leftover, which tilts the direction. It now
samples the coasting acceleration first, with the rod already in place and the motors off, and takes
the thrust as the difference - the same baseline subtraction the moment check needed, for the same
reason.

### Measuring a thrust on a hull with 35:1 drag

The vectoring check reads the thrust direction off the hull, and on this vehicle acceleration is not
that direction. Heave drag is 3.2738 N s^2/m^2 against 0.0946 in surge, so a burst still
accelerating horizontally has already gone terminal vertically: pushed straight down with the rod at
-90 it measured -48 degrees, and the magnitude came out 0.739 m/s^2 where the level burst gave
1.088. The thrust is what the hull felt plus what the air took, at the speed it was doing while it
took it, with the coefficients read out of the same model.

It reports a magnitude as well as an angle, and that is the whole reason the rest of this section
exists. Three errors got through the angle check and were caught by the magnitude, in order:

- **body coefficients on NED velocities.** The one that mattered. Sway drag is 35 times surge, so a
  hull running three metres a second on a heading that is not north had 29 N of sway drag credited
  to it: the level burst recovered 16.13 N where the pods deliver 7.95. Rotating through
  `vehicle_attitude.q` first fixed it, and it also made the expected answer *the commanded tilt
  itself* rather than a difference between tilts - the hull's 2.6 degree nose-down hang is not in
  the way once the measurement is in the frame the rod is in.
- **an average taken inside a nonlinear function.** `F * span = m dv + integral of c v|v|`, so the
  drag term is the time average of `c v|v|` and not `c v|v|` at the average speed. v|v| is convex,
  so the two differ - by 27 percent on both vertical bursts and by nothing on the level one, which
  is the signature: vertically the hull reaches terminal inside the window, `m dv` goes to nothing,
  and the whole thrust has to come out of that integral.
- **a coasting baseline subtracted *and* the drag added back**, which counts it twice, and an
  elapsed time taken as the nominal window rather than measured. Both real, neither dominant.

Every one of those produced angles that passed. With all three fixed:

| tilt | thrust, body frame | elevation | commanded | magnitude vs model |
|---|---|---|---|---|
| +90 | (-0.17, +0.10, -8.17) N | +91.2 deg | +90 | +3% |
| 0 | (+8.20, -0.01, -0.23) N | +1.6 deg | 0 | +3% |
| -90 | (-0.61, +0.03, +8.03) N | -94.4 deg | -90 | +1% |

**14 of 14.** The pods deliver 7.946 N at `actuator_test -v 0.25`, by the path rather than by
estimate: THR_MDL_FAC 1 takes 0.25 to sqrt(0.25) = 0.5, the non-reversible remap to 0.5*2-1 = 0,
`interpolate(0, -1, 1, 0, 1500)` to 750 rad/s, and 7.0632e-06 * 750^2 to 3.973 N each. Recovering
that to within 3 percent at three tilts where the drag correction spans a factor of 35 is a
statement about the model and about the method at once.

### The stripped roll case has no equilibrium to compare against

`aero_moment_check.py` runs each push twice, once with the six cross terms and once without, and the
stripped run is supposed to settle on the bare lever: the drag acts at the link origin, so the arm
about the centre of mass is `-com` instead of `hull - com`. The Cloudship's pitch axis does exactly
that, -0.017793 against -0.017599 predicted. Its roll axis does not, and running it out says why:

| settle | roll | predicted | swing |
|---|---|---|---|
| 180 s | +0.027637 | -0.017599 | 0.025230 -> 0.006140 |
| 450 s | -0.000980 | -0.017599 | 0.013337 -> 0.031116 |
| 900 s | -0.006426 | -0.017599 | 0.011793 -> 0.031938 |

The swing **grows**. The same wrench with the cross terms present settles to 0.000146. The cross
terms carry this hull's roll rate damping, which is the thing they were added for, and without them
the axis has no equilibrium for a lever formula to predict. Demanding one asks the plant for
something it does not have.

So the control case now passes either way it can show the cross terms matter: settling on the bare
lever, which is the sharper statement and stays the requirement on pitch, where the stripped swing
is only twice the kept one; or failing to settle where the kept variant settled, which on roll it
misses by a factor of 42 at the default window. Only ever against a kept variant that did settle, on
the same vehicle under the same wrench.

## The yaw thruster was in the wrong place (2026-09-18)

A full review of the SITL code found it. The model's header claims every length is the
gazebo-classic template's measurement, and for one link it was not: the yaw thruster sat on the hull
axis at the fin station, where the template puts it at the trailing edge of the *ventral fin root*.

    thruster(3, "cw", true, "hull", 0.0015, r_fin_x - fin_root_chord/2, 0, -r_fin_rad, ...)

`r_fin_x = hull_cov - (d_fin_x_s + fin_root_chord/2) = 2.22 - (3.635 + 0.2325) = -1.6475` is exactly
where the fins sit in this model, so the frames agree and the thruster simply did not follow: it was
0.2325 m too far forward and 0.5634 m too high, and buried inside the drawn hull, which still has a
0.550 m local radius at that station.

| | lever | yaw | roll | bank |
|---|---|---|---|---|
| on the axis | (-1.670, 0, +0.493) | 3.93 N m | -1.161 N m | 1.89 deg |
| at the fin root | (-1.903, 0, -0.070) | 4.48 N m | +0.166 N m | 0.27 deg |

The height is the part that matters. A yaw thruster goes in the lower fin precisely so that yaw does
not roll the vehicle, and the 1.9 degrees of bank recorded below was an artefact of the placement
rather than a property of the airship. Measured after the move, full command reaches **+-54.1 deg/s**
against the 53.8 the new lever predicts through nRabsR; before it was +-50.6 against 50.4.

The geometry check had the same wrong placement in its own derivation, so the fact passed while both
halves were wrong. Fixing the model made it fail, which is the check working.

## Yaw comes with a little roll (superseded above)

*Kept because the arithmetic is the argument for the placement fix.* With the tail on the hull axis
at z 0.8986 and the centre of mass at 0.4055, the tail's thrust acted 0.4803 m above it. Its lever about the centre of mass is (-1.663, 0, 0.4803) and its
thrust is +Y, which gives `r x F` a yaw of -1.663 * T and a roll of -0.4803 * T. At full command,
T = 2.3544 N, that is 3.92 N m of yaw and 1.13 N m of roll, and against a 35.28 N m/rad restoring
stiffness the vehicle sits 1.84 degrees banked while it turns.

Nothing is wrong with that - it is where the tail is - but it is worth writing down twice over. A
yaw-loop measurement on this vehicle reads a roll too, and a roll that tracks the yaw command is
the tail, not a bug. And it is the opposite bank from a coordinated turn, because the thrust is
above the centre of mass rather than below it.

## Two things to settle first



**The tilt range is not a model constant.** The classic model drives the rod with
`input_scaling = -mot_max_angle = -135 deg` against a joint limit of +-pi, and its PID's
`cmdMax/cmdMin` of +-67.5 is gazebo's effort clamp, not an angle one. The flight airframe assumes
+-90 and says in its own comment that the range and slew rate are assumed and not measured. Nobody
here has the vehicle's number. In a gz model this is not a blocker, because the range would come
from `SIM_GZ_SV_MINA1/MAXA1` in the airframe exactly as it does for the generic airship, so the
answer is a two-line change whenever it arrives. The sign is worth noting either way: the classic
scaling is negative, ours is not.

**The reversible tail: a negative SIM_GZ_EC_MIN does not work, and is not the route.**
`CA_R_REV 4` marks the tail reversible, `FunctionMotors::updateValues` leaves its value in [-1, 1]
rather than remapping [0, 1], and `output_limit_calc_single` interpolates that onto
[`SIM_GZ_EC_MIN`, `SIM_GZ_EC_MAX`]. The gz bridge's `module.yaml` bounds both to 0..1000, so a
non-negative MIN puts full reverse on a stopped motor and neutral at half throttle.

Setting MIN negative anyway looked like the answer and is not. The *parameter store* does take it -
`SIM_GZ_EC_MIN3` set to -300 reads back as -300.0, because `param_info_s` carries a name, a type
and a value and no bounds at all - but the *consumer* truncates it. `MixingOutput` keeps the range
in `uint16_t`:

```
uint16_t _min_value[MAX_ACTUATORS];   mixer_module.hpp:255
_min_value[i] = val;                  mixer_module.cpp:175   (val is int32_t)
if (_min_value[i] > _max_value[i]) { swap }                  mixer_module.cpp:186
```

so -1500 becomes 64036, the swap then makes the range [1500, 64036], and the bridge publishes
`interpolate(value, -1, 1, 1500, 64036)`. Measured on the vehicle, `actuator_test -f 103` at
-1 / 0 / +1 gave exactly **1500 / 32768 / 64036**. `MulticopterMotorModel` clamps its reference
only from above, so every one of those is full forward thrust: the tail pushed at 2.35 N with the
stick centred, and the first-light "yaw stick turns the hull right" check passed on a saturated
tail rather than a commanded one. The metadata bound is advisory; the storage width is not.

PX4 already ships the mechanism, and `uuv_bluerov2_heavy` already uses it:
`src/modules/simulation/gz_plugins/generic_motor`. With `<controlMethod>duty_cycle</controlMethod>`
and `<bidirectionalMotor>1</bidirectionalMotor>` the plugin centres the range itself -

```
ref = (v - (minDuty + (maxDuty - minDuty) * bidir / 2))
      / ((maxDuty - minDuty) - (maxDuty - minDuty) * bidir / 2)      GenericMotorModel.cpp:717
```

- which for bidir = 1 is `(v - centre) / half-span`, a signed -1..1, and `motorType
force_polynomial` then runs separate positive and negative thrust curves off it. The whole range
stays non-negative on the wire and the sign lives in the model. The tail now uses the bluerov2's
1100/1500/1900 duty range with `positiveThrustPolynomial [0, 0, 2.3544, 0]`, which is the velocity
model's `1.0464e-06 * 1500^2` written as a curve, and `SIM_GZ_EC_DIS3 1500` so disarmed is the
neutral duty rather than a value below the duty range.

Two things that make this safe and are not obvious:

- the gz ESC bridge constructs `MixingOutput` with `ramp_up = false`
  (`GZMixingInterfaceESC.hpp:81`), so arming steps straight from the disarmed value to the
  commanded one. With a ramp it would pass through duties below `minDutyCycle`, where the plugin
  falls through to using the raw wire value as the reference and a quadratic polynomial would turn
  1200 into meganewtons.
- `FunctionMotors` does not override `defaultFailsafeValue`, so it is NAN, and
  `output_limit_calc_single` returns `_disarmed_value` for a non-finite input. Failsafe is the
  neutral duty too.

A latent defect in that plugin, found while reading it and **not** hit here: in the
`kForcePolynomial` branch the torque polynomial's result is applied with
`link.AddWorldForce(_ecm, rot.RotateVector({0, 0, torque}))` (`GenericMotorModel.cpp:781`), which
adds it to the thrust as a force instead of applying it as a torque. No model in
`Tools/simulation/gz/models` sets `positiveTorquePolynomial` or `negativeTorquePolynomial`, so
nothing exercises it today, and this model leaves them unset as well.

A caution for anyone measuring this: `actuator_outputs` has one instance per output group and
`px4-listener` gives whichever is instance 0, which on the vectored airship is not the ESCs.
Asking it for a rotor speed returned 32768 for a neutral command, which is a PWM count. Read
`/<model>_0/command/motor_speed`, the topic the bridge actually publishes on - and note that with
a bidirectional motor that topic carries a duty, not a rotor speed, so "is it negative" is no
longer the question to ask it. Ask the hull which way it turned, and ask the wire only whether the
three commanded points land on the model's own duty range.

## Physics variants (2026-09-23)

The Cloudship is now a family: `cloudship` (rigid, buoyancy only), `cloudship_am`, `cloudship_fin`, `cloudship_drag`,
`cloudship_am_fin`, `cloudship_am_fin_drag`, airframes 2510/2514/2512/2511/2516/2517 since 2026-09-24 (ones digit =
1 drag + 2 fin + 4 added mass; they were 2520-2527 for one day, and the decades now run generic 2500, Cloudship 2510,
independent 2520 in the order of the flight numbers, with the gazebo-classic Cloudship moved from 2507 to 2519). The drag block that lived in `cloudship` moved verbatim into `cloudship_drag`, so `cloudship_drag` is
the plant every baseline above was measured on, and `cloudship` is a new, undamped plant with its own baseline.

Added mass is Lamb's for the collision ellipsoid's own fineness ratio (a/b 2.9375: k1 0.12555, k2 0.79930, k' 0.45438
on rho V 7.3065 kg): m_x 0.917, m_y = m_z 5.840 kg, I_q = I_r 3.945 kg m^2, written about the link origin with the
0.8986 m offset terms. The classic template's factors (0.04662, 0.9147, 0.75567) are Lamb's for a fineness ratio of
six and were not used; its plugin's hardcoded 6x6 also carries a sway-yaw coupling whose sign disagrees with its own
fin position. The Munk coefficient is therefore 4.92 N m per (m/s)^2 rather than the 6.35 the design note computed
from the template's factors, and the fins' static term is 0.64, a ratio of 7.7 rather than 9.

Measured in probe worlds (`added_mass_check.py` 6/6): yaw inertia 5.69 rigid, +4.09 with added mass; Munk
-4.77 (nose away from the velocity); fins +0.56 on the rigid hull, and 1.07 taken back on the added-mass hull, which
includes the yaw damping the fins produce once the Munk moment has built a yaw rate. The tail thruster's 4.48 N m
holds 5 deg of sideslip up to about 3.2 m/s on this Munk coefficient; above that the plant is directionally unstable
and out of authority, and `cloudship_am_fin_drag` shows it.

**Superseded 2026-09-24:** the fins moved to the family's lift basis (`fin_basis.py`: the joined-panel flat-plate
slope times the slender-body carryover (1 + r/s)^2 of NACA 1307), cla 1.856 -> 7.254, so their static term is 2.50
and they restore half the Munk moment, not an eighth. Re-measured (`added_mass_check.py`, two-regressor fit): static
2.53, damping 3.54, net -2.22 on the added-mass hull. Details in `airship_tail_design.md`.
Under px4 (tail_rtf_probe.py, 2026-09-23): the tail's yaw acceleration is +24.8 / -25.8 deg/s^2 on
`cloudship_am` against the SDF's 26.4, and +21.3 / -20.9 on `cloudship_am_fin`, where the fins damp
the yaw rate as it builds (a hull released at 118 deg/s decays at 8.1 deg/s^2 with fins and not at
all without). The first-light floor of 0.6 tau / I leaves room for that.

Baselines on the family, 2026-09-23 (models 71444cd, autopilot 3185916c7d): `cloudship_first_light.py` 7/7 on all
six plants; `sitl_check.py` 9/9 on `cloudship` and `cloudship_drag`, 8/9 on `cloudship_am_fin_drag`, where Acro's
rate tracking overshoots to 10.4 deg/s and averages 5.2 against a 6.67 command on a hull whose yaw inertia grew
1.7x and which carries the Munk coupling; that is the retune's starting point, not a defect. `variant_check.py` 9/9
(12/12 mutations caught), `added_mass_check.py` 8/8, repeatable to 1% across runs.
