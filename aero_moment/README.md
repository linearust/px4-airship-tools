# Hull drag about the link origin (2026-09-17, gz-sim 8, airship/dev)

`aero_moment_check.py` next to this directory is the check this record justifies.
Run `--analytic` for the cheap half, or bare for the gz half as well (~15 min).

## What was wrong

`airship_vectored_independent_aero (now airship_vectored_independent_drag)` gives the hull quadratic drag through gz's Hydrodynamics
plugin. The plugin does two things at the link origin. It applies its wrench there
(`Link::AddWorldWrench`, the two-vector overload, whose header says "applied to the link at the
link's origin", and `ExternalWorldWrenchCmd.hh` repeats that the message's `force_offset` is
ignored), and it reads the velocity there (`components::WorldLinearVelocity`, "the linear velocity
at the origin of the link frame"). This hull's origin sits 2.05 m below the axis the drag acts on
and 0.8125 m below the centre of mass, so a coefficient set written as if the origin were the axis
is wrong twice over, and both times with the sign reversed rather than the size:

- the drag force acts 0.8125 m below the centre of mass instead of 1.2375 m above it, so forward
  flight pitched the nose **down**;
- a roll rate p gives the origin a sway of `+0.8125p` where the axis has `-1.2375p`, so the lateral
  drag that rate modulates leaned **into** the roll.

## The condition, and the trap in it

The plugin's wrench has to *be* the hull's wrench. Referred to the link origin that is two
conditions, not one:

    F_plugin = F_hull                      driven by the velocity at the axis, not at the origin
    T_plugin = d x F_hull                  d = (0, 0, h), h = 2.05 m

**The centre of mass appears in neither.** That is the check on the algebra, and it is worth
stating because the obvious derivation gets it wrong: match the *moment about the centre of mass*
instead of the wrench and you satisfy the second condition, land on plausible coefficients
(`2 h z_a * c` where z_a = 1.2375 is the axis above the centre of mass), and leave the force
broken. It stays hidden because the force error's own lever through the centre of mass cancels the
coefficient error exactly, so the attitude dynamics come out right and only the translation is
wrong. That set was built, committed, and checked here first, and every measurement below passed
it. Only re-deriving from the wrench caught it.

## The six terms

With `u_hull = u + h q` and `v_hull = v - h p`, and `(x+s)|x+s| = x|x| + 2|x| s + sgn(x) s^2`
(exact, not a truncation, while |s| <= |x|):

| SDF element | equals | value | what it is |
|---|---|---|---|
| `xQabsU` | 2h * xUabsU | -0.9467 | q-induced surge, on the force |
| `mUabsU` | h * xUabsU | -0.4733 | axial drag's arm |
| `mQabsU` | 2h^2 * xUabsU | -1.9407 | that arm's rate term |
| `yPabsV` | -2h * yVabsV | +32.7697 | p-induced sway, on the force |
| `kVabsV` | -h * yVabsV | +16.3848 | lateral drag's arm |
| `kPabsV` | 2h^2 * yVabsV | -67.1778 | that arm's rate term |

Heave has no row: `d` lies along its line of action, so it gains neither an arm nor a rate term.
Yaw has none either, for the same reason. Sampled over 40,000 random states inside the expansion's
domain, this set reproduces the true wrench to 1e-14; the centre-of-mass set misses by up to 29 N
of sway force and 24 N m of roll moment about the origin.

Each one is derived from the coefficient printed next to it in the file, not from the dimensionless
chain that produced that coefficient, so the arithmetic in the table above closes exactly on the
numbers a reader can see. Recomputing from `c_d` and `c_lat` instead moves the fourth decimal by up
to 2.7e-4, which is the parents' own rounding and not a discrepancy.

These are the SNAME names the plugin builds at load time from `"xyzkmn"` x `"UVWPQR"`; they are not
literals in the binary, which is why grepping the .so for them finds nothing. The plugin applies
`coeff * v_j * |v_k|`, sign included: `Dmat(i,j) = -quadAbs[i][j][k]*|v_k|` and the wrench is
`-Dmat*v`.

One term of the shift has no home in this plugin's grammar, a squared rate following the sign of
the velocity rather than of the rate. At the centre of mass it is 3.0 N of sway and 3.7 N m of roll
at 0.3 rad/s, against 578 N m/rad of restoring, and it falls away with the rate.

Outside the expansion's domain, which is near-still air, the terms overshoot rather than reverse.
Rolling at p in still air the hull sees `-1.2375p` of sway and the origin `+0.8125p`, so the set
puts the hull's own contribution to roll damping at 26.4 N m/(rad/s)^2 where the true value is
15.1; with `kPabsP` the total reads 55.5 against 44.2, a quarter high. Over-damped, which is the
side to be wrong on, and it is the price of a plugin whose one reference point is off the axis.

## Measured

Neutrally buoyant hull in still air, pushed by a 10 N force put through the centre of mass (force
at the origin plus the compensating couple), settled, and read as the world z component of the body
x or y axis, so no Euler convention enters. Restoring stiffness is 578.4 N m/rad, which a pure 5 N m
couple confirms to 0.1% on both axes (gz gravity is 9.8, not 9.80665).

| terms | axis | settled | expected | residual swing |
|---|---|---|---|---|
| none | pitch | -0.014047 | -z_c*F/K = -0.014047 | 0 |
| none | roll | -0.014047 | -0.014047 | 0 |
| arm pair only | pitch | +0.021382 | +z_a*F/K = +0.021395 | grows 0.000082 -> 0.000422 |
| arm pair only | roll | +0.021275 | +0.021395 | limit cycle, +-0.11 (6 deg) |
| centre-of-mass set | pitch | +0.021384 | +0.021395 | 0 by 205 s |
| centre-of-mass set | roll | +0.021387 | +0.021395 | 0 by 55 s |
| **all six** | pitch | +0.021438 | +0.021400 | 0 |
| **all six** | pitch, none | -0.014083 | -0.014051 | 0 |
| **all six** | roll | +0.021442 | +0.021400 | 0 |
| **all six** | roll, none | -0.014083 | -0.014051 | 0 |

The last four are the check as it stands, run 2026-09-18: trim measured as the baseline (exactly
zero on this hull, both axes), stiffness measured with a pure couple at 579.75 N m/rad against
578.40 derived, and the push scaled to land on the same tilt on any vehicle. 4 of 4.

So the arm pair alone is not a partial fix, it is a worse one. It gets the static answer right and
takes the roll damping to `-6.46 + 16.07|v|` N m/(rad/s), which changes sign at 0.40 m/s of
sideslip. Pitch has the same defect and hides it, because `mQabsQ` is 6.7x `kPabsP` while `xUabsU`
is 1/34 of `yVabsV`. A free roll release from 5 deg decays on every variant (0.046 -> 0.002 over
30 s), so a decay test does not catch any of this. It needs sustained sideslip.

Note what these measurements could *not* see. Every one of them settles at p = q = 0, and the whole
difference between the centre-of-mass set and the correct one is proportional to a rate. No settled
state of a restored vehicle has a rate, so no settled measurement can separate them; the analytic
half of the check is what pins the force terms, and the derivation is what found them.

## The two force terms' names are read, and that had to be checked separately

`xQabsU` and `yPabsV` are the only coefficients here whose effect no settled state shows, so a typo
in either would be silent: the hull would settle in the right place and swing no more than it
should. Both names come from reading the plugin's own construction rule rather than from any
documentation, so they were verified by making them impossible to ignore. Set to +-100, each one
puts +-81 N m/(rad/s)/(m/s) of damping on its axis through the 0.8125 m lever to the centre of
mass, against 578 N m/rad of restoring, so one sign has to settle hard and the other has to come
apart. They did (2026-09-17):

| | negative | positive |
|---|---|---|
| `yPabsV` | settles at +0.02139, swing exactly 0 | tumbles, swing 0.998 over the full range |
| `xQabsU` | gz server aborts | settles, swing 0.0009 |

An ignored element would have given the same answer twice.

## What a second vehicle found in this check

Eight, none of which a single vehicle could show, because the big airship happens to be symmetric
and uniform in exactly the ways that hide them. Two were small: the geometry reader counted only
box collisions, so the Cloudship's ellipsoid hull displaced nothing and the vehicle read as 7 g of
buoyancy; and it took the centre of mass from `base_link`'s inertial, which is the vehicle's only
while every other link sits on that plane, as the big airship's pods do and the Cloudship's tail
prop does not. The other five are below.

**A hull does not start level.** The check used to compare the settled attitude against zero,
which works for a vehicle whose centre of mass sits directly under its centre of volume. The big
airship's does. The Cloudship's is 22 mm forward of it, so it hangs 2.61 degrees nose down with
nothing applied, measured -0.045537 against an analytic -0.04539, and every case read as a sign error until the trim was measured and removed. It is
measured once rather than per case: every coefficient here is quadratic in a velocity, so at rest
they are all zero and the trim is the same whichever variant is loaded.

**The compensating couple was wrong twice, and both times it showed up as yaw.** It is `r x F`,
and `r` is where the centre of mass is *in the frame the wrench is applied in*. Two separate
mistakes, each invisible on a hull that is symmetric and sits level:

- only the component on the axis under test was applied. That works while the centre of mass is on
  the x axis, which the big airship's is; the Cloudship's is 22 mm off it, so the sideways case was
  also applying 0.035 N m of yaw.
- `r` was taken in the body frame while `ApplyLinkWrench` takes world coordinates. A hull with a
  trim is not square to them: the Cloudship hangs 2.6 degrees nose down, which moves its centre of
  mass 18 mm forward in world terms and leaves 0.029 N m of yaw.

Either one, with nothing to weathercock against it, is about 4 deg/s of steady yaw, 780 degrees
over the settling window. The world-frame push becomes a body-frame anything and the roll case
reads nothing in particular: it came back at 45% of the predicted tilt with a swing a third as big
as the answer. Neither was a modeling error. `r` is now rotated by the attitude the trim run
measured, which is the identity on a hull that sits level, so the big airship is untouched.

**Nothing asserted that the hull floats.** The check reported the displacement and the stiffness
and moved on, so a collision volume set against the wrong mass would have sailed through every
coefficient test while the vehicle climbed or sank under all of them, making every settled attitude
a transient. It now fails on it. That gap was found by mutation testing the check after the second
vehicle went in, and the mutation that exposed it was on the first one: shrinking the big airship's
hull collision from 9.8 m to 9.0 m used to change nothing here.

**A swing that is still there is not a swing that is not going away.** The check failed a case
whose settled value was right to 1.3%, because 180 s later the hull was still visibly ringing. That
is not a defect in the model: the Cloudship's pitch axis has no linear damping term at all, only
the quadratic `mQabsQ`, which at these amplitudes barely damps. An absolute threshold on the
residual swing cannot tell that apart from a limit cycle, and both readings are equally valid
because the median of a symmetric oscillation is still its centre. So the criterion is now three
things: the value must match, the swing must not be growing, and it must be either small or
smaller than it was. A sustained swing fails, a growing one fails, and a decaying one does not.
Checked against every case seen here, including the arm-pair-only roll limit cycle, which holds at
0.008807 -> 0.008801 and fails on "not shrinking" rather than on size.

**The restoring stiffness is measured, not derived, and for a subtler reason than being wrong.**
`mass * g * (cov - com)` is the stiffness of a hull that has finished settling. Nothing here has:
these are pendulums damped by a quadratic term, which at the small amplitudes this check uses is
very weak. At the calibration tilt the big airship's equivalent damping ratio is 0.019, a 22 s time
constant against a 180 s window, and it measures 579.75 against 578.40 derived, 0.2% high. The
Cloudship's is 0.0074, a 59 s time constant, three time constants rather than eight, and it
measures 36.34 against 35.28, 3.0% high. Both are biased the same way for the same reason: a hull
that has not finished moving has moved less, so the couple divided by the tilt reads large.

Predicting from the measured number rather than the derived one is what makes that harmless. The
four cases below run in the same window against the same damping, so they under-settle by the same
fraction, and the bias cancels instead of eating a 5% tolerance. The derived number is still
reported, because the size of the gap is a reading on how far from settled the vehicle is.

## Why the gz half watches for growth

The two pairs fail differently. A missing arm term moves the settled attitude, which any threshold
catches. A missing rate term leaves the settled attitude alone and destabilizes the plant, and on
the pitch axis it does that two orders of magnitude below the roll axis, small enough to sit under
any absolute threshold worth setting. So the check watches the swing over a 40 s window and fails
it for being larger at the end than at the start: the arm-pair-only pitch case reads
0.000091 -> 0.000134 and fails, a correct set reads 0.000000 -> 0.000000.

## Why the SITL yaw gate cannot stand in for this

`sitl_check.py` exercises the yaw axis with the hull turning roughly in place: the stick commands a
yaw rate and the wind cases apply a torque, never a force. Surge and sway stay near zero, and every
term discussed here is quadratic in surge or sway. So all six are inert for the whole gate, and the
gate scores the same 8/10 whether they are right, half right or absent. That is not a gap in the
gate, which is a controller check; it is why this one exists.

## Not a defect

`gz sdf` on this model cannot resolve `model://` at all: the CLI never sets a find callback ("Tried
to use callback in sdf::findFile(), but the callback is empty"), and the "Error Code 25:
FrameAttachedToGraph error" that follows is downstream of that. To parse this family offline, copy
the models and rewrite each `<uri>` to an absolute path; the nested include is what fails, not the
merge. Loading it in a running server is the check that means something.

## The same offset is waiting in added mass (derived 2026-09-17, not yet built)

`<fluid_added_mass>` is not an escape from this. The sdformat 1.10 spec says it is "this link's
fluid added mass matrix **about the link's origin**", the same 2.05 m off the axis. The difference
is that this transform is exact and closed-form, a congruence rather than a series: with
`T = [[I, -S(r)], [0, I]]` for `r = (0, 0, h)` and `A_H = diag(mx, my, mz, Ip, Iq, Ir)` diagonal
about the axis by symmetry,

    A_O = T' A_H T

which is diagonal except for two coupling entries and two inflated rotational ones:

    xq = h * mx              qq = Iq + h^2 * mx
    yp = -h * my             pp = Ip + h^2 * my

`zz`, `rr` and the rest are unchanged. Sanity check on `xq`: the reaction to a surge acceleration
acts at the axis, so its moment about the origin is `-h * mx * udot`, and the generalized pitch
force is `-xq * udot`.

For this hull (a prolate spheroid, 5.25 x 1.25 x 1.25, a 4.2 fineness ratio displacing 34.36 m^3 =
42.1 kg of air) the rough Lamb factors put `my` near 36 kg against a 49 kg dry mass, and `h^2 * my`
near 152 kg m^2 against a dry `ixx` of 35.84. So in roll the offset term is four times the
vehicle's own inertia: getting it wrong is not a correction, it is the whole number. Take the
actual factors from ~/dev/airship-control-design rather than these.

`//include/experimental:params` is **not a way in**, and that is tested rather than assumed
(sdformat 14.9.0, 2026-09-17). The `element_id="base_link"` does resolve through both nested
merge-includes and does reach the inertial, but every action that would introduce a child the
target lacks fails the same way:

    Error Code 8: Could not find element [fluid_added_mass]. Skipping modification for: ...

`action="add"` and `action="replace"` on `<fluid_added_mass>` fail, and so does `action="replace"`
on the whole `<inertial>` with a complete replacement body: the implementation walks the
replacement's children and requires each to exist already. `added_mass_and_fins.md` in the parent
directory has what is left.
