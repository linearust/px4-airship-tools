# Added mass and fins: what is blocking, and what the decision is (2026-09-17)

Four things were flagged as wrong or missing in the airship simulator. Two are now fixed and
committed locally; two are not, and they are not blocked on effort.

| | state |
|---|---|
| the tilt servo could rotate forever | fixed, gz submodule 833b938 |
| the drag moment had the wrong sign | fixed in the gz submodule, and it was wrong in two ways and cost six terms, see `aero_moment/README.md` |
| added mass | open, and a design decision, below |
| the tail does nothing | open, and short of data, below |

## They are one change, not two

`<fluid_added_mass>` does not only make the hull heavier to accelerate. The physics engine folds it
into the spatial inertia, so its Coriolis terms appear too, and for a slender hull that is the Munk
moment: a destabilizing pitch-and-yaw couple proportional to the product of axial and lateral
velocity. It is what makes a bare airship hull want to fly sideways, and it is what a tail exists
to beat.

The reference design at ~/dev/airship-control-design has measured the size of this. From
`airship/dynamics/wrenches.py`:

> The omitted Munk moment `(M22 - M11)` is not small: over the shipped 340 s scenario it reaches
> 72 N m in yaw, against the 19 N m of aerodynamic yaw damping the model does carry. Adding it is a
> design change for the team to make deliberately, not a correction to slip in -- it moves every
> closed-loop fingerprint.

So the reference design carries added mass in its mass matrix and deliberately leaves the Munk
moment out of its Coriolis term, and says so in two places. Putting `<fluid_added_mass>` into the gz
model would put the Munk moment back, because a physics engine cannot be told to keep one and drop
the other. Shipping it without fins would hand the simulator a 72 N m destabilizing couple against
19 N m of damping, which is a worse plant to fly a controller against, not a better one.

## Added mass: where it can go

The transform itself is settled and exact: `<fluid_added_mass>` is defined about the link origin,
2.05 m off the hull axis, so the matrix is diagonal-plus-two, `xq = a*mx`, `yp = -a*my`,
`pp = Ip + a^2*my`, `qq = Iq + a^2*mx`. `aero_moment/README.md` has the derivation. In roll the
offset term is roughly four times the vehicle's own inertia, so this is not a correction, it is the
number.

What is not settled is which link it goes on, because it lives inside `<inertial>` and the aero
model reaches `base_link` only through two nested merge-includes. Three routes:

1. **`//include/experimental:params`.** Ruled out by test, not by taste. The `element_id` does
   resolve through both merge-includes and does reach the inertial, but every action that would
   introduce a child the target lacks fails with `Error Code 8: Could not find element
   [fluid_added_mass]`, including `action="replace"` on the whole inertial with a complete body.
2. **A dedicated link on the hull axis**, fixed-jointed, carrying the added mass. It is diagonal
   there, so no transform at all, and pointing the Hydrodynamics plugin at the same link would let
   the six drag cross terms go as well. The cost is a link that needs real mass, which has to be
   paid for with a matching collision volume so the hull stays neutrally buoyant, and a fixed joint
   whose behaviour under a 36 kg added mass against a 0.1 kg real mass is untested.
3. **Put it in `airship_base`** so both plants carry it. That is arguably the better split anyway,
   two plants differing only by dissipation rather than by inertia, but it triples the yaw inertia
   of the reference plant (izz 76.5 -> 243) and invalidates every SITL gate baseline.

## Fins: the data is missing, not the mechanism

The mechanism is easy and, unlike Hydrodynamics, correct out of the box.
`gz-sim-lift-drag-system` takes a `<cp>`, reads the velocity at that point
(`worldLinVel + worldAngVel x cpWorld`) and applies the force there
(`totalTorque = torque + cpWorld.Cross(force)`). No arm terms, no rate terms, no expansion. Four
plugin blocks in the aero model and nothing touches a link.

What is missing is the tail. The reference design has no fin coefficients at all: its
`aerodynamic_wrench` is `c_d`, `c_lat`, `c_lp_lin`, `c_lp_quad`, `c_mq`, `c_nr` and nothing else,
which is why it has no weathercock stiffness either. The gz model's tail is a cruciform of two
1.0 x 3.6 m boxes at x = -4.3, marked "visual only" and, on the evidence, illustrative. Sizing a
lift slope off that planform would be modeling a tail nobody has specified.

## The Cloudship changes this, for the Cloudship

Since 2026-09-17 there is a gz Cloudship, and unlike the generic airship it has a tail that was
measured rather than drawn: four fins of 0.465 m chord and 0.39 m span, from 3.635 m back and
0.5634 m off the axis, out of the gazebo-classic template's parameters. The gap there is narrower
than it looks: what is missing is a lift slope, and a lift slope off a known planform is an estimate
with a stated basis (finite-wing theory on an aspect ratio of 3.6, discounted for sitting in the
hull's flow) rather than an invented number. The generic airship's tail is the illustrative one; its
`airship_base` comment says so.

So the two vehicles are in different positions. The Cloudship could have fins now, sized from its
own drawing, and then added mass, in that order and in one change. The generic airship cannot, until
someone says what its tail is.

### But the Cloudship's fins do not buy it stability

Worth doing the arithmetic before building it, because it says something about the vehicle rather
than about the model. Its fins are stubby: 0.465 m chord against 0.39 m span is an aspect ratio of
0.839, so an isolated-wing lift slope of `2*pi*AR/(AR+2)` is 1.86 per radian on 0.1814 m^2 each, and
that is generous, since a fin in a hull's flow does worse. Its added mass from the template's own
factors is 0.341 kg axial against 6.689 kg lateral, so the Munk moment is `6.348 * u * v`.

| speed | sideslip | Munk | two vertical fins | ratio |
|---|---|---|---|---|
| 3 m/s | 5 deg | 5.00 N m | 0.55 N m | 9.2x |
| 5 m/s | 5 deg | 13.89 N m | 1.51 N m | 9.2x |
| 8 m/s | 10 deg | 71.64 N m | 7.94 N m | 9.0x |

The ratio does not move with speed, because both scale as V^2. So the fins are an order of
magnitude short, and so is the tail thruster: 3.93 N m holds 5 degrees of sideslip only up to
2.7 m/s. Modeling added mass on this vehicle does not give it a plant that fins stabilize; it gives
it a plant that is directionally unstable and whose yaw authority runs out at cruise. That may well
be the truth about a 4.7 m airship with 0.39 m fins, and if it is, it is worth knowing before
anyone flies it. It is also a much bigger change than "add the missing inertia".

## What I would do

For the generic airship, wait for the real vehicle's tail and land both together with the fins sized
against it. A hull that is being built is the right source, and the two have to arrive together
anyway.

If the tail is still open when this becomes urgent, the honest fallback is to size the fin from the
requirement rather than the drawing: pick the area that makes the hull directionally neutral at
cruise against its own Munk moment, and say in the model that the tail is a sizing result and not a
measurement. That is how tails get sized. It is also vehicle design rather than simulator work, so
it should be a deliberate choice and not a default.

Until then the model is honest about both: the tail comment says it adds no weathercock stiffness in
either variant, and the aero model's header says added mass belongs in `<fluid_added_mass>` and is
not modeled yet.

## Decided and built 2026-09-23

Route 1 above was ruled out by test and route 2 never tried; what shipped is a fourth: a full copy of the base SDF
with the block added (`cloudship_am`), held equal to the base by `variant_check.py`, and every other physics variant a
thin merge include chained in a fixed token order. See `README.md` (physics variants) and `cloudship_sitl.md`.

Two corrections to the arithmetic above. The template's k factors are Lamb's for a fineness ratio of six, not this
hull's 2.94, so the Cloudship's added mass is m_x 0.917 / m_y 5.840 (not 0.341 / 6.689) and the Munk coefficient
4.92, not 6.35; the fins' ratio is 7.7, not 9.2, and the tail thruster holds 5 deg of sideslip to about 3.2 m/s,
not 2.7. And the destabilising Munk moment turns the nose AWAY from the velocity: in gz's y-port, z-up body frame that
is a NEGATIVE yaw rate for a sideslip to port, which the first measurement run got wrong in its expectation, not in
the engine. The generic airship still waits for a tail specification; `airship_vectored_independent` has no `_am`
or `_fin` yet, only `_drag` (renamed from `_aero`).

**Superseded 2026-09-24:** the generic airship's tail is now sized (`airship_tail_design.md`) and its family has
every member; the Cloudship's fins moved to the same basis (cla 7.254), which restores half its Munk moment. The
"generous" remark above had the hull's effect backwards: slender-body theory says a hull RAISES a fin's lift.
