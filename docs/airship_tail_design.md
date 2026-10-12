# The generic airship's hull and tail: a sizing, not a drawing (rev. 3, 2026-09-25)

The generic gz airship (airship_base, 49 kg) had a cruciform tail drawn as two
crossed plates, visual only, and no specification behind it. This sizes fins and
control surfaces for it from stated requirements with cited methods, so that
the `_fin`/`_am` members of its family and the gz twin of the 2500 flight
airframe (fixed thrusters, elevators and rudders) can exist. Every number is an
engineering estimate on a stated basis, not a measurement. The numbers come
from `generic_hull_design.py`, which also runs the requirement checks;
`apply_generic_hull.py` writes them into the model files (idempotently);
`fin_basis.py` holds the lift basis and `hull_basis.py` the added-mass block,
and `variant_check.py` holds the files to them (facts 3, 4 and 14, fact 14
being the requirement itself).

## One hull

One prolate spheroid of fineness 4.2, 10.97 x 2.61 m (radii 5.4857 x
1.3061), drawn and collided alike. Its volume is the 39.2 m^3 the old box
collision had, so the 40 m^3 that lifts the 49 kg and the mass budget are
unchanged, and the hull axis stays at z 2.05 m, so the centre of volume
(2.017 m) and the link-origin offset of the added-mass and drag blocks
(h = 2.05 m) are unchanged too. Everything else is sized on this body: the added
mass, the Munk moment (38.03 N m per (m/s)^2 of u v), the drag reference
and the tail. The gondola's top sits 5.6 cm inside the envelope.

## Requirements

- **R1, yaw:** the dorsal and ventral fins restore at least 1.2 x the hull's
  potential-flow Munk moment at every speed (NACA Report 184; Munk's guidance is
  that fins nearly neutralise a hull, and a small rate-flown airship is better
  slightly stable), **judged at zero sideslip on the attached-flow slope**
  (Helmbold's), since small-perturbation stability is what the requirement is
  about. The plate slope the plugins carry is a secant to about 10 deg whose
  excess over lifting-surface theory is side-edge vortex lift, quadratic in
  incidence, so it says nothing at zero sideslip; its ratio is reported.
- **R2, pitch:** the side fins against the Munk pitch moment on the same basis,
  the pendulum (576 N m/rad) on top.
- **R3, control inside the travel:** a full-rudder turn on the added-mass plants
  keeps both vertical fins at least 2 deg below their stall, transient included
  (sway-yaw simulation with the lift-drag plugin's semantics: stall on the
  geometric incidence, the flap added after, cd as |cd|). This sets the flap
  travel.
- **G1, ground:** the ventral fin's tip stays 5 cm above the gondola floor, the
  ground contact, at level attitude. It is a drawing constraint: 0.6 deg of
  nose-up on the ground uses the 5 cm up, and the fins have no collision, so
  the simulation never touches. A vehicle built to this needs a stated
  ground-handling pitch; at 5 deg the ventral span would be about 0.85 m and
  R1 would move the difference to the dorsal fin.

## Method

Lift: each fin with its mirror image, AR_j = 2 span / chord, the flat-plate
slope 2 pi AR_j / (AR_j + 2) that Ananda, Sukumar and Selig (2015) measured for
AR 2-3 plates at Re 6e4-1.6e5 (NACA 1307 asks for the measured wing-alone
slope), times the slender-body carryover (1 + r/s)^2 (NACA 1307 eqs. 14 and
21), r the hull radius at the mid-chord station, s the root plus the span.
Stability is checked on Helmbold's 2 pi AR / (2 + sqrt(4 + AR^2)) with the same
carryover.

Stall: the hull's upwash raises a fin's own incidence to K_W(B) times the body
angle (the equivalent-angle-of-attack method, Hemsch and Nielsen 1983; the
slender-body K_W(B) of NACA 1307), so the fin reaches the plates' CLmax, 0.7 (the
top of the 0.55-0.7 the same data measured), at 0.7 / (plate slope x K_W(B)) of
body angle. Past it the plugin's slopes take cl to 0 and cd to a broadside
plate's 1.2 at 90 deg. The flap margin rests on the 0.7: at the 0.55 bottom of
the measured range the full-rudder turn of R3 sits at 98 % of the stall, with
no margin left. The plugin stalls the geometric incidence and adds the
flap after, so a flap opposing the incidence does not unload the fin in the
model as it would on a real one; at a rudder reversal the other way round.

Control surfaces: 30 % chord flaps, thin-airfoil effectiveness 0.661, times the
~0.8 viscous factor Caughey's "nearly 50 percent of the lift-curve slope for a
25 percent chord flap" implies, times the plate slope, times K_W(B) alone (a
deflected panel on a body at zero incidence gets NACA 1307's incidence
factors, whose slender-body sum is K_W(B)). Linear over the travel.

Fin drag: below the stall the plugin's cd is cda times the incidence, cda
sized to the induced drag CL^2 / (pi AR e) with e 0.9 at the lower of 10 deg
and the stall, because past the stall the plugin uses cda_stall instead. On
these fins the stall is the lower.

Drag: the reference design's dimensionless coefficients (an 8.3534 x 2.3693 m
hull) on this hull: frontal radius 0.9285 x the half-height (A 4.6208
m^2); the lateral coefficient, crossflow drag, also scaled by the fineness
ratio (x 1.1913, the planform); the axial one by Hoerner's
fineness dependence (x 1.0902); rotation on S L^3 (S 72.3849 m^2,
L 10.9714 m, 201.3011 N m). The six cross terms for the 2.05 m offset
follow (`aero_moment_check.py --analytic` 14/14).

## Result

| item | value |
|---|---|
| fins | cruciform, 1.20 m chord; dorsal and side 1.52 m, ventral 1.28 m (G1) |
| station | trailing edge -5.186 (0.30 m short of the stern), quarter chord -4.286, root radius 0.717 |
| dorsal/side | AR_j 2.533, cla 6.122 (Helmbold 5.309), carryover 1.744, K_W(B) 1.272, stall 0.1567 (9.0 deg), flap 2.361/rad, cda 0.8201, cda_stall 0.7577 |
| ventral | AR_j 2.133, cla 5.989 (Helmbold 5.027), carryover 1.847, K_W(B) 1.309, stall 0.1649 (9.4 deg), flap 2.244/rad, cda 0.9805, cda_stall 0.7385 |
| R1 | 45.76 / 38.03 = **1.203** on Helmbold; 1.408 on the plate secant |
| R2 | 1.339 on Helmbold, the pendulum on top |
| R3 | flaps **+-11 deg**: full rudder holds the fins at 6.9 deg (_am) and 6.7 deg (_am_drag), 0.77 of stall, at every speed |
| G1 | ventral tip 0.053 m above the ground |
| masses | hull 9.497, fins 3 x 0.651 + 0.549, gondola 36 at x 0.3277 |
| centre of mass | (0.0066, 0, 0.8169), under the centre of volume, 4.4 mm above the pods' plane (a 0.16 N m trim at full thrust against 576 N m/rad) |
| inertia | base_link ixx 41.08 iyy 151.48 izz 126.90 ixz 20.07; vehicle yaw 129.1 |

## Consequences worth knowing

- **Turns at 4 m/s, full rudder:** rigid twin 0.00 deg/s (the fins are its
  only side force, so a held rudder sets a 4.1 deg sideslip rather than a
  turn), drag twin 0.23 deg/s, added-mass twins 2.7 and
  2.7 deg/s. The docs and the posix 2500 comment say the added-mass
  members are the ones that turn.
- **The vectored airship's turn envelope:** full pod yaw (54 N m) from rest with
  u held gives 5.1 deg/s at 2.5 m/s, 3.3 at 4 and 2.2 at 6 on the
  complete plant; at 2 m/s the pods out-turn the tail and it stalls
  (25 deg/s); from rest both vertical fins stay attached from 2.5 m/s up. The stall is
  hysteretic: once on the stalled branch (reached at 2 m/s), holding u at
  2.5-6 m/s keeps them stalled, turning at 28-51 deg/s. So AS_YAWRATE_MAX 15 deg/s is
  unreachable in forward flight, and the 2520 loop winds its integrator there
  with no shortfall reported. That needs a speed-aware loop (the next
  increment); it is not a tail defect.
- **The asymmetric vertical pair** puts a rolling moment on sideslip (the dorsal
  fin is larger and higher), which the pendulum takes; the sway-yaw design
  model leaves roll out.
- **The fins' own added mass** is not modelled.
- **Rate damping read at the quarter chord.** The plugin takes the
  rotation-induced incidence at cp, the quarter chord, so the fins' yaw and
  pitch rate damping is K u l^2 with l to c/4 (the engine measured 216.9 u N m
  per rad/s of yaw damping on the rigid _fin plant against 229.8 from the
  files, chain5). Thin-airfoil theory takes it at the 3/4-chord point (arm
  4.89 m against 4.29 m), about 14 % more, so the quarter-chord reading is
  about 12 % low. The carryover (1 + r/s)^2 also multiplies
  the roll-rate incidence, where body carryover does not apply, so roll
  damping is overstated. Both are small and err toward less yaw damping.
- **World wind reaches the fins only.** LiftDrag subtracts the WindEffects
  wind; Hydrodynamics reads only its current topic, and the engine's added
  mass works on inertial velocity. A world wind would give the fins
  air-relative flow and the hull ground-relative drag and no Munk moment. The
  checks apply wind as a hull torque; world wind is not usable on these
  plants.

## Added mass

Lamb at fineness 4.2 (k1 0.07607, k2 0.86795, k' 0.62949), rho V
48.02 kg, about the link origin: xx 3.653, yy = zz 41.679,
xq 7.489, yp -85.442, pp 175.16, qq 207.60, rr
192.24; Munk -38.03 u v N m. In airship_base_am.

## The 2500 twin: airship_fixed

airship_base plus the vectored airship's pod and prop links on fixed mounts
(0.398 kg pods so the vehicle is 49.000 kg) plus the four fins WITH flaps on
revolute joints, servo_0..3 in the 2500 flight airframe's output order, travel
+-11 deg with SIM_GZ_SV_MINA/MAXA to match. With collective pods and no
tail, firmware from e550ec6597 (keep yaw stick as torque on surface-only yaw)
on keeps the yaw stick as rudder torque in every mode (`airship_att_control`:
the yaw rate loop closes only when the airship allocator's pods or tail get
yaw to make), because the loop has no airspeed
scaling: closed on the rudders it capped them at 43 % and held 30 % after
release (measured at the pins).

## What the engine measured

Every run below flew the generic fins' earlier cda, sized at 10 deg, which is
past these fins' stall (0.9133 dorsal and side, 1.0379 ventral; cda_stall
0.7474 and 0.7318). The results on those fins (the airship_fixed flights, the
vectored airship's finned gates, the fin terms of its added-mass check) have
not been re-run on the current values yet; the Cloudship's fins did not
change.

Rev. 3 chain, 2026-09-25 (sitl_out/chain5_20260925.txt), firmware with the
surface-only yaw gate:

| check | result |
|---|---|
| variant_check | 14 / 0 in the chain; 16 / 0 on the tree of 2026-09-27, with fact 15 (servo range = joint limits) and fact 16 (each gz twin's CA_* block = its flight airframe's) added since |
| added mass, vectored airship | 9 / 9: rigid yaw inertia 129.28 (SDF 129.15), added rr 194.3 (192.2), roll period about the link origin 2.297 s (origin prediction 2.278, CoM 3.075), Munk -38.45 (-38.03), fin static 51.54 (53.54), fin damping 216.9 (229.8), net static on the added-mass plant 14.14 (15.52) |
| added mass, Cloudship | 9 / 9 |
| airship_fixed flight, four members | 12 / 12 each: flaps at +-11.0 at rest with the hull still; Stabilized at rest, half stick 5.5 deg held and 0.0 after release; thrust acceleration as predicted; yaw at about 3 m/s 2.1 (rigid), 2.3 (drag), 2.7-2.8 (am), 2.6 (am_drag) deg/s against predictions within 0.2 |
| Cloudship gates | rigid 9 / 9, _fin 9 / 9, _am_fin_drag 8 / 9 (Acro tracking overshoot, 10.4 deg/s peak on 6.67; the same on 2026-09-24) |
| vectored airship gates | 8 / 10 on rigid, _drag, _fin and _am_fin, as on 2026-09-24 before rev. 3; the _am_fin_drag run stopped after five checks (its 8 / 10 is in chain6 and, from a clean start, chain7): rigid overshoots (Acro brake 27.8 -> -7.6 deg/s, tracking peak 9.6 on 6.67); _drag, _fin and _am_fin release too slowly for the window (0.5-2.2 deg/s at its end, still falling). The AS_YAWRATE_* defaults were tuned on 79 kg m^2 without a tail; the plant is now 129 kg m^2 with fin damping. Retuning is the next increment. |
| tilt range | 8 / 8 |

Final tree, same night, with the checks tightened (flight rates to 10 %, a
10 s held-rudder leg, gate tracking steps started at rest with the integrator
cleared): sitl_out/chain7 (all four flight twins, four gates), then chain8
(airship_fixed and _am again, after the two fixes below). chain6 flew the
same models under the old 30/40 % flight tolerance with no held leg and no
clean start; it is cited only for the _am_fin_drag gate chain5 cut off.

| check | result |
|---|---|
| held full rudder, 10 s, last second | rigid -0.03 deg/s (model +0.01, chain8); am +1.86 (1.93, chain8); drag +0.15 and am_drag +1.89 (chain7, whose check predicted at the entry speed, 0.19 and 2.13, both passing; at the last-second speed the same model gives 0.18 and 1.95, recomputed offline from the held-leg CSVs). The drag-free and complete twins slow from about 3 to 2.7 m/s in the turn, so the model is read at that speed |
| vectored airship gate, rigid | 9 / 10: tracking passes from a clean start (peak 8.8 on 6.67); the one failure is the known Acro brake (27.8 -> -7.7 deg/s) |
| vectored airship gate, complete | 8 / 10: release still 2.7-2.8 deg/s and falling at the end of the window |
| Cloudship gates | rigid 9 / 9; complete 8 / 9, Acro peak 9.42 on 6.67 from a clean start (limit 9.34; 10.4 before the clean start) |

chain7 failed twice. On rigid, right yaw read its onset speed as 0.00: the
pose read taken before the stick came back empty and the speed defaulted to
zero; the onset speed now comes from the leg's own first sample. On _am, the
held leg read +1.86 against +2.18 predicted at the 3.10 m/s entry speed; the
held prediction is now taken at the last second's speed. chain8 confirmed
both, on rigid and _am only.

The flight check's model changed on 2026-09-27: each fin's force is now the
lift-drag plugin's own cl and cd on the flow angle, stall included, and the
pitch added mass is the block's qq moved to the centre of mass. Replayed
through it on the cda they were flown with, the chain7/chain8 records match
within 2.5 % in yaw and 4.5 % in pitch at onset. The held legs are predicted
at +0.01 (rigid) and +0.17 (drag), within the 0.1 deg/s floor of what was
measured, and 0.8 % and 0.4 % above it on am and am_drag. A run on the
current cda followed: the 2026-09-27 run on the current fin drag (sitl_out/chain9_20260927.txt, records in sitl_out/fixed_flight_20260927_chain9/) passed all four flight twins 13 / 13, the held turns within 0.02 deg/s of the model (rigid -0.03 vs +0.01, drag +0.15 vs +0.17, am +1.89 vs +1.90, am_drag +1.90 vs +1.91 deg/s at the last second's speed), and the added-mass check 9 / 9; the gates kept their baselines: Cloudship 9 / 9, vectored rigid 9 / 10 (the known Acro brake, 27.7 -> -7.6 deg/s), vectored _fin and _am_fin_drag 8 / 10 (release 1.0 and 2.7 deg/s still falling at the window's end), Cloudship _am_fin_drag 8 / 9 (Acro peak 9.54 on 6.67 from rest). The 2026-09-25 leg records that replay used are kept in
sitl_out/fixed_flight_20260925/ for the drag, am and am_drag twins; the rigid
twin's chain8 records were overwritten by the 2026-09-27 run, so its replay
figures survive only as the text in that folder (replay_head_chain8_rigid.txt,
replay_final_head.txt).

So the vectored airship's slow release and the complete Cloudship's Acro
overshoot are the loop's own on these plants, not artefacts of the gate; they
are the retune's inputs.

## Revisions

- 2026-09-24: first sizing on the drawn ellipsoid, lifting-line slope; then
  Helmbold as the bound; then the plate slope. Stall 0.3 rad, flaps +-25 deg.
- 2026-09-25 rev. 2: the 48-finding design review: one hull volume, plate-CLmax
  stall, post-stall to a broadside plate, R3, flaps +-15 deg, the requirement
  stated on the plate slope, symmetric 1.20 x 1.28 m fins.
- 2026-09-25 rev. 3: the 34-finding implementation review: the requirement back
  on Helmbold's slope at zero sideslip (the plate slope's excess is vortex
  lift), the stall corrected for the hull's upwash, hence a larger dorsal and
  side span (1.52 m) and a shorter flap travel (+-11 deg); lateral and axial drag
  corrected for fineness.
- 2026-09-27: fin cda sized at the lower of 10 deg and the stall (the stall
  here), since past the stall the plugin uses cda_stall: cda 0.9133 -> 0.8201
  and 1.0379 -> 0.9805, cda_stall 0.7474 -> 0.7577 and 0.7318 -> 0.7385, cd at
  90 deg still 1.2. R3's worst fin incidence 6.89 -> 6.90 deg (_am) and 6.71
  -> 6.73 (_am_drag), travel unchanged. No SITL run on these values yet.

## Sources

- Munk, M. M., "The Aerodynamic Forces on Airship Hulls", NACA Report 184, 1924
  (eq. 19, 22; §8 "required size of the fins"; Table I).
- Pitts, Nielsen, Kaattari, "Lift and Center of Pressure of Wing-Body-Tail
  Combinations", NACA Report 1307, 1957 (eqs. 14 and 21 are the slender-body
  K_W(B) and K_B(W), summing to (1 + r/s)^2; the upwash approximation the
  report rejects is eqs. 15-17; p. 569 defines the wing alone as the exposed
  panels joined; p. 5 asks for the experimental wing-alone slope).
- Lamb, Hydrodynamics, 6th ed., §§114-115 (k1, k2, k' closed forms).
- Caughey, D. A., "Introduction to Aircraft Stability and Control", Cornell
  M&AE 5070 notes, 2011, §2.7 (flap effectiveness: ~50 % of the lift slope for a
  25 % flap against the theory's 61 %, a viscous factor of about 0.8).
- Ananda, Sukumar, Selig, "Measured aerodynamic characteristics of wings at low
  Reynolds numbers", Aerosp. Sci. Technol. 42 (2015) (low-AR plate slopes and
  stall).
- Skyship 600 outline specification, section 12 "Tail surfaces" (hinge line).
