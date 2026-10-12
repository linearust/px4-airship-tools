# The airship family: names and numbers (design, 2026-09-23, revised after review; brought to the built family 2026-09-25)

One grammar for the gz models, the airframe files, the make targets, the
numbers and the display names, with the PX4 convention each rule comes from.
Checked in the tree on 2026-09-23 against PX4 upstream/main 1cfd3f8895 and
PX4-gazebo-models upstream/main 5577035. A first draft renamed the vectored
airship, moved two parameters into rc.airship_defaults and borrowed the x500
era's "Gazebo ..." names; a 33-agent review refuted all three, and this
revision keeps every existing name and number.

## What PX4 writes down, and what it only practises

Written, and therefore binding:

- An airframe is `<unused ID>_<short descriptive name>`, added to the CMakeLists
  "in the relevant section for the type of vehicle" (docs/en/dev_airframes/
  adding_a_new_frame.md:40-42). Nothing about ranges, digits or sharing.
- A flight header must carry `@name`, `@type`, `@class` (the parser aborts
  otherwise), and `@type` must be exactly `Airship` to get the class image
  (Tools/px4airframes/srcparser.py:99-111, :417-427).
- A posix airframe is `<digits>_<PX4_SIM_MODEL>` exactly (init.d-posix/rcS:56); a
  gz one is `<digits>_gz_<model>` where `<model>` is a directory under
  Tools/simulation/gz/models and must not contain `_gz_` (gz_bridge/
  CMakeLists.txt:40, :117; px4-rc.gzsim:115, :142).
- Numbers are unique within a directory (last glob match wins silently,
  rcS:216). The flight and posix directories are independent number spaces:
  every generator scans init.d only, so posix headers never reach QGC or the
  airframe reference (ROMFS/CMakeLists.txt:149-151; cmake/metadata.cmake:38-43).
- The 2015 partition comment in rcout.py puts 2000-2999 under "Standard planes";
  the airship block in init.d/airframes/CMakeLists.txt:128-134 has no range
  comment. 25xx is an airship home only by the precedent of 2507 (fa4818e4679,
  2020) and 2500 (2bc9774170b, 2026).

Derived from the build, not written: two gz airframes may not produce one make
target, so no gz-airframe model may equal another gz-airframe model plus `_` plus
a world name (the target grammar `gz_<model>[_<world>]` is joined by `_` and
never parsed back). Upstream's r1_rover beside the rover world is fine by this
rule and would be forbidden by a stricter one.

Practised, and therefore the thing to fit:

- gz posix numbers: the 2023-2025 files took one sequential block (4001-4021,
  8011) regardless of vehicle type; every class added since takes the flight
  generic's number (rovers 50000/51000/52000 renumbered to it in a6bf5a90665,
  atmos 70000 in 23ad974f303, bluerov2 60002), and a SITL-only variant takes the
  next ones digit in that decade with no flight twin (70001_gz_atmos_dual,
  33c7eed5f19). That is the shape this family has.
- gz model names: lowercase snake_case; `<vehicle>_base` is the plugin-free body
  of a vehicle, wrapped by exactly one spawnable model, never spawned itself and
  never given an airframe (x500_base, omniquad_base); a propulsion or steering
  configuration is part of the vehicle name, never a suffix (standard_vtol,
  tiltrotor, rover_ackermann); sensor variants are `<vehicle>_<sensor>[_<facing>]`
  as thin merge includes of the spawnable wrapper. There is no physics suffix
  upstream: the one aerodynamics-different pair (rc_cessna, advanced_plane) was
  named as a second vehicle, and other physics differences live in worlds
  (windy, underwater). The `[_am][_fin][_drag]` tokens are this family's own
  extension, and have to be: added mass is a link property and fins and drag are
  model plugins, which no world can add.
- posix `@name`: the x500 series says "Gazebo x500 lidar down"; every class added
  on the flight-number pattern names the posix file after the vehicle, as the
  flight file does or with "SITL" ("Generic Differential Rover", "KTH-ATMOS",
  "Advanced Plane SITL"). Since this family follows the flight-number pattern for
  its numbers, it follows it for names too.
- Docs: "## <Class>" then "### <Vehicle> with <Payload> (<Facing>)" in Title
  Case, a make block per target; the index table's Vehicle cell links the
  heading's slug and its text does not always equal the heading (7 of 15
  upstream rows differ). The slug is the third name space.
- The airframe reference is regenerated and committed by CI after a merge; a
  local `make airframe_metadata` is a check that a new flight header parses, not
  a file to commit.

## The family

One generic hull with three propulsion configurations, and one named vehicle.
The propulsion words are the firmware's own: the tilt range (zero = the pods are
fixed) and CA_AIRSHIP_GRP (0 Collective, 1 Independent), module.yaml.

| vehicle | propulsion | yaw | flight | gz model |
|---|---|---|---|---|
| generic airship, fixed | two fixed thrusters | four control surfaces | 2500 | airship_fixed |
| generic airship, independent vectoring | two pods, each on its own tilt | differential tilt | 2520 | airship_vectored_independent |
| Cloudship | two thrusters on one collective tilt | reversible tail thruster | 2507 (its gazebo-classic twin is posix 2519) | cloudship |

2500 and 2507 are upstream's files and numbers. This branch already changes
their bodies (CA_AIRFRAME 16, which the branch introduces, and the CA_AIRSHIP_*
block); this design does not rename or renumber them.

## The grammar

**Model name**: `<body>[_<propulsion>][_am][_fin][_drag]`.

- `<body>` is `airship` for the generic hull, or the vehicle's own name
  (`cloudship`), the way upstream has `x500` and `rc_cessna`.
- `<propulsion>` names the configuration when the body has more than one, in
  the firmware's words: `fixed`, `vectored_independent`, and `vectored_collective`
  should a generic collective pair ever exist. "Vectored" alone would claim what
  the Cloudship also does (it vectors collectively), so the grouping word stays.
  A named vehicle has one propulsion and no token.
- `<body>_base` is the plugin-free hull, wrapped by one spawnable model and never
  spawned: `airship_base` under `airship_vectored_independent` and
  `airship_fixed`. A vehicle with one propulsion is its own
  body and spawnable (`cloudship`, the rc_cessna shape). Both shapes are
  upstream's.
- Physics tokens follow in that order. Every member but `_am` is a thin merge
  include of the member with one token fewer, and the bare model may itself
  merge the family's `_base`. `_am` needs `<fluid_added_mass>` in the hull
  link's `<inertial>`, which a plain merge include cannot change (SDF's
  `experimental:params` can replace it at parse level; untried in gz here), so: a self-contained
  body's `_am` is a full copy of it (`cloudship_am`, held equal to `cloudship`
  but for the block); a `_base` family's `_am` is `<body>_base_am`, the base plus
  the block, held equal to the base the same way, and `<vehicle>_am` merges it
  and restates the propulsion block, held identical to the bare model's as the
  fin and drag blocks already are. Each rule is a check in variant_check.py.

**Airframe number**: the decade is the vehicle; a gz member's ones digit is the
mask of its name's tokens, 1 `_drag` + 2 `_fin` + 4 `_am`, and 9 is the
gazebo-classic twin. The fixed generic airship's fins are its flaps, in every
member, so its family has no `_fin` token and uses 2500, 2501, 2504 and 2505
only; 2502, 2503, 2506 and 2507 stay unused on purpose.
Flight numbers are not masked (2507 has ones digit 7 and is a different vehicle
from 2500). The decades run in the order of the flight numbers, which is also
the order of propulsion complexity the maintainer asked for: the generic
airship first, the vehicle with a yaw thruster next, the independently
vectored one last.

| vehicle | flight | gz decade | gz members |
|---|---|---|---|
| generic airship, fixed | 2500 | 2500 | 2500, 2501, 2504, 2505 |
| Cloudship | 2507 | 2510 | 2510, 2511, 2512, 2514, 2516, 2517; classic twin 2519 |
| generic airship, independent vectoring | 2520 | 2520 | 2520, 2521, 2522, 2524, 2526, 2527 |

2500 and 2520 share their flight numbers with gz, which is upstream's own
pattern where it can hold. 2500 and 2507 share the 2500 decade, so 2500's
decade belongs to the generic airship's gz family and the Cloudship's gz
family takes the next one. Upstream's gazebo-classic Cloudship, which sat at
2507 in the posix directory, sits at 2519 with the rest of the Cloudship's
family. The first reason given for the move, a slot for the generic family's
mask 7, went away when that family dropped the fin token; the move was kept
on 2026-09-25 because the classic model has not been maintained for about
four years and one decade per vehicle is the cleaner scheme, at the cost of
upstream's flight/posix 2507 pairing. The make target
`gazebo-classic_cloudship` is unchanged by that move, since targets come from
the name after the simulator token, and rcS resolves the number from the
name; only a user launching by `PX4_SYS_AUTOSTART=2507` on the classic sim
notices. The table lives in one place in the tree, as a comment above the
25xx entries of init.d-posix/airframes/CMakeLists.txt, which variant_check.py
reads.

**Airframe file**: `<number>_gz_<model>`, thin for a physics member (the
`PX4_SIM_MODEL` default and a source of the decade's base airframe, as x500's
sensor variants source 4001). Flight filenames are upstream's habit and not
constrained by the model grammar (2500_generic_airship, 2507_cloudship,
2520_airship_vectored_independent).

**Display names**:

- flight `@name`: what QGC and the reference show, unchanged: "Generic Airship",
  "Generic Airship (Independent Vectoring)", "Cloudship".
- docs heading: the flight name with the physics in parentheses, Title Case:
  "Cloudship", "Cloudship (Hull Drag)", "Cloudship (Added Mass, Fins)",
  "Cloudship (Complete)", "Generic Airship (Independent Vectoring, Hull Drag)".
- posix `@name`: the docs heading, verbatim, and no simulator token: the
  "(gz, ...)" of the first version goes.
- links: index.md and frames_airship/index.md link the heading's slug; the
  check resolves every such link against the headings present, since the docs
  build ignores dead links.

**Class defaults**: stay as they are. Moving CA_AIRFRAME 16 and COM_PREARM_MODE 2
into rc.airship_defaults reads well against rc.fw_defaults and rc.heli_defaults
but would change upstream's 2507_gazebo-classic_cloudship (2519 on this
branch), which sources the defaults, never sets COM_PREARM_MODE, and keeps its
own CA_AIRFRAME 9; the repetition in six airframes is the smaller cost.

## What the 2026-09-23 revision changed in the tree

In that day's numbers, since renumbered (see Aliases):

- the six Cloudship posix `@name`s lost their "(gz, ...)" token; 2520 became
  "Cloudship" and 2521 "Cloudship (Hull Drag)", matching the docs headings;
- the decade table was written above the 25xx entries of
  init.d-posix/airframes/CMakeLists.txt, with 2500 then reserved for the
  generic airship's gz twin (built 2026-09-24, Decisions 1);
- variant_check.py gained: the `_base` rule as upstream practises it (never an
  airframe, at least one merge includer), the display rule (posix `@name` equals
  the docs heading of the same target; every airship link in index.md and
  frames_airship/index.md resolves to a heading), the target-clash rule as
  CMake would hit it, and 25xx uniqueness across both airframe directories.

Nothing is renamed: `airship_vectored_independent` is already on the fork's
published branches and the review found the name is the firmware's own phrase.
The numbers did move, once more, to put the decades in the order above
(2026-09-24): the Cloudship gz family 2520-2527 -> 2510-2517, the independent
airship 2510/2511 -> 2520/2521 on both sides, the classic Cloudship 2507 ->
2519. The bench FMU carried 2510 (the independent flight airframe); after the
user flashed 9535f8abcd on 2026-09-25, `head_20260924/renumber_fmu.sh` moved it
to SYS_AUTOSTART 2520 (head_20260925/t0_identity.log, README.md).

## Aliases, for the records that keep the old names

| old | now | when |
|---|---|---|
| 2504, 2505 (SITL) | 2510, 2511 | 8a5cf11cd08, 2026-09-17 |
| airship_vectored_independent_aero, 2511_..._aero | ..._drag | 71444cd / 3185916c7d, 2026-09-23 |
| 2508_gz_cloudship | 2520_gz_cloudship | 3185916c7d, 2026-09-23 |
| 2520-2527 (Cloudship gz), 2510/2511 (independent), 2507 (classic posix) | 2510-2517, 2520/2521, 2519 | 2026-09-24 |

The identity harness names its parameter sets 2500/2504/2507/bench; the pass9
and preset2500 snapshots carry SYS_AUTOSTART 2504; head_20260923 carries 2510;
head_20260925 and head_20260926 carry 2520. None of these are edited.

## Decisions taken since

1. `airship_fixed` was built at 2500 once the tail was specified
   (airship_tail_design.md), with 2501, 2504 and 2505 (6e74f20, 2026-09-24).
2. `airship_vectored_independent_am`, `_fin`, `_am_fin` and `_am_fin_drag` were
   built by the `_base_am` rule (ac41e6c, 2026-09-24).
3. The classic Cloudship stays at 2519 (2026-09-25, above).
