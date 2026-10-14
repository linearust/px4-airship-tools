# Overlay: flying what upstream does not carry

Upstream PX4 is meant to carry one complete plant per airship family and the
`lta` world. The physics-variant members (`_am`, `_fin`, `_drag` and their
combinations) and the literature vehicles exist to tell the model's share of
a result from the loop's share; they are validation apparatus and live here.

The overlay is three directories and `vehicles.md` the generator fills (`airship_family_gen.py
<vehicle.json> --overlay`: the whole family under the rigid naming, a hundred
above the vehicle's decade, self-contained with its own added-mass body;
`airship_family_gen.py cloudship --overlay`: the hand-written Cloudship family
copied the same way; the tree's plain plant, written by `--upstream`, is held
to the overlay's complete member element for element by the generator test
and by `variant_check.py`'s fact 17) and nothing in the PX4 tree changes:

| directory | reaches PX4 through |
|---|---|
| `models/` | both `overlay/models` and the tree's models are on the gz server's `GZ_SIM_RESOURCE_PATH` (the server is started by `gz_run.sh` with the build's `gz_env.sh` sourced); names never collide between the homes, so order does not matter. The overlay entity is spawned by `gz_run.sh` itself through the world's `create` service from the model file, since PX4's `rc.gzsim` only resolves a model under its own models directory. |
| `airframes/` | copied into `build/px4_sitl_default/etc/init.d-posix/airframes/` by `install.sh` after every `make px4_sitl` (the ROMFS regeneration deletes the copies); rcS resolves `PX4_SIM_MODEL` and `SYS_AUTOSTART` by listing that directory, so no CMake entry is needed. There is no `make px4_sitl gz_<overlay member>` target: `gz_run.sh` builds with `make px4_sitl` and runs the binary directly with `PX4_SIM_MODEL=gz_<model> PX4_GZ_STANDALONE=1 PX4_GZ_MODEL_NAME=<model>_0 PX4_GZ_WORLD=lta_lab`, PX4's documented standalone path, attaching to the entity the script spawned. |
| `worlds/` | `lta_lab.sdf`, the `lta.sdf` of the tree the overlay runs against (`PX4_ROOT` at generation; a world includes that tree's own models) with the Buoyancy `<enable>` entries of every overlay model and its own `<world name>`, started standalone by `gz_run.sh` before PX4. |

## What is here, by number

The bare member of each family carries the `_rigid` token and the Rigid Hull
title; the other members keep the tree's names. The ones digit is the physics
mask (1 hull drag, 2 fins, 4 added mass). The tree's plain plant of a family
is the complete member under the plain name, a hundred below (2500 = 2605,
2510 = 2617, 2520 = 2627).

| decade | bare model (`_rigid`) | members | source |
|---|---|---|---|
| 2600 | `airship_fixed_rigid` | 2601 `_drag`, 2604 `_am`, 2605 `_am_drag` | the generic's fixed-thruster twin (tree 2500-2505), copied with the generic family |
| 2610 | `cloudship_rigid` | 2611 `_drag`, 2612 `_fin`, 2614 `_am`, 2616 `_am_fin`, 2617 `_am_fin_drag` | the tree's Cloudship (2510-2517), copied: `cloudship --overlay` |
| 2620 | `airship_vectored_independent_rigid` | 2621, 2622, 2624, 2626, 2627 | `vehicles/airship_generic.json` |
| 2630 | `airship_indoor_rigid` | 2631, 2632, 2634, 2636, 2637 | `vehicles/airship_indoor.json` |
| 2640 | `airship_aurora_rigid` | 2641, 2642, 2644, 2646, 2647 | `vehicles/airship_aurora.json` |
| 2650 | `airship_jaxa_rigid` | 2651, 2652, 2654, 2656, 2657 | `vehicles/airship_jaxa.json` |
| 2660 | `airship_mpi_rigid` | 2661, 2662, 2664, 2666, 2667 | `vehicles/airship_mpi.json` |

Each generated family also has its two bodies, `<name>_base_rigid` and
`<name>_base_am` (the generic's `airship_base_rigid` and `airship_base_am`,
which the fixed twin's members merge as well); the Cloudship is its own body.
The classic Cloudship (posix 2519) stays in the tree.

## Running a member

The interface is the tree's: `GZ_TARGET=gz_<model> ./gz_run.sh ./<check>.py`,
with `PX4_ROOT` naming the tree and `AIRSHIP_OVERLAY` the overlay (default
this directory; `gz_run.sh` exports the resolved value so the check reads the
same home). The launcher asks the overlay first: a target whose model has a
`model.sdf` under `overlay/models` takes the overlay path, anything else the
tree path, unchanged. A name in both homes is refused (exit 8), as
`gz_model.model_names()` refuses it: rcS would find two airframes for the one
`PX4_SIM_MODEL` and silently load the tree's. The overlay path, in order:

1. `make px4_sitl` (build only, bounded), then `overlay/install.sh <tree>`
   after it, since the build's ROMFS regeneration deletes the copies;
2. the build's `gz_env.sh` sourced, `overlay/models` prepended to
   `GZ_SIM_RESOURCE_PATH`, and `gz sim --verbose=1 -r -s overlay/worlds/lta_lab.sdf`
   started; readiness is rc.gzsim's own test (a provider on
   `/world/lta_lab/scene/info`, 30 s), and a `Failed to load system plugin`
   line in `sitl_out/gz.log` is fatal;
3. the model spawned as `<model>_0` at `PX4_GZ_MODEL_POSE` (default `0,0,20`)
   through `/world/lta_lab/create` from `file://overlay/models/<model>/model.sdf`,
   the request rc.gzsim builds; the create service accepts a request whose
   include does not resolve, so the name is then required on
   `/world/lta_lab/pose/info`;
4. `bin/px4` from `build/px4_sitl_default/rootfs` with the stdin pipe,
   `PX4_SIM_MODEL=gz_<model> PX4_GZ_STANDALONE=1 PX4_GZ_MODEL_NAME=<model>_0
   PX4_GZ_WORLD=lta_lab` (and `PX4_PARAM_SDLOG_DIRS_MAX=0` as on the tree path):
   rc.gzsim starts no world, waits on `lta_lab` and attaches `gz_bridge` to the
   entity; the uORB and "Gazebo world is ready" waits are the tree path's.

The check is given `PX4_GZ_ENTITY=<model>` and `PX4_GZ_WORLD=lta_lab`; the
world name is what tells an overlay run from a tree run in a log, and
`gz_model.WORLD` (default `lta`) is the one place the checks read it. The
"attached to a world this script did not start" refusal (exit 10) is the tree
path's: on the overlay path px4 runs standalone against the world the script
started and never looks for another.

The overlay airframes are the fork's, and a trimmed firmware may not carry
every parameter they set (`CA_AIRSHIP_GRP` on the bare member; `AS_SPD_*` and
`AS_YR_FF_SLP` on the complete one). rcS runs with `set +e`, so each such line
is a `Parameter X not found` warning in `sitl_out/server.log` and nothing
else; `gz_run.sh` lists them once before the check starts, so a verdict is
read against what the airframe actually applied.

The overlay can only be used against a trimmed tree: while the tree still
carries the families, rcS's name lookup would find two airframes for one
`PX4_SIM_MODEL` and load the tree's, and `gz_model.model_names()` refuses a
name present in both homes. The generator fills these directories in the same
increment as the trim (`docs/upstream_split_design_20261012.md`, sections 6
and 7); until then they are empty and `pins.env` names the tree. Before the
trim a member can still be generated into a scratch overlay
(`AIRSHIP_OVERLAY=<dir>`) from the fork's family (`--root <fork tree>`) with
the world from the trimmed tree (`PX4_ROOT=<trimmed tree>`), which is how step
(5) was verified.
