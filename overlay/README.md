# Overlay: flying what upstream does not carry

Upstream PX4 is meant to carry one complete plant per airship family and the
`lta` world. The physics-variant members (`_am`, `_fin`, `_drag` and their
combinations) and the literature vehicles exist to tell the model's share of
a result from the loop's share; they are validation apparatus and live here.

The overlay is three directories the generator fills (`airship_family_gen.py
--overlay`, planned with the upstream trim) and nothing in the PX4 tree
changes:

| directory | reaches PX4 through |
|---|---|
| `models/` | both `overlay/models` and the tree's models are on the gz server's `GZ_SIM_RESOURCE_PATH` (the server is started by `gz_run.sh` with the build's `gz_env.sh` sourced); names never collide between the homes, so order does not matter. The overlay entity is spawned by `gz_run.sh` itself through the world's `create` service from the model file, since PX4's `rc.gzsim` only resolves a model under its own models directory. |
| `airframes/` | copied into `build/px4_sitl_default/etc/init.d-posix/airframes/` by `install.sh` after every `make px4_sitl` (the ROMFS regeneration deletes the copies); rcS resolves `PX4_SIM_MODEL` and `SYS_AUTOSTART` by listing that directory, so no CMake entry is needed. There is no `make px4_sitl gz_<overlay member>` target: `gz_run.sh` builds with `make px4_sitl` and runs the binary directly with `PX4_SIM_MODEL=gz_<model> PX4_GZ_STANDALONE=1 PX4_GZ_MODEL_NAME=<model>_0 PX4_GZ_WORLD=lta_lab`, PX4's documented standalone path, attaching to the entity the script spawned. |
| `worlds/` | `lta_lab.sdf`, the tree's `lta.sdf` with the Buoyancy `<enable>` entries of every overlay model and its own `<world name>`, started standalone by `gz_run.sh` before PX4. |

The overlay can only be used against a trimmed tree: while the tree still
carries the families, rcS's name lookup would find two airframes for one
`PX4_SIM_MODEL` and load the tree's, and `gz_model.model_names()` refuses a
name present in both homes. The generator fills these directories in the same
increment as the trim (`docs/upstream_split_design_20261012.md`, sections 6
and 7); until then they are empty and `pins.env` names the tree.
