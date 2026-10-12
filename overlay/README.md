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
| `models/` | `GZ_SIM_RESOURCE_PATH`, exported before the launch (`install.sh` prints the line); PX4's `gz_env.sh` appends its own model directory after it |
| `airframes/` | copied into `build/px4_sitl_default/etc/init.d-posix/airframes/` by `install.sh`; rcS finds airframes by listing that directory, so no CMake entry is needed |
| `worlds/` | a world with the Buoyancy `<enable>` entries of every overlay model. PX4's `rc.gzsim` launches a world only by name under its own worlds directory, so an overlay world is started standalone (`gz sim -r -s overlay/worlds/<world>.sdf`) and PX4 attaches to it, which is PX4's documented path for an already-running server. `gz_run.sh` will do this when the overlay carries a world. |

Until the upstream trim lands, every member is still in the fork's tree and
these directories are empty; `pins.env` names the tree.
