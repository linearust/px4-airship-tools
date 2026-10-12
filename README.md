# px4-airship-tools

Tools, simulator checks and design records for PX4 airship support that do
not belong in the PX4 tree: a generator that turns one vehicle description
into a Gazebo model family with its airframes, a calculator for the yaw loop's
values, SITL gates, a validated model of the yaw rate loop, bench scripts for
the flight controller at the pins, and the notes behind the decisions.

The firmware is the `airship/dev` branch of
[icarus-lta/PX4-Autopilot](https://github.com/icarus-lta/PX4-Autopilot) and
the models are `airship/models` of
[icarus-lta/PX4-gazebo-models](https://github.com/icarus-lta/PX4-gazebo-models).
`pins.env` names the commits this repository is verified against. What
hardware validates goes upstream; this repository keeps what upstream would
not take: the physics-variant families, the literature vehicles, the
generators, the gates and the records.

## Layout

| | |
|---|---|
| `airship_family_gen.py`, `apply_generic_hull.py`, `generic_hull_design.py`, `hull_basis.py`, `fin_basis.py` | one vehicle JSON in, a gz model family, world entries, posix airframes and docs tables out |
| `airship_gains.py` | the yaw loop's values (P, I, feedforward, speed schedule) from the family's model files |
| `vehicles/` | vehicle descriptions, their measured held-turn slopes, the FMU parameter set |
| `sitl_check.py`, `forward_yaw_check.py`, `airship_fixed_flight_check.py`, `cloudship_*.py`, `tilt_range_check.py`, `reverse_thrust_check.py` | SITL gates, run through `gz_run.sh` |
| `variant_check.py`, `added_mass_check.py`, `aero_moment_check.py` | is the model family what its names say, in the files and in the engine |
| `gz_model.py`, `gz_link.py`, `sitl_link.py`, `report.py` | the four modules the checks are built on: SDF and airframe reading, gz topics, the px4 client and MAVLink, the verdict printer |
| `yaw_model/` | the 1-DOF and planar models of the yaw loop, validated against the gate's flight logs, with the replays that settled the design |
| `bench_*.py`, `fmu_*.py`, `build_fmu.py`, `la_mcp.py`, `fp_tools/`, `identity/` | the bench: firmware staging with a metadata check, MAVLink link to the FMU, pin fingerprints through the logic analyzer, allocator identity harness |
| `overlay/` | how the families upstream does not carry reach a local PX4 without touching its tree |
| `docs/` | `log.md`, the running log in English; the dated design notes, mostly Korean |

## Setup

- A PX4 tree at `PX4_ROOT` (default `~/dev/PX4-Autopilot`) on the fork
  branch, with `Tools/simulation/gz` on the models branch and
  `make px4_sitl_default` built once.
- gz sim 8 (Harmonic) for the SITL checks.
- Python 3.12 with `numpy`, `pyyaml`, `pyulog`, `pymavlink`.

## Running

```sh
python3 airship_family_gen.py vehicles/airship_indoor.json          # write the family into the PX4 tree
python3 airship_family_gen.py vehicles/airship_generic.json --check # regenerate into a scratch copy, diff
python3 airship_gains.py airship_indoor --write --params vehicles/airship_indoor.params
python3 variant_check.py                                            # the scheme, in the files; no gz

GZ_TARGET=gz_airship_indoor_am_fin_drag ./gz_run.sh ./sitl_check.py          # hover yaw gate
GZ_TARGET=gz_airship_indoor_am_fin_drag ./gz_run.sh ./forward_yaw_check.py   # forward-flight legs
python3 added_mass_check.py --vehicle airship_indoor                          # gz only, no px4

python3 build_fmu.py                                                # build, check, stage fw_<commit>/
./run_tests.sh                                                      # what CI runs
```

`gz_run.sh` boots a headless gz SITL for `GZ_TARGET`, runs one script against
it and cleans up. Never start px4 while another gz world is up: it attaches to
it. Each script's docstring is its manual.

## Tests and CI

`run_tests.sh` runs the checks that need only the PX4 source tree: the
generator round trip, the variant scheme, the forward-flight analysis and the
yaw model's controller reading. CI checks out the fork at the commits in
`pins.env` and runs it. The SITL gates need gz and stay local; their verdicts
are recorded in `docs/`.

Four yaw model tests (`test_forward`, `test_forward_replay`,
`test_policies`, `test_fin_damping`) replay records flown on the 10.97 m hull
the tree carried until 2026-10-07 while reading the plant from the tree, and
are not in `run_tests.sh` until the model is re-anchored; `test_controller`
prints a visible `SKIP` for the one replay that needs that plant.

## Conventions

Conventional commits, `type(scope): description`. AI assistance is disclosed
by an `Assisted-by:` trailer and nothing else. Outputs (`sitl_out/`,
`fw_*/`, `head_*/`, logs) are not committed; see `.gitignore`.

BSD-3-Clause, like PX4.
