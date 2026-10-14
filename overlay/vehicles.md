# Overlay vehicles

The families this repository carries beside the PX4 tree (`docs/upstream_split_design_20261012.md`):
every physics-variant member of each vehicle, under the overlay numbers. A member is flown with
`GZ_TARGET=gz_<model> ./gz_run.sh <check>`; there is no make target.

| Vehicle | Model | SYS_AUTOSTART |
|---|---|---|
| [Generic Airship (Independent Vectoring, Rigid Hull)](#generic-airship-independent-vectoring-rigid-hull) | `gz_airship_vectored_independent_rigid` | 2620 |
| [Generic Airship (Independent Vectoring, Hull Drag)](#generic-airship-independent-vectoring-hull-drag) | `gz_airship_vectored_independent_drag` | 2621 |
| [Generic Airship (Independent Vectoring, Fins)](#generic-airship-independent-vectoring-fins) | `gz_airship_vectored_independent_fin` | 2622 |
| [Generic Airship (Independent Vectoring, Added Mass)](#generic-airship-independent-vectoring-added-mass) | `gz_airship_vectored_independent_am` | 2624 |
| [Generic Airship (Independent Vectoring, Added Mass, Fins)](#generic-airship-independent-vectoring-added-mass-fins) | `gz_airship_vectored_independent_am_fin` | 2626 |
| [Generic Airship (Independent Vectoring, Complete)](#generic-airship-independent-vectoring-complete) | `gz_airship_vectored_independent_am_fin_drag` | 2627 |
| [Generic Airship (Rigid Hull)](#generic-airship-rigid-hull) | `gz_airship_fixed_rigid` | 2600 |
| [Generic Airship (Hull Drag)](#generic-airship-hull-drag) | `gz_airship_fixed_drag` | 2601 |
| [Generic Airship (Added Mass)](#generic-airship-added-mass) | `gz_airship_fixed_am` | 2604 |
| [Generic Airship (Complete)](#generic-airship-complete) | `gz_airship_fixed_am_drag` | 2605 |
| [Indoor Airship (Rigid Hull)](#indoor-airship-rigid-hull) | `gz_airship_indoor_rigid` | 2630 |
| [Indoor Airship (Hull Drag)](#indoor-airship-hull-drag) | `gz_airship_indoor_drag` | 2631 |
| [Indoor Airship (Fins)](#indoor-airship-fins) | `gz_airship_indoor_fin` | 2632 |
| [Indoor Airship (Added Mass)](#indoor-airship-added-mass) | `gz_airship_indoor_am` | 2634 |
| [Indoor Airship (Added Mass, Fins)](#indoor-airship-added-mass-fins) | `gz_airship_indoor_am_fin` | 2636 |
| [Indoor Airship (Complete)](#indoor-airship-complete) | `gz_airship_indoor_am_fin_drag` | 2637 |
| [AURORA Airship (AS800 class, Rigid Hull)](#aurora-airship-as800-class-rigid-hull) | `gz_airship_aurora_rigid` | 2640 |
| [AURORA Airship (AS800 class, Hull Drag)](#aurora-airship-as800-class-hull-drag) | `gz_airship_aurora_drag` | 2641 |
| [AURORA Airship (AS800 class, Fins)](#aurora-airship-as800-class-fins) | `gz_airship_aurora_fin` | 2642 |
| [AURORA Airship (AS800 class, Added Mass)](#aurora-airship-as800-class-added-mass) | `gz_airship_aurora_am` | 2644 |
| [AURORA Airship (AS800 class, Added Mass, Fins)](#aurora-airship-as800-class-added-mass-fins) | `gz_airship_aurora_am_fin` | 2646 |
| [AURORA Airship (AS800 class, Complete)](#aurora-airship-as800-class-complete) | `gz_airship_aurora_am_fin_drag` | 2647 |
| [JAXA 14 m LTA Test Vehicle (Rigid Hull)](#jaxa-14-m-lta-test-vehicle-rigid-hull) | `gz_airship_jaxa_rigid` | 2650 |
| [JAXA 14 m LTA Test Vehicle (Hull Drag)](#jaxa-14-m-lta-test-vehicle-hull-drag) | `gz_airship_jaxa_drag` | 2651 |
| [JAXA 14 m LTA Test Vehicle (Fins)](#jaxa-14-m-lta-test-vehicle-fins) | `gz_airship_jaxa_fin` | 2652 |
| [JAXA 14 m LTA Test Vehicle (Added Mass)](#jaxa-14-m-lta-test-vehicle-added-mass) | `gz_airship_jaxa_am` | 2654 |
| [JAXA 14 m LTA Test Vehicle (Added Mass, Fins)](#jaxa-14-m-lta-test-vehicle-added-mass-fins) | `gz_airship_jaxa_am_fin` | 2656 |
| [JAXA 14 m LTA Test Vehicle (Complete)](#jaxa-14-m-lta-test-vehicle-complete) | `gz_airship_jaxa_am_fin_drag` | 2657 |
| [MPI 5 m Outdoor Blimp (Rigid Hull)](#mpi-5-m-outdoor-blimp-rigid-hull) | `gz_airship_mpi_rigid` | 2660 |
| [MPI 5 m Outdoor Blimp (Hull Drag)](#mpi-5-m-outdoor-blimp-hull-drag) | `gz_airship_mpi_drag` | 2661 |
| [MPI 5 m Outdoor Blimp (Fins)](#mpi-5-m-outdoor-blimp-fins) | `gz_airship_mpi_fin` | 2662 |
| [MPI 5 m Outdoor Blimp (Added Mass)](#mpi-5-m-outdoor-blimp-added-mass) | `gz_airship_mpi_am` | 2664 |
| [MPI 5 m Outdoor Blimp (Added Mass, Fins)](#mpi-5-m-outdoor-blimp-added-mass-fins) | `gz_airship_mpi_am_fin` | 2666 |
| [MPI 5 m Outdoor Blimp (Complete)](#mpi-5-m-outdoor-blimp-complete) | `gz_airship_mpi_am_fin_drag` | 2667 |
| [Cloudship (Rigid Hull)](#cloudship-rigid-hull) | `gz_cloudship_rigid` | 2610 |
| [Cloudship (Hull Drag)](#cloudship-hull-drag) | `gz_cloudship_drag` | 2611 |
| [Cloudship (Fins)](#cloudship-fins) | `gz_cloudship_fin` | 2612 |
| [Cloudship (Added Mass)](#cloudship-added-mass) | `gz_cloudship_am` | 2614 |
| [Cloudship (Added Mass, Fins)](#cloudship-added-mass-fins) | `gz_cloudship_am_fin` | 2616 |
| [Cloudship (Complete)](#cloudship-complete) | `gz_cloudship_am_fin_drag` | 2617 |

### Generic Airship (Independent Vectoring, Rigid Hull)

[Generic Airship (Independent Vectoring)](https://docs.px4.io/main/en/frames_airship/): two independently tilting thrust pods on a 7.00 x 2.00 m hull, the rigid hull: a torque keeps accelerating it and nothing damps a rotation.

```sh
GZ_TARGET=gz_airship_vectored_independent_rigid ./gz_run.sh <check>
```

### Generic Airship (Independent Vectoring, Hull Drag)

The same vehicle with hull drag and rotational damping.

```sh
GZ_TARGET=gz_airship_vectored_independent_drag ./gz_run.sh <check>
```

### Generic Airship (Independent Vectoring, Fins)

The same vehicle with its four fins as lifting surfaces, isolated from the other physics so that their contribution can be measured on its own.
The plant they exist for is the Added Mass, Fins variant, where they have the Munk moment to oppose.

```sh
GZ_TARGET=gz_airship_vectored_independent_fin ./gz_run.sh <check>
```

### Generic Airship (Independent Vectoring, Added Mass)

The same vehicle with the hull's fluid added mass and nothing to oppose the Munk moment it brings.
It exists to measure that term on its own.

```sh
GZ_TARGET=gz_airship_vectored_independent_am ./gz_run.sh <check>
```

### Generic Airship (Independent Vectoring, Added Mass, Fins)

Added mass and fin lift together, the Munk moment and the one thing on this hull that opposes it, with no drag.
With no hull drag, the fins are also all that damps a rate here.

```sh
GZ_TARGET=gz_airship_vectored_independent_am_fin ./gz_run.sh <check>
```

### Generic Airship (Independent Vectoring, Complete)

Everything: added mass, fin lift, hull drag and rotational damping.
This is the plant to tune the loops against.

```sh
GZ_TARGET=gz_airship_vectored_independent_am_fin_drag ./gz_run.sh <check>
```

### Generic Airship (Rigid Hull)

[Generic Airship](https://docs.px4.io/main/en/frames_airship/) as its flight airframe declares it: two fixed forward thrusters and four fin flaps, two elevators and two rudders, on a rigid hull that meets no air.
The flaps are its only attitude authority, and like real ones they give nothing until the hull moves, so fly it forward first.
Nothing on the rigid hull limits that speed; the members with hull drag (Hull Drag, Complete) settle at one.
On the rigid hull the fins are the only yaw moment, so in a steady turn they carry no net side force: a held rudder sets a sideslip rather than a turn, and hull drag adds a weak one.
The members with added mass (Added Mass, Complete) turn because there the fins must also hold the hull's Munk moment.
A held elevator likewise makes heave on the rigid hull, the pitch settling back to level; with added mass the Munk moment tilts the hull until the buoyancy pendulum balances it, so the elevator holds an attitude.

```sh
GZ_TARGET=gz_airship_fixed_rigid ./gz_run.sh <check>
```

### Generic Airship (Hull Drag)

The same vehicle with hull drag and rotational damping.

```sh
GZ_TARGET=gz_airship_fixed_drag ./gz_run.sh <check>
```

### Generic Airship (Added Mass)

The same vehicle with the hull's fluid added mass, which brings the Munk moment the tail was sized against.
No drag.

```sh
GZ_TARGET=gz_airship_fixed_am ./gz_run.sh <check>
```

### Generic Airship (Complete)

Everything the hull has: added mass, hull drag and rotational damping.
This is the plant to fly it on.

```sh
GZ_TARGET=gz_airship_fixed_am_drag ./gz_run.sh <check>
```

### Indoor Airship (Rigid Hull)

[Indoor Airship](https://docs.px4.io/main/en/frames_airship/): two independently tilting thrust pods on a 2.00 x 0.80 m hull, the rigid hull: a torque keeps accelerating it and nothing damps a rotation.

```sh
GZ_TARGET=gz_airship_indoor_rigid ./gz_run.sh <check>
```

### Indoor Airship (Hull Drag)

The same vehicle with hull drag and rotational damping.

```sh
GZ_TARGET=gz_airship_indoor_drag ./gz_run.sh <check>
```

### Indoor Airship (Fins)

The same vehicle with its four fins as lifting surfaces, isolated from the other physics so that their contribution can be measured on its own.
The plant they exist for is the Added Mass, Fins variant, where they have the Munk moment to oppose.

```sh
GZ_TARGET=gz_airship_indoor_fin ./gz_run.sh <check>
```

### Indoor Airship (Added Mass)

The same vehicle with the hull's fluid added mass and nothing to oppose the Munk moment it brings.
It exists to measure that term on its own.

```sh
GZ_TARGET=gz_airship_indoor_am ./gz_run.sh <check>
```

### Indoor Airship (Added Mass, Fins)

Added mass and fin lift together, the Munk moment and the one thing on this hull that opposes it, with no drag.
With no hull drag, the fins are also all that damps a rate here.

```sh
GZ_TARGET=gz_airship_indoor_am_fin ./gz_run.sh <check>
```

### Indoor Airship (Complete)

Everything: added mass, fin lift, hull drag and rotational damping.
This is the plant to tune the loops against.

```sh
GZ_TARGET=gz_airship_indoor_am_fin_drag ./gz_run.sh <check>
```

### AURORA Airship (AS800 class, Rigid Hull)

[AURORA Airship (AS800 class)](https://docs.px4.io/main/en/frames_airship/): two independently tilting thrust pods on a 9.27 x 2.65 m hull, the rigid hull: a torque keeps accelerating it and nothing damps a rotation.
Its envelope is that of the AURORA robotic airship, an Airspeed Airships AS800 (de Paiva et al., [Project AURORA](https://doi.org/10.1002/rob.20111), Journal of Field Robotics 23, 2006: 10.5 m x 3.0 m, 34 m³), drawn as the spheroid of the same volume and fineness; the pods, fins and masses follow this family's rules, not that vehicle.

```sh
GZ_TARGET=gz_airship_aurora_rigid ./gz_run.sh <check>
```

### AURORA Airship (AS800 class, Hull Drag)

The same vehicle with hull drag and rotational damping.

```sh
GZ_TARGET=gz_airship_aurora_drag ./gz_run.sh <check>
```

### AURORA Airship (AS800 class, Fins)

The same vehicle with its four fins as lifting surfaces, isolated from the other physics so that their contribution can be measured on its own.
The plant they exist for is the Added Mass, Fins variant, where they have the Munk moment to oppose.

```sh
GZ_TARGET=gz_airship_aurora_fin ./gz_run.sh <check>
```

### AURORA Airship (AS800 class, Added Mass)

The same vehicle with the hull's fluid added mass and nothing to oppose the Munk moment it brings.
It exists to measure that term on its own.

```sh
GZ_TARGET=gz_airship_aurora_am ./gz_run.sh <check>
```

### AURORA Airship (AS800 class, Added Mass, Fins)

Added mass and fin lift together, the Munk moment and the one thing on this hull that opposes it, with no drag.
With no hull drag, the fins are also all that damps a rate here.

```sh
GZ_TARGET=gz_airship_aurora_am_fin ./gz_run.sh <check>
```

### AURORA Airship (AS800 class, Complete)

Everything: added mass, fin lift, hull drag and rotational damping.
This is the plant to tune the loops against.

```sh
GZ_TARGET=gz_airship_aurora_am_fin_drag ./gz_run.sh <check>
```

### JAXA 14 m LTA Test Vehicle (Rigid Hull)

[JAXA 14 m LTA Test Vehicle](https://docs.px4.io/main/en/frames_airship/): two independently tilting thrust pods on a 14.00 x 4.00 m hull, the rigid hull: a torque keeps accelerating it and nothing damps a rotation.
Its hull is that of the 14 m LTA flying test bed of the Japan Aerospace Exploration Agency (Kohno, Nakadate, Okuyama, [AIAA 2009-2860](https://doi.org/10.2514/6.2009-2860): 14 m, 4.0 m maximum diameter, 117 m³, two tilting ducted propellers, rudders and elevators on a cruciform tail); the pods, fins and masses follow this family's rules, not that vehicle.

```sh
GZ_TARGET=gz_airship_jaxa_rigid ./gz_run.sh <check>
```

### JAXA 14 m LTA Test Vehicle (Hull Drag)

The same vehicle with hull drag and rotational damping.

```sh
GZ_TARGET=gz_airship_jaxa_drag ./gz_run.sh <check>
```

### JAXA 14 m LTA Test Vehicle (Fins)

The same vehicle with its four fins as lifting surfaces, isolated from the other physics so that their contribution can be measured on its own.
The plant they exist for is the Added Mass, Fins variant, where they have the Munk moment to oppose.

```sh
GZ_TARGET=gz_airship_jaxa_fin ./gz_run.sh <check>
```

### JAXA 14 m LTA Test Vehicle (Added Mass)

The same vehicle with the hull's fluid added mass and nothing to oppose the Munk moment it brings.
It exists to measure that term on its own.

```sh
GZ_TARGET=gz_airship_jaxa_am ./gz_run.sh <check>
```

### JAXA 14 m LTA Test Vehicle (Added Mass, Fins)

Added mass and fin lift together, the Munk moment and the one thing on this hull that opposes it, with no drag.
With no hull drag, the fins are also all that damps a rate here.

```sh
GZ_TARGET=gz_airship_jaxa_am_fin ./gz_run.sh <check>
```

### JAXA 14 m LTA Test Vehicle (Complete)

Everything: added mass, fin lift, hull drag and rotational damping.
This is the plant to tune the loops against.

```sh
GZ_TARGET=gz_airship_jaxa_am_fin_drag ./gz_run.sh <check>
```

### MPI 5 m Outdoor Blimp (Rigid Hull)

[MPI 5 m Outdoor Blimp](https://docs.px4.io/main/en/frames_airship/): two independently tilting thrust pods on a 5.00 x 2.00 m hull, the rigid hull: a torque keeps accelerating it and nothing damps a rotation.
Its envelope is that of the 5 m outdoor blimp of Price et al. ([Simulation and Control of Deformable Autonomous Airships in Turbulent Wind](https://doi.org/10.1007/978-3-030-95892-3_46), 2022) as its open simulation model gives it (5.0 m x 2.0 m, 10.47 m³); the pods, fins and masses follow this family's rules, not that vehicle.

```sh
GZ_TARGET=gz_airship_mpi_rigid ./gz_run.sh <check>
```

### MPI 5 m Outdoor Blimp (Hull Drag)

The same vehicle with hull drag and rotational damping.

```sh
GZ_TARGET=gz_airship_mpi_drag ./gz_run.sh <check>
```

### MPI 5 m Outdoor Blimp (Fins)

The same vehicle with its four fins as lifting surfaces, isolated from the other physics so that their contribution can be measured on its own.
The plant they exist for is the Added Mass, Fins variant, where they have the Munk moment to oppose.

```sh
GZ_TARGET=gz_airship_mpi_fin ./gz_run.sh <check>
```

### MPI 5 m Outdoor Blimp (Added Mass)

The same vehicle with the hull's fluid added mass and nothing to oppose the Munk moment it brings.
It exists to measure that term on its own.

```sh
GZ_TARGET=gz_airship_mpi_am ./gz_run.sh <check>
```

### MPI 5 m Outdoor Blimp (Added Mass, Fins)

Added mass and fin lift together, the Munk moment and the one thing on this hull that opposes it, with no drag.
With no hull drag, the fins are also all that damps a rate here.

```sh
GZ_TARGET=gz_airship_mpi_am_fin ./gz_run.sh <check>
```

### MPI 5 m Outdoor Blimp (Complete)

Everything: added mass, fin lift, hull drag and rotational damping.
This is the plant to tune the loops against.

```sh
GZ_TARGET=gz_airship_mpi_am_fin_drag ./gz_run.sh <check>
```

### Cloudship (Rigid Hull)

[Cloudship](https://docs.px4.io/main/en/frames_airship/): collective thrust on one tilt, with a reversible tail thruster for yaw.
The rigid hull.

```sh
GZ_TARGET=gz_cloudship_rigid ./gz_run.sh <check>
```

### Cloudship (Hull Drag)

The same vehicle with hull drag and rotational damping.

```sh
GZ_TARGET=gz_cloudship_drag ./gz_run.sh <check>
```

### Cloudship (Fins)

The same vehicle with its four fins as lifting surfaces, isolated from the other physics so that their contribution can be measured on its own.
The plant they exist for is the Added Mass, Fins variant, where they have the Munk moment to oppose.

```sh
GZ_TARGET=gz_cloudship_fin ./gz_run.sh <check>
```

### Cloudship (Added Mass)

The same vehicle with the hull's fluid added mass.
The physics engine folds added mass into the spatial inertia, Coriolis terms included, so this plant carries the Munk moment: a hull slipping sideways is turned further sideways, and nothing in this model opposes it.
It exists to measure that term on its own.

```sh
GZ_TARGET=gz_cloudship_am ./gz_run.sh <check>
```

### Cloudship (Added Mass, Fins)

Added mass and fin lift together, the Munk moment and the one thing on this hull that opposes it, with no drag.
With no hull drag, the fins are also all that damps a rate here.

```sh
GZ_TARGET=gz_cloudship_am_fin ./gz_run.sh <check>
```

### Cloudship (Complete)

Everything: added mass, fin lift, hull drag and rotational damping.
This is the plant to tune the yaw loop against.

```sh
GZ_TARGET=gz_cloudship_am_fin_drag ./gz_run.sh <check>
```
