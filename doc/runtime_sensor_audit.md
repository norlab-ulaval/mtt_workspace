# Runtime Integration Notes

This document records the stable runtime boundaries for the MTT stack.

It is not a scratchpad. Keep short-lived investigations in local notes, not in
the versioned public docs.

## Ownership

`mtt_workspace` owns:
- the curated workspace layout,
- demo entry points,
- operator-facing commands,
- shared tooling and documentation.

`src/mtt_core` owns:
- driver logic,
- control logic,
- estimation and bringup nodes,
- the base robot description.

`src/external/norlab_robot` owns:
- robot-side launch assembly,
- sensor integration,
- mapping integration,
- Foxglove and runtime overlays.

If a sensor launch, TF attachment, or runtime startup path is inconsistent on
the real robot, the first place to check is usually `norlab_robot`.

## Sensor stack in use

The live runtime currently expects these sensor families:
- Xsens IMUs (`mti100`, `mti10`)
- Hesai LiDAR
- RoboSense RS-Airy LiDAR
- ZED2i
- OAK, when that branch is enabled and calibrated

The runtime layer also assembles:
- IMU-assisted odometry
- ICP mapping
- Foxglove bridge

Keep the declared sensor list, the actual launch files, and the preflight
checks aligned. If one file says a sensor is installed and another one does
not, treat that as a runtime integration bug, not as normal drift.

## TF and robot description rules

The live robot should expose a single coherent robot model through:
- `/robot_description`
- `/joint_states`
- `/tf`
- `/tf_static`

Runtime-specific camera or LiDAR publishers must not publish a second competing
robot description on the same topic.

Current expectations:
- `base_footprint -> base_link` is the base body chain
- `center_lidar_link` is the physical central LiDAR mount in the URDF
- compatibility frame aliases may be added when legacy runtime configs require
  a historical frame name, but the physical mount should stay explicit in the
  URDF

Do not invent extrinsics for IMUs or cameras just to satisfy a launch file.
If the physical pose is not known, keep the frame unattached or attach it only
through a clearly identified calibration artifact.

## Known runtime integration constraints

- The ZED tree may exist independently from the main robot body until the real
  body-to-camera extrinsic is calibrated and wired into the runtime.
- OAK should remain optional until its dependency path and body attachment are
  confirmed on the real robot.
- Foxglove should use one documented default configuration. Keep alternate
  bridge configs only if their role is explicit.

## Maintenance rule

When you change the robot runtime:
1. update the launch path,
2. update the matching config,
3. update the operator-facing note if the behavior changed.

Do not leave one source of truth in launch, another one in a helper script, and
a third one in an outdated note.
