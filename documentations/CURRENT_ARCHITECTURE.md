# Current MTT core architecture

This note is intentionally scoped to the MTT-specific code in this repository.

It does not describe the full robot runtime by itself.
The complete robot stack currently also depends on:
- `norlab_robot` as the runtime integration layer,
- external drivers and estimation packages imported through `dependencies/robot.repos`,
- robot-side startup and sensor configuration that are not owned by the MTT core.

For the workspace structure, see [../doc/workspace_architecture.md](../doc/workspace_architecture.md).

## What lives in this repository

- `mtt_driver`
  ROS wrapper and Python CAN driver for the MTT platform.
- `mtt_msgs`
  MTT-specific messages.
- `mtt_interfaces`
  MTT-specific services.
- `mtt_description`
  Base MTT description and simulation assets owned by this repository.
- `mtt_bringup`
  MTT-side launch files and local bringup helpers.

## Local command path

```text
joy / controller / manual command
        ↓
   relative ROS topics
        ↓
   mtt_ros_wrapper
        ↓
     mtt_driver
        ↓
       CAN bus
        ↓
      MTT vehicle
```

Telemetry follows the reverse path back into the MTT-specific ROS topics.

## What this file should not be used for

Do not use this file as proof of:
- the final CAN truth on the robot,
- final safety semantics,
- the full startup sequence,
- the full sensor or mapping stack,
- the exact runtime deployed on the high-level computer.

Those points depend on the external integration layer and still require robot-side verification.

## Known open points

- CAN ID ownership and safety semantics still need live confirmation on the robot.
- Steering truth is still weaker than command truth in the current audited stack.
- OAK support is still incomplete at the workspace level.
- The namespace story is only partially cleaned up in the older launch files.
- TF is not fully closed yet across all sensors:
  - the live model tree is present,
  - `hesai_lidar` needed a compatibility alias over `center_lidar_link`,
  - IMU frame links are still missing from the audited URDF,
  - ZED currently publishes a disconnected camera-local tree on the live robot.
