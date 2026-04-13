# Runtime sensor audit

This note tracks what belongs to the runtime integration layer around MTT.

It is based on the robot snapshot under `Workspace/tmp/robot_src/norlab_robot`.
It is not a claim that `mtt_workspace` or `src/mtt_core` alone own that logic.

## Scope

[OBSERVÉ] `norlab_robot` is the layer that currently assembles:
- sensors,
- mapping,
- Foxglove,
- startup scripts,
- robot-side runtime utilities.

[RECOMMANDATION] Fix runtime sensor inconsistencies in `norlab_robot`, not in `mtt_workspace`.

## Sensor launch truth

[OBSERVÉ] `launch/sensors.launch.py` currently includes:
- `mti100.launch.py`
- `mti10.launch.py`
- `hesai32.launch.py`
- `rsairy.launch.py`
- `zed2i.launch.py`
- `oak.launch.py`

[OBSERVÉ] `scripts/config/sensors_interfaces.yaml` currently declares only:
- `lslidar16`
- `mti100`
- `zed_camera`

[DÉDUIT] The sensor ping/preflight file is stale relative to the actual launch list.

[RECOMMANDATION] The source of truth should be made consistent in `norlab_robot`:
- either update `sensors_interfaces.yaml` to match the launch list,
- or reduce `sensors.launch.py` to what is really installed,
- or move to a manifest-driven sensor list and keep one file authoritative.

## What looks confirmed

[OBSERVÉ] Xsens support is real:
- `launch/include/mti100.launch.py`
- `launch/include/mti10.launch.py`
- `config/_mti100.yaml`
- `config/_mti10.yaml`

[OBSERVÉ] The Xsens path uses:
- `xsens_driver`
- `norlab_imu_tools`
- `imu_filter_madgwick`

[OBSERVÉ] Hesai support is real:
- `launch/include/hesai32.launch.py`
- `config/_hesai32_ns.yaml`

[OBSERVÉ] RoboSense support is real:
- `launch/include/rsairy.launch.py`
- `config/_rsairy.yaml`

[OBSERVÉ] ZED2i support is real:
- `launch/include/zed2i.launch.py`
- `config/_zed2i.yaml`
- `zed-ros2-wrapper` is already in the curated manifest

[OBSERVÉ] IMU odometry and ICP mapping are wired in the runtime layer:
- `launch/include/imu_odom.launch.py`
- `launch/include/imu_and_wheel_odom.launch.py`
- `launch/include/icp_mapper.launch.py`

## What stays uncertain

[OBSERVÉ] OAK is referenced by:
- `launch/include/oak.launch.py`
- `config/_oak.yaml`

[OBSERVÉ] The OAK launch expects `depthai_ros_driver`.

[À VÉRIFIER] The exact upstream or fork used on the robot is still not confirmed from the audited workspace alone.

[RECOMMANDATION] Keep OAK optional in `dependencies/robot.repos` until the real robot confirms the dependency and the package name.

## Foxglove

[OBSERVÉ] `launch/include/foxglove_bridge.launch.py` loads:
- `config/_foxglove_bridge.yaml`

[OBSERVÉ] A second config also exists:
- `config/_foxglove_bridge_whitelist.yaml`

[DÉDUIT] The launch path currently uses the open config, not the whitelist config.

[RECOMMANDATION] Decide this in `norlab_robot` and keep one documented default:
- open bridge if the lab network and operator workflow really require it,
- whitelist config if the goal is tighter exposure and a stable operator surface.

## What should change where

[RECOMMANDATION] `mtt_workspace`
- keep documenting the boundary,
- keep the curated dependency manifest honest,
- do not absorb runtime sensor launch logic.

[RECOMMANDATION] `norlab_robot`
- reconcile `sensors.launch.py` and `sensors_interfaces.yaml`,
- confirm OAK dependency and namespace strategy,
- choose the real Foxglove config,
- align startup scripts with the runtime that is actually used on the robot.
