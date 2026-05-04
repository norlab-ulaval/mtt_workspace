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

## TF state observed on the live robot

[OBSERVÉ] The live robot runtime publishes the core model topics:
- `/tf`
- `/tf_static`
- `/robot_description`
- `/joint_states`

[OBSERVÉ] The MTT body tree is present:
- `base_footprint -> base_link`
- `base_link -> center_lidar_link`

[OBSERVÉ] `center_lidar_link` exists in the URDF and in the live TF tree.

[OBSERVÉ] The Hesai driver publishes data under `/hesai_lidar/*`, but the
runtime expected a frame named `hesai_lidar` that was not present in the TF
tree.

[RECOMMANDATION] Keep `center_lidar_link` as the physical mount in the URDF and
publish a fixed compatibility frame `hesai_lidar` on top of it. This is a safe
alias because the live stack, mapping config, and bagging config already use the
`hesai_lidar` name.

[OBSERVÉ] `imu_link` and `mti10_imu_link_right` are used as IMU frame ids in the
runtime configs, but no matching links were found in the current MTT URDF.

[À VÉRIFIER] Their physical poses are still missing from the audited workspace.

[RECOMMANDATION] Do not invent IMU extrinsics in the URDF. Confirm the actual
mounting poses on the robot first, then add those links in `mtt_description`.

[OBSERVÉ] `zed_state_publisher` publishes a separate ZED TF tree, but
`base_link -> zed_camera_link` is currently not connected on the live robot.

[DÉDUIT] The ZED wrapper is publishing its own camera-local URDF without a
robot-side parent transform.

[RECOMMANDATION] Treat the ZED extrinsic as a missing runtime calibration item.
Do not hard-code a fake static transform until the camera mount pose is measured
or sourced from a trusted calibration file.

[OBSERVÉ] The live robot had two publishers on `/robot_description`:
- `robot_state_publisher`
- `oak_state_publisher`

[DÉDUIT] The previous OAK launch path was publishing a second standalone camera
description on the same topic, which can confuse RViz and Foxglove.

[RECOMMANDATION] Launch OAK with the upstream
`camera_as_part_of_a_robot.launch.py` path until the real OAK parent frame and
extrinsic pose are calibrated. That keeps the camera topics without pretending
the OAK already has a validated robot-side TF attachment.

[OBSERVÉ] A measured extrinsic was later provided for:
- `rsairy -> oak_rgb_camera_optical_frame`

[RECOMMANDATION] Use that calibration in the OAK launch path so the OAK and the
RS Bpearl share one TF subtree. This still does not anchor them to the main
robot body until `rsairy` itself is tied into the MTT URDF or by a trusted
static transform.

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
