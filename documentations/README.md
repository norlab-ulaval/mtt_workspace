# MTT technical documentation

This folder is the single source of technical documentation for the workspace.
It replaces the previous split between doc/ and documentations/.

If you are new to the project, read this file once end to end.
It is meant to be the operational reference for an engineer who must modify the
stack without guessing.

## Workspace layout

```text
mtt_workspace/
  src/
    mtt_core/
    external/
      norlab_robot/
  dependencies/
    robot.repos
  docker/
  demos/
  documentations/
  scripts/
  data/
  build/            (generated)
  install/          (generated)
  log/              (generated)
```

## Ownership and responsibilities

- mtt_workspace (this repo)
  - owns Docker, Compose, scripts, demos, docs, manifests
  - does not own the robot runtime code itself

- src/mtt_core
  - MTT owned packages (driver, messages, interfaces, description, bringup)

- src/external
  - imported dependencies and runtime overlays

- src/external/norlab_robot
  - runtime integration layer (sensor composition, launch assembly, mapping,
    runtime overlays)

If a runtime path is wrong, fix it in norlab_robot first.
If a driver or message is wrong, fix it in mtt_core.

## Bootstrap and repo import

The workspace is a shell around nested repositories.
Use the scripts instead of manual git calls.

```bash
./scripts/create_env
./scripts/create_ws
./scripts/status
```

- create_env writes .env (paths, domain IDs, image tags)
- create_ws imports repos from dependencies/robot.repos
- status shows the root repo plus nested repos together

Root level git status is not enough.

## Docker build model

There are two images:

- mtt_workspace:base
  Heavy system layer: ZED SDK, depthai, libpointmatcher, libnabo, CUDA helpers,
  OS tools. Slow to rebuild. Keep stable.

- mtt_workspace:devel
  ROS packages and Python dependencies used by the workspace. Used by compile,
  bash, dev, and demo services.

Compose rules:

- Runtime services are image only. They do not rebuild images.
- Images are built explicitly only when you ask for it.

Build once when needed (new clone or Dockerfile change):

```bash
docker compose --profile build -f compose.yaml build devel_image
docker compose -f compose.yaml build base
```

Then run normally without rebuilds:

```bash
docker compose -f compose.yaml run --rm compile
docker compose -f compose.yaml run --rm bash
```

compile builds the ROS workspace inside the container. It does not rebuild the
image.

### Compose services and wiring

compose.yaml defines:

- base
  builds mtt_workspace:base
- devel_image (profile build)
  builds mtt_workspace:devel
- compile
  runs colcon build inside mtt_workspace:devel
- bash/dev
  interactive shell inside the runtime image
- monitor
  foxglove bridge runtime

docker/common.yaml provides base runtime wiring:

- host network, pid, ipc
- NVIDIA variables for GPU
- X11 mounts for RViz and other GUI apps
- shared bind mounts for the workspace and ROS state

### Platform honesty

- supported: Linux x86_64 with Docker
- GPU support requires NVIDIA container runtime
- X11 GUI works on Linux when host allows it

Not guaranteed:
- macOS and Windows (host networking and GPU passthrough are Linux only)
- ARM hosts (images are built for x86_64)

## Key files and knobs

Workspace import:
- dependencies/robot.repos
  list of repos and branches that define the workspace content
- scripts/create_ws
  vcstool import wrapper
- scripts/workspace_source_paths
  source list used by compile to build mtt_core and src/external

Docker build:
- docker/Dockerfile.base
  heavy base (ZED SDK, depthai, libpointmatcher, libnabo)
- docker/Dockerfile
  devel layer and colcon build entry
- compose.yaml
  build and runtime services
- docker/common.yaml
  host network, GPU, X11, mounts

Runtime configs:
- demos/common/config
- demos/live_robot/config
- demos/data_collection/config
- demos/bag_replay

Runtime integration:
- src/external/norlab_robot/launch
- src/external/norlab_robot/config

MTT core packages:
- src/mtt_core/mtt_driver
- src/mtt_core/mtt_bringup
- src/mtt_core/mtt_description
- src/mtt_core/mtt_msgs
- src/mtt_core/mtt_interfaces

Environment:
- .env is written by scripts/create_env
- MTT_IMAGE and MTT_BASE_IMAGE define the image tags
- ROS_DOMAIN_ID and RMW_IMPLEMENTATION control DDS behavior

Colcon build notes:
- compile runs colcon without --symlink-install to avoid editable install issues
- if you add a new package, update dependencies/robot.repos and re-run create_ws

## Host build model

Host build is optional and harder to keep correct.
Use it only if you really need it.

```bash
./scripts/install_host.sh --with-ros
./scripts/install_host.sh --check
```

The script checks:
- ROS 2 Jazzy setup
- rosdep, colcon, vcs
- ZED SDK under /usr/local/zed
- depthai headers under /usr/local/include/depthai
- libpointmatcher and libnabo in the linker cache

The script does not install vendor SDKs for you.
Those are provided by the Docker base image, not by apt.

## Runtime model and demos

Runtime configuration is demo owned.
Do not edit package defaults under src/**/config as the first step.

Key directories:
- demos/common/config
- demos/live_robot/config
- demos/data_collection/config
- demos/bag_replay

The control path is intended to stay simple:

manual intent -> cmd_vel/manual_raw -> cmd_vel/manual
auto intent   -> controller/cmd_vel
arbiter       -> cmd_vel
mtt_can_node  -> CAN bus

## Namespace and frames

Namespace contract:
- top level launch exposes robot_namespace and use_namespace
- topics and services are relative by default
- frames are parameterized
- namespace adaptation happens at the launch boundary

TF contract:
- one robot_description per runtime
- do not publish a second robot tree from camera drivers
- base_footprint -> base_link is the base chain
- center_lidar_link is the physical central LiDAR mount

If the camera tree is disconnected, do not fake an extrinsic. Fix the real
calibration and attach it explicitly.

## Sensor stack expectations

Live runtime expects:
- Xsens IMUs (mti100, mti10)
- Hesai LiDAR
- RoboSense RS-Airy LiDAR
- ZED2i
- OAK only when explicitly enabled and calibrated

## MTT core architecture

Owned packages:
- mtt_driver
- mtt_msgs
- mtt_interfaces
- mtt_description
- mtt_bringup

Control path:

joy/controller/manual -> mtt_ros_wrapper -> mtt_driver -> CAN -> vehicle

Known open points:
- CAN ID ownership still needs live confirmation
- steering truth is weaker than command truth
- OAK support incomplete
- namespace cleanup not finished
- TF not fully closed across all sensors

## CAN bus reference

Primary files:
- MTT_CAN_v1_1_simple.dbc
- mttDriver.dbc
- mtt_driver_architecture.drawio

Tools:
- scripts/mtt_can_monitor.py
- scripts/mtt_can_audit.py
- scripts/mtt_can_export.py

CAN characteristics:
- Standard CAN (11 bit IDs) for control and status
- Extended CAN (29 bit IDs) for charger
- Bit rate is TBD (likely 250 kbps or 500 kbps)

Control frames:
- 0x001 (primary joystick)
- 0x100 (external control, overrides 0x001 when present)

Payload (8 bytes):
- Byte 0: vehicle type (0x00 single track, 0x01 side-by-side left, 0x02 right)
- Byte 1: global switches
  - bit 7 security switch (1 unlocked, 0 locked)
  - bit 6 light (0 on, 1 off) inverted
  - bit 5 direction (0 forward, 1 reverse) inverted
- Byte 2: throttle (0..230, values > 230 are clamped)
- Byte 3: winch (0xe5 in, 0x7f neutral, 0x18 out)
- Byte 4: brake (0..255)
- Byte 5: steer (0..255)
- Byte 6: steering mode bit 0 (0 open loop, 1 closed loop)
- Byte 7: reserved

Temporary firmware behavior (2025-07-03):
- light switch acts as emergency stop
- security switch and light state must be valid for motion

Status frames:
- 0x2FF main module status
  - byte 0 MainSensorTempA (int8)
  - byte 1 MainSensorTempB (int8)
  - bytes 2-3 Tachimeter_Instant (uint16, MSB first)
  - bytes 4-7 Tachimeter_Cumulative (uint32, MSB first)
- 0x300 main controller version
  - bytes 0-3 hardware revision (float, MSB first)
  - bytes 4-7 software revision (float, MSB first)
- 0x301 battery controller version
  - bytes 0-3 hardware revision (float, MSB first)
  - bytes 4-7 software revision (float, MSB first)

BMS frames:
- 0x600 cell temps (int16, MSB first)
- 0x601 ambient/mosfet/heatpad temps (int16, MSB first)
- 0x602 SOC/current/voltage/heatpads/charge time

Charger frames (extended 29 bit):
- 0x1806E5F4 charger command
- 0x18FF50E5 charger status

DTC mapping:
- 1,2 0xC102 over voltage
- 1,3 0xC103 low voltage
- 1,4 0xC104 over temp warning (>90 C)
- 2,2 0xC202 internal voltage fault
- 2,3 0xC203 over temp fault (>100 C)
- 2,4 0xC204 throttle error at power up
- 3,1 0xC301 frequent reset
- 3,2 0xC302 internal reset
- 3,3 0xC303 hall throttle open or short
- 3,4 0xC304 non zero throttle on direction change
- 4,1 0xC401 regen or start up over voltage
- 4,3 0xC403 motor over temperature

Speed conversion (from raw tachometer):

```c
#define MTT_GEAR1 16
#define MTT_GEAR2 36
#define MTT_GEAR3 15
#define MTT_GEAR4 32
#define MTT_GEAR_DRIVE 8
#define MTT_GEAR_TRACK 54
#define MTT_TRACK_LENGTH_CM 393
#define MTT_Encoder_TEET 5
#define MTT_TRACK_LENGTH_KM (MTT_TRACK_LENGTH_CM / 100000.0)

volatile float FinalRatio;

void RPS_to_KMh_Precalc(void)
{
    float ratio1, ratio2;
    ratio1 = ((float)MTT_GEAR2 / (float)MTT_GEAR1) * (float)MTT_Encoder_TEET;
    ratio2 = ((float)MTT_GEAR4 / (float)MTT_GEAR3) * ratio1;
    FinalRatio = (((float)MTT_GEAR_TRACK / (float)MTT_GEAR_DRIVE) * ratio2) * 2;
}

float RPS_to_KMh(float RPS)
{
    if (FinalRatio == 0.0) return 0.0;
    return ((float)RPS / FinalRatio) * (float)MTT_TRACK_LENGTH_KM * 3600.0;
}
```

## CAN tools and usage

If cantools is already available:

```bash
python3 scripts/mtt_can_monitor.py --interface can0
python3 scripts/mtt_can_audit.py --interface can0 --duration 8
python3 scripts/mtt_can_export.py mtt_session.log
```

If you prefer the repo tool env:

```bash
.venv-tools/bin/python scripts/mtt_can_monitor.py --interface can0
```

Capture a candump log:

```bash
candump -L can0 > mtt_session.log
```

Export a subset of IDs:

```bash
.venv-tools/bin/python scripts/mtt_can_export.py mtt_session.log \
  --ids 0x001 0x100 0x2FF 0x600 0x601 0x602 0x603
```

## Development commands

CAN:

```bash
sudo ip link set can0 up type can bitrate 250000
candump can0
candump can0 -xct z -n 10
python3 scripts/mtt_can_audit.py --interface can0 --duration 8
python3 scripts/mtt_can_monitor.py --interface can0
python3 scripts/mtt_health_monitor.py
python3 scripts/mtt_brake_cycle.py --mode release
python3 scripts/mtt_brake_cycle.py --mode pulse --max-brake 1.0 --min-brake 0.0 --period 1.2 --cycles 8
python3 scripts/mtt_brake_cycle.py --mode triangle --max-brake 1.0 --period 2.0 --cycles 5
.venv-tools/bin/python scripts/mtt_can_export.py /path/to/candump.log
```

Gazebo and xacro:

```bash
gz topic --echo --topic /world/default/model/mtt_robot/link/base_footprint/sensor/lidar/scan
gz topic --list

gz sdf -p robot.urdf.xacro > robot.sdf
ros2 run xacro xacro robot_no_collision.urdf.xacro -o robot_no_collision.urdf
ros2 run xacro xacro robot_less_collision.urdf.xacro -o robot_less_collision.urdf
gz sdf -p robot_no_collision.urdf > robot_no_collision.sdf
gz sdf -p robot_less_collision.urdf > robot_less_collision.sdf
```

Controllers and simulation:

```bash
ros2 control list_controllers

ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [-0.5]"
ros2 topic pub /wheel_group_controller/commands std_msgs/msg/Float64MultiArray "data: [-2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5]"

ros2 launch mtt_bringup mtt_simulation.launch.py
ros2 launch mtt_bringup mtt_teleop_controller.launch.py
```

Virtual CAN:

```bash
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

python3 src/mtt_core/mtt_driver/scripts/mtt_cmd_tachometer_sim.py --can-interface vcan0
ros2 launch mtt_driver mtt_composable_system.launch.py can_interface:=vcan0
python3 scripts/mtt_can_monitor.py --interface vcan0
```

## Asset inventory

- MTT_CAN_v1_1_simple.dbc
- mttDriver.dbc
- mtt_driver_architecture.drawio
- mtt_doc_code_exemple_detail/
- img/
- Owner manuals (PDF)
