# Development commands

Small commands that were previously sitting in `notes.txt` and are still worth keeping nearby.

## CAN

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

Example frame payload:

```text
[0, 64, 0, 127, 0, 128, 0, 0]
```

## Gazebo and xacro

```bash
gz topic --echo --topic /world/default/model/mtt_robot/link/base_footprint/sensor/lidar/scan
gz topic --list

gz sdf -p robot.urdf.xacro > robot.sdf
ros2 run xacro xacro robot_no_collision.urdf.xacro -o robot_no_collision.urdf
ros2 run xacro xacro robot_less_collision.urdf.xacro -o robot_less_collision.urdf
gz sdf -p robot_no_collision.urdf > robot_no_collision.sdf
gz sdf -p robot_less_collision.urdf > robot_less_collision.sdf
```

## Controllers and simulation

```bash
ros2 control list_controllers

ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [-0.5]"
ros2 topic pub /wheel_group_controller/commands std_msgs/msg/Float64MultiArray "data: [-2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5]"

ros2 launch mtt_bringup mtt_simulation.launch.py
ros2 launch mtt_bringup mtt_teleop_controller.launch.py
```

## Virtual CAN

```bash
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

python3 src/mtt_core/mtt_driver/scripts/mtt_cmd_tachometer_sim.py --can-interface vcan0
ros2 launch mtt_driver mtt_composable_system.launch.py can_interface:=vcan0
python3 scripts/mtt_can_monitor.py --interface vcan0
```
