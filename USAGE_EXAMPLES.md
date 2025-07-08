# MTT Driver Usage Examples

## Production (Robot) Usage

### Default launch (recommended for robot)
```bash
ros2 launch mtt_driver mtt_teleop.launch.py
```
- Uses `can0` interface
- Production-ready configuration
- Safe for robot deployment

### Explicit production mode
```bash
ros2 launch mtt_driver mtt_teleop.launch.py test_mode:=false
```

### Custom CAN interface
```bash
ros2 launch mtt_driver mtt_teleop.launch.py can_interface:=can1
```

## Testing Usage

### Dedicated test launch (recommended for testing)
```bash
ros2 launch mtt_driver mtt_test.launch.py
```
- Uses `vcan0` interface
- Includes automated test node
- Explicit test configuration

### Main launch in test mode
```bash
ros2 launch mtt_driver mtt_teleop.launch.py test_mode:=true
```
- Overrides to use `vcan0`
- Keeps joystick functionality

### Manual driver testing
```bash
ros2 run mtt_driver mtt_ros_wrapper --ros-args -p test_mode:=true
```

## Quick Test Setup

### Complete test environment
```bash
# Terminal 1: Setup and run mockserver
cd mtt_project
./test_mtt_ros_driver.sh

# Terminal 2: Run driver in test mode
cd mtt_ws && source install/setup.bash
ros2 launch mtt_driver mtt_test.launch.py

# Terminal 3: Monitor
candump vcan0
```

### Manual testing commands
```bash
# Enable and test movement
ros2 topic pub /mtt_aux_cmd mtt_driver/msg/MttAuxCommand '{dead_man_switch: true}' --once
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}}' --once

# Monitor feedback
ros2 topic echo /mtt_tachometer
```

## Configuration Summary

| Mode | CAN Interface | Launch Command | Use Case |
|------|---------------|----------------|----------|
| Production | `can0` | `ros2 launch mtt_driver mtt_teleop.launch.py` | Robot deployment |
| Test | `vcan0` | `ros2 launch mtt_driver mtt_test.launch.py` | Development/testing |
| Test | `vcan0` | `ros2 launch mtt_driver mtt_teleop.launch.py test_mode:=true` | Testing with joystick |
| Custom | User-defined | `ros2 launch mtt_driver mtt_teleop.launch.py can_interface:=canX` | Custom hardware |
