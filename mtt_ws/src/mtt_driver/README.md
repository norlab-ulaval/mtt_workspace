# MTT-154 Driver

ROS 2 driver for MTT-154 All-Terrain Vehicle.

**July 2025 Specification Compliant** - Includes tachometer data processing, proper security switch handling, and emergency stop functionality.

## Installation

```bash
cd ~/ros2_ws
colcon build --packages-select mtt_driver
source install/setup.bash
```

## Usage

```bash
# Setup CAN interface
sudo ip link set can0 up type can bitrate 250000

# Launch driver
ros2 launch mtt_driver mtt_driver.launch.py

# Test teleop
ros2 launch mtt_driver mtt_teleop.launch.py
```

## Components

- `mtt_ros_wrapper.py`: Main ROS 2 wrapper with tachometer data publishing
- `mtt_driver.py`: Pure Python CAN bus interface (July 2025 compliant)
- `mtt_teleop_joy.py`: Joystick control
```bash
ros2 run mtt_driver mtt_teleop_joy
```

## Topics

### Subscribed Topics

- `/cmd_vel` (geometry_msgs/Twist): Standard ROS velocity commands
- `/mtt_aux_cmd` (mtt_driver/MttAuxCommand): Auxiliary commands (brake, winch, dead man's switch)
- `/joy` (sensor_msgs/Joy): Raw joystick input (teleop node only)

### Published Topics

- `/cmd_vel` (geometry_msgs/Twist): Velocity commands from joystick (teleop node only)
- `/mtt_aux_cmd` (mtt_driver/MttAuxCommand): Auxiliary commands from joystick (teleop node only)
- `/mtt/speed_kmh` (std_msgs/Float64): Vehicle speed in km/h from tachometer
- `/mtt/distance_km` (std_msgs/Float64): Cumulative distance in kilometers
- `/mtt/temperature_a` (sensor_msgs/Temperature): Main sensor temperature A
- `/mtt/temperature_b` (sensor_msgs/Temperature): Main sensor temperature B

## Controller Mapping

The default controller mapping is configured for 8BitDo controllers:

- **Left Stick Vertical** (axis 1): Forward/backward motion
- **Right Stick Horizontal** (axis 3): Steering
- **Right Trigger** (axis 5): Brake control
- **D-Pad Vertical** (axis 7): Winch control (up=in, down=out)
- **Right Shoulder Button** (button 5): Dead man's switch

## Safety Features

- **Dead Man's Switch**: Motion is disabled unless the dead man's switch is active
- **E-Stop**: Releasing the dead man's switch immediately stops all motion
- **Brake Priority**: Brake commands take precedence over throttle commands
- **Safe Defaults**: All commands default to safe values (stopped, braked)

## CAN Interface (July 2025 Specification)

The driver uses SocketCAN for communication and implements the July 2025 specification:

- **0x001**: Joystick/remote controller 
- **0x100**: Auxiliary control (this driver) - **overrides 0x001 when active**
- **0x2ff**: Tachometer data (receive only)

```bash
sudo ip link set can0 up type can bitrate 500000
```

### Critical Safety Requirements

- **Security switch MUST be unlocked** for operation (controlled via dead man's switch)
- **Light state acts as emergency stop** (test both states to determine operational mode)
- **Direction is controlled by master system** (this driver)
- **Emergency stop** immediately stops all motion and applies maximum brake

### Tachometer Data

The driver automatically receives and processes tachometer data from the vehicle:
- **Speed calculation**: Real-time km/h from encoder ticks
- **Distance tracking**: Cumulative distance with ~2-3mm accuracy  
- **Temperature monitoring**: Two temperature sensors for diagnostics
- **Gear ratios**: Precisely calculated using manufacturer specifications

## Troubleshooting

### CAN Bus Issues
- Ensure the CAN interface is up and properly configured
- Check CAN bus connections and termination
- Verify the correct CAN interface name in the driver

### Joystick Issues
- Verify joystick is connected: `ls /dev/input/js*`
- Check joystick permissions: `sudo chmod a+rw /dev/input/js0`
- Test joystick input: `ros2 topic echo /joy`

### Build Issues
- Ensure all dependencies are installed
- Check that the workspace is properly sourced
- Verify Python path includes the package

## Development

### Adding New Controllers

To add support for a new controller, modify the `axis_map` and `button_map` dictionaries in `mtt_teleop_joy.py`.

### Extending Functionality

The modular architecture allows easy extension:
- Add new commands to `MttAuxCommand.msg`
- Implement new control logic in `mtt_driver.py`
- Create new ROS nodes that publish to `/cmd_vel` and `/mtt_aux_cmd`

