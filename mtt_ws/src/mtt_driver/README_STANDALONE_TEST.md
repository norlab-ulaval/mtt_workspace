# MTT Driver Standalone Initialization Test

This directory contains tools for testing the MTT driver initialization frame without the ROS wrapper.

## Files Created

### 1. `scripts/test_driver_init.py`
A standalone Python script that tests the MTT driver initialization and basic functionality:
- Tests driver initialization on a specified CAN interface
- Verifies initial CAN frame generation
- Checks initial state values
- Monitors frame transmission for 3 seconds
- Performs clean shutdown

### 2. `launch/mtt_driver_standalone_test.launch.py`
A ROS2 launch file that runs the standalone test:
- Optionally sets up virtual CAN interface (vcan0)
- Runs the driver initialization test
- Provides detailed logging

## Usage

### Running the standalone script directly:
```bash
cd /home/robot/mtt_project/mtt_ws/src/mtt_driver/mtt_driver
python3 ../scripts/test_driver_init.py [CAN_INTERFACE]
```

### Running via ROS2 launch:
```bash
# Basic test (uses vcan0, no automatic setup)
ros2 launch mtt_driver mtt_driver_standalone_test.launch.py

# With automatic vcan0 setup
ros2 launch mtt_driver mtt_driver_standalone_test.launch.py setup_vcan:=true

# Custom CAN interface (manual setup required)
ros2 launch mtt_driver mtt_driver_standalone_test.launch.py can_interface:=can0 setup_vcan:=false
```

## Test Output

The test performs the following checks:

1. **Basic driver initialization** - Verifies the driver can connect to the CAN interface
2. **Initial CAN frame** - Shows the hexadecimal representation of the initial frame
3. **Initial state values** - Displays all driver state variables
4. **Thread verification** - Confirms sender/listener threads start automatically
5. **Frame monitoring** - Shows 6 consecutive frames being sent
6. **Clean shutdown** - Verifies proper cleanup

### Expected Initial Frame
```
Initial frame: 00 40 00 7F 00 80 00 00
```

Which in decimal is: `[0, 64, 0, 127, 0, 128, 0, 0]`

Where:
- `00`: Vehicle type (single track)
- `40`: Global switches (0x40 with direction bit cleared for reverse)
- `00`: Throttle (idle)
- `7F`: Winch (neutral - 127 decimal)
- `00`: Brake (idle)
- `80`: Steer (center - 128 decimal)
- `00`: Direction mode (open loop)
- `00`: Reserved byte

## Dependencies

- `python3-can` (installed via apt or pip)
- Virtual CAN support in kernel (`vcan` module)
- Sudo access for creating virtual CAN interfaces

## Key Benefits

- **No ROS wrapper dependency** - Tests the core driver logic only
- **Standalone operation** - Can run independently of ROS infrastructure  
- **Clear verification** - Shows exact initialization frame and state values
- **Automated setup** - Optional virtual CAN interface creation
- **Clean testing** - Proper initialization and cleanup cycle
