# MTT Driver Initialization Test

This test launch file is designed to test the MTT driver initialization and basic frame generation **without** the ROS wrapper.

## Purpose

- Test the basic initialization of the MTT CAN driver
- Verify initial frame generation
- Check that all driver state variables are properly initialized
- Test thread startup and shutdown
- Validate CAN frame transmission without ROS dependencies

## Usage

### Quick Test (with automatic vcan setup)

```bash
# Build the package first
cd /home/robot/mtt_project/mtt_ws
colcon build --packages-select mtt_driver

# Source the workspace
source install/setup.bash

# Run the test with automatic virtual CAN setup
ros2 launch mtt_driver mtt_driver_init_test.launch.py
```

### Manual CAN Interface Setup

If you want to use a different CAN interface or set it up manually:

```bash
# Setup virtual CAN interface manually
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

# Run test without automatic setup
ros2 launch mtt_driver mtt_driver_init_test.launch.py setup_vcan:=false

# Or use a different CAN interface
ros2 launch mtt_driver mtt_driver_init_test.launch.py can_interface:=can1 setup_vcan:=false
```

### Direct Script Execution

You can also run the test script directly:

```bash
# Using default vcan0
ros2 run mtt_driver mtt_test_node

# Using specific CAN interface
ros2 run mtt_driver mtt_test_node can1
```

## What the Test Does

1. **Interface Check**: Verifies the CAN interface exists
2. **Driver Initialization**: Creates MTTCanDriver instance with test parameters
3. **Frame Verification**: Checks initial CAN frame content
4. **State Validation**: Verifies all driver state variables
5. **Thread Testing**: Starts sender/listener threads
6. **Frame Transmission**: Sends frames for 3 seconds
7. **Clean Shutdown**: Properly stops the driver

## Expected Output

The test should show:
- Successful CAN interface detection
- Driver initialization confirmation
- Initial CAN frame in hex format
- Driver state variables (direction, winch, etc.)
- Live frame transmission
- Clean shutdown

## Differences from Full System

This test runs **only** the low-level CAN driver (`MTTCanDriver`) without:
- ROS wrapper (`MTTRosWrapper`)
- ROS topics (cmd_vel, mtt_aux_cmd, etc.)
- Odometry publishing
- Parameter server integration

This isolation helps debug driver-level issues without ROS complexity.
