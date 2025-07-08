# MTT ROS 2 Driver Testing Guide

This guide explains how to test the MTT ROS 2 driver using the offline mockserver instead of the physical robot.

## Overview

The testing setup includes:
- **Test Mode**: Uses `test_mode` parameter to switch between real hardware (`can0`) and virtual testing (`vcan0`)
- **Offline Mockserver**: Simulates the MTT-154 vehicle's CAN bus behavior
- **Virtual CAN Interface**: Creates a virtual network for testing without hardware
- **ROS 2 Driver**: The actual driver code you want to test
- **Test Nodes**: Automated testing scripts to verify functionality

## Testing Modes

### Production Mode (Default)
- Uses `can0` interface for real hardware
- Default configuration for robot deployment
- Launch: `ros2 launch mtt_driver mtt_teleop.launch.py`

### Test Mode 
- Uses `vcan0` interface for simulation
- Requires explicit test mode activation
- Launch: `ros2 launch mtt_driver mtt_teleop.launch.py test_mode:=true`

## Prerequisites

### Required Packages
```bash
# Ubuntu/Debian
sudo apt update
sudo apt install can-utils build-essential cmake

# Python packages
pip3 install python-can

# ROS 2 (if not already installed)
# Follow official ROS 2 installation guide for your distro
```

## Quick Start

### 1. Build ROS Package

```bash
cd ~/mtt_ws
# Build the ROS package
colcon build --packages-select mtt_driver

# Source the workspace
source install/setup.bash
```

### 2. Setup and Run Mockserver (Standalone)

```bash
# Build standalone mockserver
cd ~/mtt_ws/mtt_test_tools/offline_mockserver
chmod +x build_mtt_mock.sh
./build_mtt_mock.sh

# Setup virtual CAN interface
sudo modprobe vcan
sudo ip link delete vcan0 2>/dev/null || true  # Remove if exists
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up

# Start mockserver (standalone, no ROS dependencies)
cd build
./mtt_mock_server -c vcan0
```

### 3. Run ROS Driver in Test Mode

In a new terminal:
```bash
cd ~/mtt_ws
source /opt/ros/jazzy/setup.bash   # For ROS 2 Jazzy
# source /opt/ros/humble/setup.bash # For ROS 2 Humble  
# source /opt/ros/foxy/setup.bash  # For ROS 2 Foxy
source install/setup.bash

# Option A: Use dedicated test launch file
ros2 launch mtt_driver mtt_test.launch.py

# Option B: Use main launch file with test mode
ros2 launch mtt_driver mtt_teleop.launch.py test_mode:=true

# Option C: Run individual components
ros2 run mtt_driver mtt_ros_wrapper --ros-args -p test_mode:=true
```

### 3. Monitor System

In another terminal:
```bash
# Monitor CAN traffic
candump vcan0

# Monitor ROS topics
ros2 topic echo /mtt_tachometer
ros2 topic echo /mtt_speed
ros2 topic list
```

## Manual Testing

### Step-by-Step Manual Setup

#### 1. Setup Virtual CAN
```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up
```

#### 2. Start Mockserver
```bash
# Standalone mockserver (no ROS dependencies)
cd ~/mtt_ws/mtt_test_tools/offline_mockserver
./build_mtt_mock.sh
cd build
./mtt_mock_server -c vcan0
```

#### 3. Build and Run ROS Driver
```bash
cd ~/mtt_ws
source /opt/ros/foxy/setup.bash  # or your ROS 2 distro
colcon build --packages-select mtt_driver
source install/setup.bash

# Option A: Test launch file (driver + test node)
ros2 launch mtt_driver mtt_test.launch.py

# Option B: Main launch file in test mode
ros2 launch mtt_driver mtt_teleop.launch.py test_mode:=true

# Option C: Run components individually
ros2 run mtt_driver mtt_ros_wrapper --ros-args -p test_mode:=true
```

### Manual Command Testing

#### Send Test Commands
```bash
# Enable safety and move forward
ros2 topic pub /mtt_aux_cmd mtt_driver/msg/MttAuxCommand '{dead_man_switch: true}' --once
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}}' --once

# Test braking
ros2 topic pub /mtt_aux_cmd mtt_driver/msg/MttAuxCommand '{dead_man_switch: true, brake: 1.0}' --once

# Test reverse
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: -0.3}}' --once

# Test steering
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3}, angular: {z: 0.5}}' --once

# Emergency stop
ros2 topic pub /mtt_aux_cmd mtt_driver/msg/MttAuxCommand '{dead_man_switch: false}' --once
```

#### Monitor Feedback
```bash
# Real-time tachometer data
ros2 topic echo /mtt_tachometer

# Speed monitoring
ros2 topic echo /mtt_speed

# Distance monitoring  
ros2 topic echo /mtt_distance

# Temperature monitoring
ros2 topic echo /mtt_temperature
```

## Expected Behavior

### 1. Mockserver Output
You should see:
```
[MTT_MOCK] Starting MTT vehicle mock server on vcan0
[MTT_MOCK] Control received - Safety: UNLOCKED, Dir: FWD, Mode: THROTTLING, Throttle: 115, ...
```

### 2. ROS Driver Output
You should see:
```
[INFO] [mtt_ros_wrapper]: MTT Driver initialized on CAN interface: vcan0
[INFO] [mtt_ros_wrapper]: MTT ROS Wrapper ready. E-stop is ACTIVE by default.
```

### 3. CAN Traffic
Monitor with `candump vcan0`:
```
vcan0  100   [8]  00 80 73 7F 00 80 00 00    # Control frame (driver -> mockserver)
vcan0  2FF   [8]  19 16 00 1C 00 00 00 64    # Main data frame (mockserver -> driver)
vcan0  600   [8]  00 19 00 1A 00 18 00 1B    # BMS frame 1 (mockserver -> driver)
```

## Testing Scenarios

### 1. Basic Movement Test
- Enable dead man's switch
- Send forward command
- Verify speed feedback
- Send stop command

### 2. Safety System Test
- Try movement without dead man's switch (should be ignored)
- Enable dead man's switch
- Disable dead man's switch during movement (should stop)

### 3. Direction Control Test
- Move forward
- Move reverse
- Test steering left/right

### 4. Brake System Test
- Move forward
- Apply brakes (should override throttle)
- Release brakes

### 5. Winch Control Test
- Extend winch
- Retract winch
- Stop winch

## Troubleshooting

### Common Issues

#### 1. "CAN interface 'vcan0' not found"
```bash
# Recreate the interface
sudo ip link delete vcan0
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up
```

#### 2. "Permission denied" on CAN interface
```bash
# Add user to can group or run with sudo
sudo usermod -a -G dialout $USER
# Re-login or use sudo
```

#### 3. No CAN traffic visible
```bash
# Check if mockserver is running
ps aux | grep mtt_mock_server

# Check if interface is up
ip link show vcan0
```

#### 4. ROS 2 import errors
```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash
source install/setup.bash
```

#### 5. Python-can not found
```bash
pip3 install python-can
# or
sudo apt install python3-can
```

### Validation Checklist

- [ ] Virtual CAN interface created successfully
- [ ] Mockserver starts without errors
- [ ] ROS 2 driver connects to CAN interface
- [ ] Can send control commands via ROS topics
- [ ] Receiving tachometer feedback from mockserver
- [ ] Safety system prevents movement when dead man's switch is off
- [ ] Emergency stop works correctly
- [ ] Speed, distance, and temperature data is published
- [ ] CAN traffic visible with candump

## Advanced Testing

### Load Testing
Run multiple test cycles to verify stability:
```bash
# Run test node in loop
while true; do
    timeout 30 ros2 run mtt_driver mtt_test_node
    sleep 5
done
```

### Network Simulation
Test with CAN frame delays and errors:
```bash
# Add artificial latency to vcan0
sudo tc qdisc add dev vcan0 root netem delay 10ms
```

### Multi-Node Testing
Test with multiple ROS nodes subscribing to the same topics to verify data integrity.

## Integration with Hardware

Once testing is complete, switch to real hardware by simply using the default launch:
```bash
# Production mode (default) - uses can0
ros2 launch mtt_driver mtt_teleop.launch.py

# Or explicitly specify production mode
ros2 launch mtt_driver mtt_teleop.launch.py test_mode:=false

# Or specify the real CAN interface
ros2 launch mtt_driver mtt_teleop.launch.py can_interface:=can0
```

Ensure proper CAN bus setup for your hardware:
```bash
sudo ip link set can0 up type can bitrate 250000
```

## Files Modified for Testing

- `config/mtt_config.yaml`: Kept `can0` as default (production ready)
- `launch/mtt_teleop.launch.py`: Added test_mode and can_interface arguments
- `mtt_ros_wrapper.py`: Added test_mode parameter support
- Added `mtt_test_node.py`: Automated testing node
- Added `mtt_test.launch.py`: Test launch configuration with explicit test mode
- Added `test_mtt_ros_driver.sh`: Complete test setup script

### Launch Arguments Summary

| Launch File | Default Mode | Test Mode |
|-------------|--------------|-----------|
| `mtt_teleop.launch.py` | Production (`can0`) | `test_mode:=true` → `vcan0` |
| `mtt_test.launch.py` | Test (`vcan0`) | Always test mode |

### Parameter Options

| Parameter | Default | Description |
|-----------|---------|-------------|
| `test_mode` | `false` | When `true`, forces use of `vcan0` |
| `can_interface` | `can0` | Specific CAN interface to use |

This ensures that:
- **Default behavior is production-ready** (uses `can0`)
- **Test mode is explicit** (must specify `test_mode:=true` or use test launch file)
- **No accidental testing in production** (default config is safe for robot)
