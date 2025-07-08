#!/bin/bash

# MTT Driver Quick Test Script
# Run this from your ROS workspace root

set -e

echo "=== MTT Driver ROS Package Test ==="

# Check if we're in a ROS workspace
if [ ! -f "src/mtt_driver/package.xml" ]; then
    echo "Error: Please run this script from your ROS workspace root (where src/ directory is)"
    exit 1
fi

# Build the package
echo "Building mtt_driver package..."
colcon build --packages-select mtt_driver

# Source the workspace
source install/setup.bash

# Setup virtual CAN
echo "Setting up virtual CAN interface..."
sudo modprobe vcan 2>/dev/null || true
sudo ip link delete vcan0 2>/dev/null || true
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up

echo "Virtual CAN interface vcan0 is ready!"
echo ""
echo "Now you can run:"
echo "  ros2 launch mtt_driver mtt_full_test.launch.py    # Complete test environment"
echo "  ros2 launch mtt_driver mtt_test.launch.py         # Driver + test node only"
echo "  ros2 run mtt_driver mtt_mock_server -c vcan0      # Mockserver only"
echo ""
echo "Monitor with:"
echo "  candump vcan0"
echo "  ros2 topic echo /mtt_tachometer"
