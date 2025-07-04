#!/bin/bash

# MTT System Test Script
# This script tests the complete MTT system with the new mock server

echo "=== MTT System Test ==="
echo "This script will test the MTT driver with the new mock server"
echo ""

# Function to setup virtual CAN interface
setup_vcan() {
    echo "Setting up virtual CAN interface for testing..."
    sudo modprobe vcan
    sudo ip link add dev vcan0 type vcan 2>/dev/null || true
    sudo ip link set vcan0 up
    echo "Virtual CAN interface vcan0 created and activated"
    echo ""
}

# Function to check CAN interface
check_can_interface() {
    local interface=$1
    if ip link show "$interface" &> /dev/null; then
        echo "CAN interface '$interface' found and available"
        return 0
    else
        echo "CAN interface '$interface' not found"
        return 1
    fi
}

# Determine which CAN interface to use
CAN_INTERFACE=""
if check_can_interface "vcan0"; then
    CAN_INTERFACE="vcan0"
    echo "Using virtual CAN interface: vcan0"
elif check_can_interface "can0"; then
    echo "Warning: Using real CAN interface 'can0' - this may not work without physical CAN hardware"
    echo "Creating virtual CAN interface instead..."
    setup_vcan
    CAN_INTERFACE="vcan0"
else
    echo "No CAN interface found. Setting up virtual CAN interface..."
    setup_vcan
    CAN_INTERFACE="vcan0"
fi

echo ""

# Build the mock server if not already built
if [ ! -f "build/mtt_mock_server" ]; then
    echo "Building MTT mock server..."
    ./build_mtt_mock.sh
    echo ""
fi

echo "Starting MTT mock server on interface: $CAN_INTERFACE"
echo "The mock server will:"
echo "- Listen for control frames on CAN ID 0x100 (and 0x101 for compatibility)"
echo "- Send encoder data on CAN ID 0x2FF"
echo "- Send BMS data on CAN IDs 0x600-0x602"
echo "- Simulate vehicle dynamics based on control commands"
echo ""
echo "Press Ctrl+C to stop the mock server"
echo ""

# Run the mock server
cd build
./mtt_mock_server -c "$CAN_INTERFACE"
