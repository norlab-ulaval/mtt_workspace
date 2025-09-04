#!/bin/bash

# MTT Mock Server Build and Test Script

echo "Building MTT Mock Server..."
cd "$(dirname "$0")"

mkdir -p build
cd build

cmake ..
make

echo "MTT Mock Server built successfully!"
echo ""
echo "⚠️  SAFETY WARNING ⚠️"
echo "This mock server is for TESTING and DEVELOPMENT only!"
echo "DO NOT run this on a network connected to real MTT hardware!"
echo "It could interfere with actual vehicle operation and cause unexpected behavior."
echo ""
echo "Usage options:"
echo ""
echo "Option 1 - Virtual CAN for testing (recommended for development):"
echo "  sudo modprobe vcan"
echo "  sudo ip link add dev vcan0 type vcan"
echo "  sudo ip link set vcan0 up"
echo "  ./build/mtt_mock_server -c vcan0"
echo ""
echo "Option 2 - Two physical CAN interfaces (isolated testing setup):"
echo "  # ENSURE these interfaces are NOT connected to real MTT hardware"
echo "  # Setup interface 1 for mock server"
echo "  sudo ip link set can0 up type can bitrate 250000"
echo "  ./build/mtt_mock_server -c can0"
echo "  # In another terminal, setup interface 2 for driver"
echo "  sudo ip link set can1 up type can bitrate 250000"
echo "  # Run your MTT driver on can1"
echo ""
echo "Option 3 - Single physical CAN interface (isolated testing):"
echo "  # ENSURE this interface is NOT connected to real MTT hardware"
echo "  sudo ip link set can0 up type can bitrate 250000"
echo "  ./build/mtt_mock_server -c can0"
echo "  # Run MTT driver on same can0 interface"
echo ""
echo "The mock server will:"
echo "- Listen for control frames on 0x100 (and 0x101 for compatibility)"
echo "- Send encoder data on 0x2FF every 100ms"
echo "- Send BMS data on 0x600-0x602 every 500ms"
echo "- Simulate vehicle dynamics based on received control commands"
echo ""
echo "IMPORTANT: Make sure to set safety_unlocked in the control frame for vehicle movement simulation!"
echo ""
echo "For monitoring real MTT systems safely, use monitor_can.sh instead."
