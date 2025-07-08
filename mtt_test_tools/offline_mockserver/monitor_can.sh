#!/bin/bash

# This script monitors CAN traffic and decodes MTT-specific messages

echo "=== MTT CAN Monitor ==="
echo "Monitoring CAN traffic for MTT system messages..."
echo ""
echo "Key CAN IDs to watch:"
echo "- 0x100/0x101: Control frames (from driver to vehicle)"
echo "- 0x2FF: Main controller data (encoder, temperature)"
echo "- 0x600-0x602: BMS data (battery status)"
echo "- 0x300-0x301: Controller version info"
echo ""
echo "Press Ctrl+C to stop monitoring"
echo ""

# Check if candump is available
if ! command -v candump &> /dev/null; then
    echo "Error: candump not found. Install can-utils:"
    echo "  sudo apt install can-utils"
    exit 1
fi

CAN_INTERFACE=""
if ip link show vcan0 &> /dev/null; then
    CAN_INTERFACE="vcan0"
    echo "Using virtual CAN interface: vcan0"
elif ip link show can0 &> /dev/null; then
    CAN_INTERFACE="can0"
    echo "Using CAN interface: can0"
else
    echo "No CAN interface found. Please set up a CAN interface first:"
    echo "For virtual CAN:"
    echo "  sudo modprobe vcan"
    echo "  sudo ip link add dev vcan0 type vcan"
    echo "  sudo ip link set vcan0 up"
    exit 1
fi

echo ""

candump "$CAN_INTERFACE" -t z -x
