# MTT-154 Driver

ROS 2 driver for the MTT-154 All-Terrain Vehicle.

## Overview

ROS 2 driver for the MTT-154 vehicle. Communicates via CAN bus for movement control, steering, braking, and winch operations.

## Quick Start

```bash
cd mtt_ws
colcon build --packages-select mtt_driver
source install/setup.bash
ros2 launch mtt_driver mtt_driver.launch.py
```

## Package Structure
```
mtt_ws/src/mtt_driver/
├── config/           # Configuration files
├── launch/           # Launch files
├── mtt_driver/       # Python package
├── msg/              # Custom message definitions
└── test/             # Test files
```

## Setup CAN Interface
```bash
sudo ip link set can0 up type can bitrate 250000
```

## Documentation
- CAN Bus Specification: `documentations/CANBus_Specification.md`
- Owner's Manual: `documentations/Owner's_manual_MTT-154_2024-1.pdf`