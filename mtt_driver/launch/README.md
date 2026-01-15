# MTT-154 Launch Files Organization

## 📂 Structure

### 🎯 `production/` - Production/System Launch Files
- **`mtt_teleop.launch.py`** - Complete teleoperation system (simple, always-on teleop)
- **`mtt_composable_system.launch.py`** - Flexible system with enable/disable options

#### `mtt_composable_system.launch.py` Details
This launch file starts the complete MTT composable architecture including:
- MTT driver wrapper (hardware abstraction + ROS integration + safety)
- MTT odometry node (dedicated composable odometry calculations)
- Joystick input (optional)
- Teleop controller (optional)

**Architecture:**
```
Driver → Wrapper → Odometry Node
   ↓         ↓           ↓
hardware  /mtt_tachometer  /mtt_odometry
```

### 🔧 `development/` - Development Launch Files  
- **`mtt_driver_odometry.launch.py`** - Minimal driver + odometry (no teleop)
- **`mtt_canbus.launch.py`** - CAN driver only

### 🧪 `tests/` - Test Launch Files
- **`mtt_full_test.launch.py`** - Complete system test (wrapper + odometry + test node)
- **`mtt_driver_init_test.launch.py`** - Driver initialization test with test node
- **`mtt_driver_standalone_test.launch.py`** - Standalone driver test script
- **`mtt_test.launch.py`** - Basic wrapper + test node (minimal test)

## 🚀 Usage Examples

### Production Systems:
```bash
# Complete teleoperation system  
ros2 launch mtt_driver production/mtt_teleop.launch.py can_interface:=vcan0

# Complete system for real hardware (mtt_composable_system)
ros2 launch mtt_driver production/mtt_composable_system.launch.py

# Driver + odometry only (no teleop)
ros2 launch mtt_driver production/mtt_composable_system.launch.py enable_teleop:=false

# Custom logging
ros2 launch mtt_driver production/mtt_composable_system.launch.py driver_log_level:=DEBUG

# === CAN Interface Setup Options ===

# Docker/vcan testing - setup vcan0 from launch
ros2 launch mtt_driver production/mtt_composable_system.launch.py setup_vcan:=true can_interface:=vcan0

# Real hardware - setup real CAN interface with bitrate from launch
ros2 launch mtt_driver production/mtt_composable_system.launch.py setup_real_can:=true can_interface:=can0 can_bitrate:=250000

# Real hardware with custom bitrate (500kbps)
ros2 launch mtt_driver production/mtt_composable_system.launch.py setup_real_can:=true can_interface:=can0 can_bitrate:=500000

# Use existing CAN interface (no setup from launch)
ros2 launch mtt_driver production/mtt_composable_system.launch.py can_interface:=can0

# Flexible system (can disable teleop)
ros2 launch mtt_driver production/mtt_composable_system.launch.py enable_teleop:=false
```

### Development:
```bash
# Driver + odometry only
ros2 launch mtt_driver development/mtt_driver_odometry.launch.py test_mode:=true

# CAN driver testing
ros2 launch mtt_driver development/mtt_canbus.launch.py can_interface:=vcan0
```

### Testing:
```bash
# Full system test (wrapper + odometry + test validation)
ros2 launch mtt_driver tests/mtt_full_test.launch.py

# Driver initialization test with custom interface
ros2 launch mtt_driver tests/mtt_driver_init_test.launch.py can_interface:=vcan0

# Standalone driver test script
ros2 launch mtt_driver tests/mtt_driver_standalone_test.launch.py

# Basic minimal test
ros2 launch mtt_driver tests/mtt_test.launch.py

# Test with debug logging
ros2 launch mtt_driver tests/mtt_full_test.launch.py driver_log_level:=DEBUG

# Skip vcan setup (if already configured)
ros2 launch mtt_driver tests/mtt_test.launch.py setup_vcan:=false
```

## 🏗️ Architecture

All production systems now use:
- **mtt_ros_wrapper** - Hardware abstraction + ROS integration + safety
- **mtt_odometry_manager** - Multi-mode odometry (single/dual differential/dual serpentine)  
- **joy_linux_node** + **mtt_teleop_joy** - Joystick teleoperation

## 🔧 CAN Interface Setup

The `mtt_composable_system.launch.py` now supports automatic CAN interface setup:

### Virtual CAN (vcan) - Docker/Testing
- **`setup_vcan:=true`** - Creates and brings up `vcan0` (no modprobe, Docker-friendly)
- No bitrate configuration needed for vcan interfaces

### Real CAN Hardware
- **`setup_real_can:=true`** - Brings up real CAN interface with specified bitrate
- **`can_interface`** - CAN interface name (default: `can0`)
- **`can_bitrate`** - Bitrate in bps (default: `250000`)

### Manual Setup (Default)
- Both `setup_vcan` and `setup_real_can` default to `false`
- Assumes CAN interface is already configured by the host system
- Use this when CAN is managed externally (systemd, udev rules, etc.)
