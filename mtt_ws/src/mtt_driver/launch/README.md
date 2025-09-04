# MTT-154 Launch Files Organization

## 📂 Structure

### 🎯 `production/` - Production/System Launch Files
- **`mtt_teleop.launch.py`** - Complete teleoperation system (simple, always-on teleop)
- **`mtt_composable_system.launch.py`** - Flexible system with enable/disable options

### 🔧 `development/` - Development Launch Files  
- **`mtt_driver_odometry.launch.py`** - Minimal driver + odometry (no teleop)
- **`mtt_canbus.launch.py`** - CAN driver only

### 🧪 `tests/` - Test Launch Files
- **`mtt_full_test.launch.py`** - Complete test with mock server
- **`mtt_driver_init_test.launch.py`** - Driver initialization test only
- **`mtt_driver_standalone_test.launch.py`** - Standalone driver test
- **`mtt_test.launch.py`** - Basic wrapper + test node

## 🚀 Usage Examples

### Production Systems:
```bash
# Complete teleoperation system  
ros2 launch mtt_driver production/mtt_teleop.launch.py can_interface:=vcan0

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
# Full system test
ros2 launch mtt_driver tests/mtt_full_test.launch.py

# Init test only  
ros2 launch mtt_driver tests/mtt_driver_init_test.launch.py
```

## 🏗️ Architecture

All production systems now use:
- **mtt_ros_wrapper** - Hardware abstraction + ROS integration + safety
- **mtt_odometry_manager** - Multi-mode odometry (single/dual differential/dual serpentine)  
- **joy_linux_node** + **mtt_teleop_joy** - Joystick teleoperation
