#!/bin/bash

# MTT ROS 2 Driver Test Script
# Tests the complete ROS 2 MTT driver stack with the offline mockserver

set -e

echo "=== MTT ROS 2 Driver Test Setup ==="
echo "This script will test the MTT ROS 2 driver with the offline mockserver"
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

# Function to setup virtual CAN interface
setup_vcan() {
    print_status "Setting up virtual CAN interface..."
    sudo modprobe vcan 2>/dev/null || true
    sudo ip link delete vcan0 2>/dev/null || true
    sudo ip link add dev vcan0 type vcan
    sudo ip link set vcan0 up
    
    if ip link show vcan0 &>/dev/null; then
        print_success "Virtual CAN interface vcan0 created and activated"
    else
        print_error "Failed to create virtual CAN interface"
        exit 1
    fi
}

# Function to build the mockserver
build_mockserver() {
    print_status "Building MTT mockserver..."
    cd mtt_test_tools/offline_mockserver
    if [ ! -f "build_mtt_mock.sh" ]; then
        print_error "Mockserver build script not found!"
        exit 1
    fi
    
    chmod +x build_mtt_mock.sh
    ./build_mtt_mock.sh
    
    if [ -f "build/mtt_mock_server" ]; then
        print_success "Mockserver built successfully"
    else
        print_error "Failed to build mockserver"
        exit 1
    fi
    cd ../..
}

# Function to build ROS 2 driver
build_ros_driver() {
    print_status "Building ROS 2 MTT driver..."
    cd mtt_ws
    
    # Source ROS 2 if available
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
        print_status "Sourced ROS 2 Humble"
    elif [ -f "/opt/ros/iron/setup.bash" ]; then
        source /opt/ros/iron/setup.bash
        print_status "Sourced ROS 2 Iron"
    elif [ -f "/opt/ros/rolling/setup.bash" ]; then
        source /opt/ros/rolling/setup.bash
        print_status "Sourced ROS 2 Rolling"
    else
        print_warning "ROS 2 installation not found in standard locations"
        print_warning "Please source your ROS 2 setup.bash manually"
    fi
    
    # Install Python dependencies
    print_status "Installing Python dependencies..."
    pip3 install python-can || print_warning "Failed to install python-can"
    
    # Build the package
    colcon build --packages-select mtt_driver
    
    if [ -f "install/setup.bash" ]; then
        print_success "ROS 2 driver built successfully"
    else
        print_error "Failed to build ROS 2 driver"
        exit 1
    fi
    cd ..
}

# Function to start mockserver in background
start_mockserver() {
    print_status "Starting MTT mockserver on vcan0..."
    cd mtt_test_tools/offline_mockserver/build
    ./mtt_mock_server -c vcan0 &
    MOCKSERVER_PID=$!
    echo $MOCKSERVER_PID > /tmp/mtt_mockserver.pid
    print_success "Mockserver started with PID $MOCKSERVER_PID"
    sleep 2  # Give mockserver time to initialize
    cd ../../..
}

# Function to test CAN communication
test_can_communication() {
    print_status "Testing basic CAN communication..."
    
    # Send a test frame and check if mockserver responds
    timeout 5 candump vcan0 &
    CANDUMP_PID=$!
    sleep 1
    
    # Send safety unlock command
    print_status "Sending safety unlock command..."
    cansend vcan0 100#0080007F00800000
    
    sleep 2
    kill $CANDUMP_PID 2>/dev/null || true
    
    print_success "CAN communication test completed"
}

# Function to monitor CAN traffic
monitor_can() {
    print_status "Monitoring CAN traffic (Press Ctrl+C to stop)..."
    echo ""
    echo "Key CAN IDs to watch:"
    echo "- 0x100: Control frames (from ROS driver to mockserver)"
    echo "- 0x2FF: Main data (from mockserver to ROS driver)"
    echo "- 0x600-0x602: BMS data (from mockserver to ROS driver)"
    echo ""
    
    candump vcan0
}

# Function to cleanup processes
cleanup() {
    print_status "Cleaning up..."
    
    # Kill mockserver if running
    if [ -f "/tmp/mtt_mockserver.pid" ]; then
        MOCKSERVER_PID=$(cat /tmp/mtt_mockserver.pid)
        kill $MOCKSERVER_PID 2>/dev/null || true
        rm -f /tmp/mtt_mockserver.pid
        print_status "Stopped mockserver"
    fi
    
    # Kill any remaining CAN monitoring
    pkill -f "candump vcan0" 2>/dev/null || true
    
    print_success "Cleanup completed"
}

# Trap to cleanup on exit
trap cleanup EXIT

# Main execution
print_status "Starting MTT ROS 2 driver test setup..."
echo ""

# Check prerequisites
if ! command -v cansend &> /dev/null; then
    print_error "can-utils not found. Install with: sudo apt install can-utils"
    exit 1
fi

if ! command -v colcon &> /dev/null; then
    print_error "colcon not found. Install with: pip3 install colcon-common-extensions"
    exit 1
fi

# Execute setup steps
setup_vcan
build_mockserver
build_ros_driver
start_mockserver
test_can_communication

print_success "MTT ROS 2 driver test environment is ready!"
echo ""
echo "==================== NEXT STEPS ===================="
echo ""
echo "1. In Terminal 1 (Monitor CAN traffic):"
echo "   ./test_mtt_ros_driver.sh monitor"
echo ""
echo "2. In Terminal 2 (Launch ROS 2 driver):"
echo "   cd mtt_ws"
echo "   source install/setup.bash"
echo "   ros2 launch mtt_driver mtt_teleop.launch.py"
echo ""
echo "3. In Terminal 3 (Test with manual commands):"
echo "   cd mtt_ws"
echo "   source install/setup.bash"
echo "   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}}' --once"
echo "   ros2 topic pub /mtt_aux_cmd mtt_driver/msg/MttAuxCommand '{dead_man_switch: true}' --once"
echo ""
echo "4. Monitor topics:"
echo "   ros2 topic echo /mtt_tachometer"
echo "   ros2 topic echo /mtt_speed"
echo ""
echo "Press Ctrl+C to stop mockserver and cleanup"
echo "=================================================="

# If argument provided, handle specific commands
if [ "$1" = "monitor" ]; then
    monitor_can
elif [ "$1" = "cleanup" ]; then
    cleanup
    exit 0
else
    # Keep script running to maintain mockserver
    wait
fi
