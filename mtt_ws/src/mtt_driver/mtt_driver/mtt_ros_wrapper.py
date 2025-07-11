#!/usr/bin/env python3
"""
MTT-154 ROS2 Wrapper

ROS2 node providing standard interfaces for MTT-154 control.
Implements CANBus_Specification.md v1.1 (2025-07-03).

Subscribes to /cmd_vel and /mtt_aux_cmd topics to control the vehicle
through CAN bus communication. Implements safety features including
dead man's switch and emergency stop functionality.

Publishes tachometer data including speed, distance, and temperatures.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Temperature
from std_msgs.msg import Float64, Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from mtt_msgs.msg import MttAuxCommand, MttTachometerData
from .mtt_driver import MTTCanDriver, WinchState, DirectionState, SecuritySwitchState

class MTTRosWrapper(Node):
    """ROS 2 node that provides standard interfaces for MTT-154 control.
    
    Subscribes to /cmd_vel and /mtt_aux_cmd topics to control the vehicle
    through CAN bus communication. Implements safety features including
    dead man's switch and emergency stop functionality.
    
    Publishes tachometer data including speed, distance, and temperatures.
    """

    def __init__(self):
        super().__init__('mtt_ros_wrapper')
        
        # Declare and get parameters
        self.declare_parameter('can_interface', 'can0')  # Default to real hardware
        self.declare_parameter('test_mode', False)       # Default to production mode
        
        can_interface = self.get_parameter('can_interface').get_parameter_value().string_value
        test_mode = self.get_parameter('test_mode').get_parameter_value().bool_value
        
        # Override CAN interface if in test mode
        if test_mode:
            can_interface = 'vcan0'
            self.get_logger().info("TEST MODE: Using virtual CAN interface (vcan0)")
        else:
            self.get_logger().info(f"PRODUCTION MODE: Using CAN interface: {can_interface}")
        
        try:
            self.driver = MTTCanDriver(can_interface)
            self.get_logger().info(f"MTT Driver initialized successfully")
        except Exception as e:
            self.get_logger().fatal(f"Could not start driver: {e}")
            return

        self.is_estopped = True
        
        # ROS odometry tracking variables
        self.ros_position_x = 0.0  # Current ROS odometry position
        self.last_driver_distance = 0.0  # Last distance from driver to calculate increments
        
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(MttAuxCommand, 'mtt_aux_cmd', self.aux_cmd_callback, 10)
        
        # Publishers for tachometer and odometry data
        self.speed_pub = self.create_publisher(Float64, 'mtt_speed', 10)
        self.distance_pub = self.create_publisher(Float64, 'mtt_distance', 10)
        self.temperature_pub = self.create_publisher(Float64MultiArray, 'mtt_temperature', 10)
        self.tachometer_pub = self.create_publisher(MttTachometerData, 'mtt_tachometer', 10)
        self.odometry_pub = self.create_publisher(Odometry, 'mtt_odometry', 10)
        
        # Control loop timer (20 Hz)
        self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info("MTT ROS Wrapper ready. E-stop is ACTIVE by default.")

    def cmd_vel_callback(self, msg: Twist):
        """Handle standard ROS velocity commands."""
        if self.is_estopped: 
            return
        
        # Convert linear velocity to throttle (0-230 range)
        throttle = int(abs(msg.linear.x) * 230)
        self.driver.set_throttle(throttle)
        
        # Convert angular velocity to steering (0-255 range, 128 is center)
        steer = int((msg.angular.z + 1.0) * 127.5)
        self.driver.set_steer(steer)
        
        # Set direction based on linear velocity sign
        direction = DirectionState.Forward if msg.linear.x >= 0 else DirectionState.Reverse
        self.driver.set_direction(direction)

    def aux_cmd_callback(self, msg: MttAuxCommand):
        """Handle auxiliary commands (brake, winch, dead man's switch, light toggle)."""
        if not msg.dead_man_switch and not self.is_estopped:
            self.is_estopped = True
            self.driver.reset_motion_commands()
            self.get_logger().warn("E-STOP ENGAGED (Dead man's switch released).")
        elif msg.dead_man_switch and self.is_estopped:
            self.is_estopped = False
            self.driver.set_security_switch(SecuritySwitchState.SafetyUnlocked)
            self.get_logger().info("E-stop disengaged. Motion enabled.")

        if self.is_estopped: 
            return
            
        brake = int(msg.brake * 255)
        self.driver.set_brake(brake)
        
        if msg.winch_command == MttAuxCommand.WINCH_IN: 
            self.driver.set_winch_state(WinchState.WinchIn)
        elif msg.winch_command == MttAuxCommand.WINCH_OUT: 
            self.driver.set_winch_state(WinchState.WinchOut)
        else: 
            self.driver.set_winch_state(WinchState.WinchNeutral)

        # Light state handling (0=off, 1=on)
        if hasattr(msg, 'light_state'):
            from .mtt_driver import LightState
            if msg.light_state == 1:
                self.driver.set_light_state(LightState.On)
            else:
                self.driver.set_light_state(LightState.Off)

    def control_loop(self):
        """Main control loop - publishes tachometer data and manages safety state."""
        if self.is_estopped:
            self.driver.set_security_switch(SecuritySwitchState.SafetyLocked)
        
        # CAN frames are sent automatically by the driver's keepalive thread
        
        # Publish tachometer data if available
        self._publish_tachometer_data()

    def _publish_tachometer_data(self):
        """Publish tachometer and odometry data to ROS topics"""
        tach_data = self.driver.get_tachometer_data()
        odometry_data = self.driver.get_odometry_data()
        
        if tach_data.new_data_available:
            # Publish speed (km/h)
            speed_msg = Float64()
            speed_msg.data = odometry_data['speed_kmh']
            self.speed_pub.publish(speed_msg)
            
            # Publish distance (km)
            distance_msg = Float64()
            distance_msg.data = odometry_data['total_distance_m'] / 1000.0  # Convert m to km
            self.distance_pub.publish(distance_msg)
            
            # Publish temperature array
            temp_msg = Float64MultiArray()
            temp_msg.data = [
                odometry_data['temperature_a'], 
                odometry_data['temperature_b']
            ]
            self.temperature_pub.publish(temp_msg)
            
            # Publish complete tachometer data
            tachometer_msg = MttTachometerData()
            tachometer_msg.header.stamp = self.get_clock().now().to_msg()
            tachometer_msg.header.frame_id = "mtt_base_link"
            tachometer_msg.main_sensor_temp_a = odometry_data['temperature_a']
            tachometer_msg.main_sensor_temp_b = odometry_data['temperature_b']
            tachometer_msg.tachometer_instant = tach_data.tachometer_instant
            tachometer_msg.tachometer_cumulative = tach_data.tachometer_cumulative
            tachometer_msg.speed_ms = odometry_data['speed_ms']
            tachometer_msg.speed_kmh = odometry_data['speed_kmh']
            tachometer_msg.distance_km = odometry_data['total_distance_m'] / 1000.0
            self.tachometer_pub.publish(tachometer_msg)
            
            # Publish standard ROS odometry
            self._publish_odometry(odometry_data)

    def _publish_odometry(self, odometry_data):
        """Publish standard ROS2 odometry message"""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "mtt_base_link"

        # Calculate incremental distance from driver
        current_driver_distance = odometry_data['total_distance_m']
        distance_increment = current_driver_distance - self.last_driver_distance
        
        # Apply direction to the increment (not the total distance)
        if odometry_data['direction'] == 'Forward':
            self.ros_position_x += distance_increment
        else:  # Reverse
            self.ros_position_x -= distance_increment
        
        # Update last distance for next calculation
        self.last_driver_distance = current_driver_distance

        # Set position in odometry message
        odom_msg.pose.pose.position.x = self.ros_position_x
        odom_msg.pose.pose.position.y = 0.0
        odom_msg.pose.pose.position.z = 0.0

        # Orientation - no rotation
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = 0.0
        odom_msg.pose.pose.orientation.w = 1.0

        # Velocity
        odom_msg.twist.twist.linear.x = odometry_data['speed_ms']
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0

        self.odometry_pub.publish(odom_msg)

    def destroy_node(self):
        """Clean shutdown with emergency stop."""
        self.get_logger().info("Shutting down MTT driver - applying emergency stop")
        if hasattr(self, 'driver') and self.driver:
            self.driver.emergency_stop()
            self.driver.send_can_frame()
            self.driver.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    wrapper_node = MTTRosWrapper()
    try:
        rclpy.spin(wrapper_node)
    except KeyboardInterrupt:
        pass
    finally:
        wrapper_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
