import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Temperature
from std_msgs.msg import Float64
from mtt_driver.msg import MttAuxCommand
from mtt_driver.mtt_driver import MTTCanDriver, WinchState, DirectionState, SecuritySwitchState

class MTTRosWrapper(Node):
    """ROS 2 node that provides standard interfaces for MTT-154 control.
    
    MTT Driver - Compliant with July 2025 specifications
    
    Subscribes to /cmd_vel and /mtt_aux_cmd topics to control the vehicle
    through CAN bus communication. Implements safety features including
    dead man's switch and emergency stop functionality.
    
    Publishes tachometer data including speed, distance, and temperatures.
    """

    def __init__(self):
        super().__init__('mtt_ros_wrapper')
        
        try:
            self.driver = MTTCanDriver()
        except Exception as e:
            self.get_logger().fatal(f"Could not start driver: {e}")
            return

        self.is_estopped = True
        
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(MttAuxCommand, 'mtt_aux_cmd', self.aux_cmd_callback, 10)
        
        # Publishers for tachometer data
        self.speed_pub = self.create_publisher(Float64, 'mtt/speed_kmh', 10)
        self.distance_pub = self.create_publisher(Float64, 'mtt/distance_km', 10)
        self.temp_a_pub = self.create_publisher(Temperature, 'mtt/temperature_a', 10)
        self.temp_b_pub = self.create_publisher(Temperature, 'mtt/temperature_b', 10)
        
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
        steer = int((-msg.angular.z + 1.0) * 127.5)
        self.driver.set_steer(steer)
        
        # Set direction based on linear velocity sign
        direction = DirectionState.Forward if msg.linear.x >= 0 else DirectionState.Reverse
        self.driver.set_direction(direction)

    def aux_cmd_callback(self, msg: MttAuxCommand):
        """Handle auxiliary commands (brake, winch, dead man's switch)."""
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

    def control_loop(self):
        """Main control loop - sends CAN frames at 20 Hz and publishes tachometer data."""
        if self.is_estopped:
            self.driver.set_security_switch(SecuritySwitchState.SafetyLocked)
        
        self.driver.send_can_frame()
        
        # Publish tachometer data if available
        self._publish_tachometer_data()

    def _publish_tachometer_data(self):
        """Publish tachometer data to ROS topics"""
        tach_data = self.driver.get_tachometer_data()
        
        if tach_data['new_data_available']:
            # Publish speed
            speed_msg = Float64()
            speed_msg.data = self.driver.get_speed_kmh()
            self.speed_pub.publish(speed_msg)
            
            # Publish distance
            distance_msg = Float64()
            distance_msg.data = self.driver.get_cumulative_distance_km()
            self.distance_pub.publish(distance_msg)
            
            # Publish temperatures
            temp_a_msg = Temperature()
            temp_a_msg.header.stamp = self.get_clock().now().to_msg()
            temp_a_msg.header.frame_id = "mtt_sensor_a"
            temp_a_msg.temperature = tach_data['main_sensor_temp_a']
            temp_a_msg.variance = 0.0
            self.temp_a_pub.publish(temp_a_msg)
            
            temp_b_msg = Temperature()
            temp_b_msg.header.stamp = self.get_clock().now().to_msg()
            temp_b_msg.header.frame_id = "mtt_sensor_b"
            temp_b_msg.temperature = tach_data['main_sensor_temp_b']
            temp_b_msg.variance = 0.0
            self.temp_b_pub.publish(temp_b_msg)

    def destroy_node(self):
        """Clean shutdown with emergency stop."""
        self.get_logger().info("Shutting down MTT driver - applying emergency stop")
        self.driver.emergency_stop()
        self.driver.send_can_frame()
        self.driver.shutdown()
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
