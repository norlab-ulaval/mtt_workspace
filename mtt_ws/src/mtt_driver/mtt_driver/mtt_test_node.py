#!/usr/bin/env python3
"""
MTT Test Node

Simple test node to verify MTT driver functionality without joystick.
Sends test commands and monitors responses.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mtt_msgs.msg import MttAuxCommand, MttTachometerData
from std_msgs.msg import Float64
import time

class MTTTestNode(Node):
    """Test node for MTT driver validation."""

    def __init__(self):
        super().__init__('mtt_test_node')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.aux_cmd_pub = self.create_publisher(MttAuxCommand, 'mtt_aux_cmd', 10)
        
        # Subscribers for feedback
        self.create_subscription(MttTachometerData, 'mtt_tachometer', self.tachometer_callback, 10)
        self.create_subscription(Float64, 'mtt_speed', self.speed_callback, 10)
        
        # Test sequence timer
        self.test_timer = self.create_timer(2.0, self.run_test_sequence)
        self.test_step = 0
        
        self.get_logger().info("MTT Test Node started - Running automated test sequence")
        
        # Data storage
        self.last_speed = 0.0
        self.last_tachometer = None

    def tachometer_callback(self, msg: MttTachometerData):
        """Handle tachometer data feedback."""
        self.last_tachometer = msg
        self.get_logger().info(f"Tachometer: Speed={msg.speed_kmh:.2f} km/h, "
                              f"Distance={msg.distance_km:.3f}km, "
                              f"Temp A={msg.main_sensor_temp_a}°C, Temp B={msg.main_sensor_temp_b}°C")

    def speed_callback(self, msg: Float64):
        """Handle speed data feedback."""
        self.last_speed = msg.data
        self.get_logger().info(f"Speed: {msg.data:.2f} km/h")

    def send_aux_command(self, dead_man=False, brake=0.0, winch=MttAuxCommand.WINCH_NEUTRAL):
        """Send auxiliary command."""
        aux_msg = MttAuxCommand()
        aux_msg.dead_man_switch = dead_man
        aux_msg.brake = brake
        aux_msg.winch_command = winch
        self.aux_cmd_pub.publish(aux_msg)

    def send_velocity_command(self, linear_x=0.0, angular_z=0.0):
        """Send velocity command."""
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = angular_z
        self.cmd_vel_pub.publish(twist_msg)

    def run_test_sequence(self):
        """Run automated test sequence."""
        
        if self.test_step == 0:
            self.get_logger().info("=== TEST 1: Enable safety (dead man's switch) ===")
            self.send_aux_command(dead_man=True)
            
        elif self.test_step == 1:
            self.get_logger().info("=== TEST 2: Move forward slowly ===")
            self.send_velocity_command(linear_x=0.3)  # 30% forward
            
        elif self.test_step == 2:
            self.get_logger().info("=== TEST 3: Turn right while moving ===")
            self.send_velocity_command(linear_x=0.3, angular_z=-0.5)  # Forward + right turn
            
        elif self.test_step == 3:
            self.get_logger().info("=== TEST 4: Stop and brake ===")
            self.send_velocity_command(linear_x=0.0, angular_z=0.0)
            self.send_aux_command(dead_man=True, brake=1.0)  # Full brake
            
        elif self.test_step == 4:
            self.get_logger().info("=== TEST 5: Reverse slowly ===")
            self.send_aux_command(dead_man=True, brake=0.0)  # Release brake
            self.send_velocity_command(linear_x=-0.2)  # 20% reverse
            
        elif self.test_step == 5:
            self.get_logger().info("=== TEST 6: Test winch extend ===")
            self.send_velocity_command(linear_x=0.0)  # Stop movement
            self.send_aux_command(dead_man=True, winch=MttAuxCommand.WINCH_IN)
            
        elif self.test_step == 6:
            self.get_logger().info("=== TEST 7: Test winch retract ===")
            self.send_aux_command(dead_man=True, winch=MttAuxCommand.WINCH_OUT)
            
        elif self.test_step == 7:
            self.get_logger().info("=== TEST 8: Stop winch ===")
            self.send_aux_command(dead_man=True, winch=MttAuxCommand.WINCH_NEUTRAL)
            
        elif self.test_step == 8:
            self.get_logger().info("=== TEST 9: Emergency stop (disable dead man's switch) ===")
            self.send_aux_command(dead_man=False)
            self.send_velocity_command(linear_x=0.0, angular_z=0.0)
            
        elif self.test_step == 9:
            self.get_logger().info("=== TEST SEQUENCE COMPLETED ===")
            self.get_logger().info("All tests completed successfully!")
            if self.last_tachometer:
                self.get_logger().info(f"Final status - Speed: {self.last_tachometer.speed_kmh:.2f} km/h, "
                                     f"Distance: {self.last_tachometer.distance_m:.2f}m")
            
            # Reset for next cycle
            self.test_step = -1
            
        self.test_step += 1

def main(args=None):
    rclpy.init(args=args)
    test_node = MTTTestNode()
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
