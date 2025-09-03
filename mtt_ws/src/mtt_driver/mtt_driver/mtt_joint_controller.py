#!/usr/bin/env python3

"""
MTT Joint Controller for Real Hardware
Converts cmd_vel commands to joint movements and updates joint_states.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import math

class MttJointController(Node):
    def __init__(self):
        super().__init__('mtt_joint_controller')
        
        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Subscribers  
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Joint states
        self.joint_names = [
            # Tracks (chenilles)
            '1_continuous', '2_continuous', '3_continuous', '4_continuous', '5_continuous',
            '6_continuous', '7_continuous', '8_continuous', '9_continuous', '10_continuous',
            '11_continuous', '12_continuous', '13_continuous', '14_continuous', '15_continuous',
            '16_continuous', '17_continuous', '18_continuous', '19_continuous', '20_continuous',
            # Main wheels
            'frontleft_wheel', 'backleft_wheel', 'frontright_wheel', 'backright_wheel',
            # Trailer wheels
            'Remorque_lien_roue_gauche_joint', 'Remorque_lien_roue_droite_joint',
            # Orientation joints  
            'roll', 'yaw', 'pitch'
        ]
        
        # Joint positions (accumulated for continuous joints)
        self.joint_positions = [0.0] * len(self.joint_names)
        
        # Robot parameters
        self.wheel_radius = 0.15  # meters
        self.track_length = 2.0   # distance between front and back wheels
        
        # Current velocities
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        # Timer for publishing joint states
        self.timer = self.create_timer(0.02, self.publish_joint_states)  # 50Hz
        
        self.get_logger().info('MTT Joint Controller started')

    def cmd_vel_callback(self, msg: Twist):
        """Process velocity commands and update joint positions"""
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z
        
        # Calculate differential velocities for turning
        # Simple differential drive model
        if abs(self.angular_vel) > 0.01:  # If turning
            # Calculate left/right wheel speeds for differential steering
            left_vel = self.linear_vel - (self.angular_vel * self.track_length / 2.0)
            right_vel = self.linear_vel + (self.angular_vel * self.track_length / 2.0)
        else:
            # Going straight
            left_vel = right_vel = self.linear_vel
        
        # Convert linear velocities to angular velocities (rad/s)
        dt = 0.02  # 50Hz update rate
        left_angular_vel = left_vel / self.wheel_radius
        right_angular_vel = right_vel / self.wheel_radius
        
        # Update track positions (chenilles 1-20)
        # Left tracks (1-10)
        for i in range(10):
            self.joint_positions[i] += left_angular_vel * dt
            
        # Right tracks (11-20)  
        for i in range(10, 20):
            self.joint_positions[i] += right_angular_vel * dt
            
        # Update main wheel positions
        self.joint_positions[20] += left_angular_vel * dt   # frontleft_wheel
        self.joint_positions[21] += left_angular_vel * dt   # backleft_wheel
        self.joint_positions[22] += right_angular_vel * dt  # frontright_wheel
        self.joint_positions[23] += right_angular_vel * dt  # backright_wheel
        
        # Force trailer axle joints to stay fixed (no motor on trailer wheels)
        self.joint_positions[24] = 0.0  # Remorque_lien_roue_gauche_joint
        self.joint_positions[25] = 0.0  # Remorque_lien_roue_droite_joint
        
        # Update orientation (yaw based on angular velocity)
        self.joint_positions[27] += self.angular_vel * dt  # yaw joint
        
        # Keep roll and pitch at 0 for now
        self.joint_positions[26] = 0.0  # roll
        self.joint_positions[28] = 0.0  # pitch

    def publish_joint_states(self):
        """Publish current joint states"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.joint_positions
        msg.velocity = []  # Empty for now
        msg.effort = []   # Empty for now
        
        self.joint_state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MttJointController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
