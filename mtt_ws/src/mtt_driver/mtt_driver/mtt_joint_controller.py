#!/usr/bin/env python3

"""
MTT Joint Controller for Real Hardware
Converts cmd_vel commands to joint movements and updates joint_states with PID control.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import ParameterDescriptor
import math
import time

class SimplePID:
    """Simple PID controller implementation"""
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, setpoint=0.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.setpoint = setpoint
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()
    
    def update(self, measured_value):
        current_time = time.time()
        dt = current_time - self.prev_time
        if dt <= 0.0:
            dt = 0.02  # Fallback to 50Hz
        
        error = self.setpoint - measured_value
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        
        # Protect against NaN and infinite values
        if not math.isfinite(output):
            print(f"WARNING: PID output NaN/inf detected, resetting: {output}")
            output = 0.0
            self.integral = 0.0  # Reset integral windup
        
        # Clamp output to reasonable limits
        output = max(-10.0, min(10.0, output))
        
        self.prev_error = error
        self.prev_time = current_time
        
        return output

class MttJointController(Node):
    def __init__(self):
        super().__init__('mtt_joint_controller')
        
        # Declare PID parameters
        self.declare_parameter('velocity_pid.kp', 1.0, 
                             ParameterDescriptor(description='Velocity PID proportional gain'))
        self.declare_parameter('velocity_pid.ki', 0.1,
                             ParameterDescriptor(description='Velocity PID integral gain'))
        self.declare_parameter('velocity_pid.kd', 0.05,
                             ParameterDescriptor(description='Velocity PID derivative gain'))
        
        # Initialize PID controllers for left and right tracks
        kp = self.get_parameter('velocity_pid.kp').get_parameter_value().double_value
        ki = self.get_parameter('velocity_pid.ki').get_parameter_value().double_value
        kd = self.get_parameter('velocity_pid.kd').get_parameter_value().double_value
        
        self.left_velocity_pid = SimplePID(kp, ki, kd)
        self.right_velocity_pid = SimplePID(kp, ki, kd)
        
        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.cmd_vel_pid_pub = self.create_publisher(Twist, '/cmd_vel_pid', 10)
        
        # Subscribers  
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel_raw', self.cmd_vel_callback, 10)
        
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
        """Process velocity commands with PID control and update joint positions"""
        # Apply PID control to smooth the velocity commands
        raw_linear = msg.linear.x
        raw_angular = msg.angular.z
        
        # Update PID setpoints
        self.left_velocity_pid.setpoint = raw_linear - (raw_angular * self.track_length / 2.0)
        self.right_velocity_pid.setpoint = raw_linear + (raw_angular * self.track_length / 2.0)
        
        # Current measured velocities (simplified - would use actual feedback)
        current_left_vel = self.linear_vel - (self.angular_vel * self.track_length / 2.0)
        current_right_vel = self.linear_vel + (self.angular_vel * self.track_length / 2.0)
        
        # Apply PID control
        left_output = self.left_velocity_pid.update(current_left_vel)
        right_output = self.right_velocity_pid.update(current_right_vel)
        
        # Convert back to linear/angular for output
        smoothed_linear = (left_output + right_output) / 2.0
        smoothed_angular = (right_output - left_output) / self.track_length
        
        # Publish smoothed command for ROS wrapper
        pid_cmd = Twist()
        pid_cmd.linear.x = smoothed_linear
        pid_cmd.angular.z = smoothed_angular
        self.cmd_vel_pid_pub.publish(pid_cmd)
        
        # Update internal state for joint calculations
        self.linear_vel = smoothed_linear
        self.angular_vel = smoothed_angular
        
        # Protect against NaN propagation
        if not math.isfinite(self.linear_vel):
            self.get_logger().warning("Linear velocity NaN detected, resetting to 0")
            self.linear_vel = 0.0
        if not math.isfinite(self.angular_vel):
            self.get_logger().warning("Angular velocity NaN detected, resetting to 0")
            self.angular_vel = 0.0
        
        # Calculate differential velocities for turning
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
            new_pos = self.joint_positions[i] + left_angular_vel * dt
            if math.isfinite(new_pos):
                self.joint_positions[i] = new_pos
            else:
                self.get_logger().warning(f"NaN in left track {i+1}, keeping previous position")
            
        # Right tracks (11-20)  
        for i in range(10, 20):
            new_pos = self.joint_positions[i] + right_angular_vel * dt
            if math.isfinite(new_pos):
                self.joint_positions[i] = new_pos
            else:
                self.get_logger().warning(f"NaN in right track {i+1}, keeping previous position")
            
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
