#!/usr/bin/env python3

"""
MTT Joint Controller (Frame-Steer Articulated)

Converts cmd_vel commands to joint movements and updates joint_states with PID control.

Key behavior for articulated frame-steer:
- All 20 track rollers rotate in the same direction/speed based on linear.x only.
- Steering comes from the articulation joint (yaw) using angular.z as target angle (phi_cmd).
- Trailer wheels remain passive (positions held at 0).
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
        super().__init__("mtt_joint_controller")

        # Declare PID parameters (velocity smoothing for linear speed)
        # Deprecated: previously used a PID to smooth linear speed; now replaced by slew limiter
        self.declare_parameter(
            "velocity_pid.kp", 1.0, ParameterDescriptor(description="[Deprecated] Velocity PID proportional gain")
        )
        self.declare_parameter(
            "velocity_pid.ki", 0.1, ParameterDescriptor(description="[Deprecated] Velocity PID integral gain")
        )
        self.declare_parameter(
            "velocity_pid.kd", 0.05, ParameterDescriptor(description="[Deprecated] Velocity PID derivative gain")
        )
        # New: linear slew-rate limiter (units: normalized units per second; input expected in [-1, 1])
        self.declare_parameter(
            "linear_slew_rate", 3.0, ParameterDescriptor(description="Max change per second for linear.x (normalized)")
        )
        # Articulation (yaw) PID parameters
        self.declare_parameter(
            "articulation_pid.kp", 2.0, ParameterDescriptor(description="Articulation PID proportional gain")
        )
        self.declare_parameter(
            "articulation_pid.ki", 0.0, ParameterDescriptor(description="Articulation PID integral gain")
        )
        self.declare_parameter(
            "articulation_pid.kd", 0.2, ParameterDescriptor(description="Articulation PID derivative gain")
        )
        self.declare_parameter(
            "articulation_max_deg", 35.0, ParameterDescriptor(description="Max articulation absolute angle (deg)")
        )

        # Initialize PID controllers
        v_kp = self.get_parameter("velocity_pid.kp").get_parameter_value().double_value
        v_ki = self.get_parameter("velocity_pid.ki").get_parameter_value().double_value
        v_kd = self.get_parameter("velocity_pid.kd").get_parameter_value().double_value
        a_kp = self.get_parameter("articulation_pid.kp").get_parameter_value().double_value
        a_ki = self.get_parameter("articulation_pid.ki").get_parameter_value().double_value
        a_kd = self.get_parameter("articulation_pid.kd").get_parameter_value().double_value

        # Keep instance for backward compatibility but it's no longer used for linear smoothing
        self.velocity_pid = SimplePID(v_kp, v_ki, v_kd)
        self.artic_pid = SimplePID(a_kp, a_ki, a_kd)
        self.articulation_max = math.radians(
            self.get_parameter("articulation_max_deg").get_parameter_value().double_value
        )
        self.linear_slew_rate = self.get_parameter("linear_slew_rate").get_parameter_value().double_value

        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.cmd_vel_pid_pub = self.create_publisher(Twist, "/cmd_vel_pid", 10)

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel_raw", self.cmd_vel_callback, 10)

        # Joint states
        self.joint_names = [
            # Tracks (chenilles)
            "1_continuous",
            "2_continuous",
            "3_continuous",
            "4_continuous",
            "5_continuous",
            "6_continuous",
            "7_continuous",
            "8_continuous",
            "9_continuous",
            "10_continuous",
            "11_continuous",
            "12_continuous",
            "13_continuous",
            "14_continuous",
            "15_continuous",
            "16_continuous",
            "17_continuous",
            "18_continuous",
            "19_continuous",
            "20_continuous",
            # Main wheels
            "frontleft_wheel",
            "backleft_wheel",
            "frontright_wheel",
            "backright_wheel",
            # Trailer wheels
            "Remorque_lien_roue_gauche_joint",
            "Remorque_lien_roue_droite_joint",
            # Orientation joints
            "roll",
            "yaw",
            "pitch",
        ]

        # Joint positions (accumulated for continuous joints)
        self.joint_positions = [0.0] * len(self.joint_names)

        # Robot parameters
        self.wheel_radius = 0.15  # meters
        self.track_length = 2.0  # legacy param (not used for steering in frame-steer)

        # Current velocities
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.dt = 0.02  # 50 Hz

        # Timer for publishing joint states
        self.timer = self.create_timer(self.dt, self.publish_joint_states)  # 50Hz

        self.get_logger().info("MTT Joint Controller started")

    def cmd_vel_callback(self, msg: Twist):
        """Process cmd_vel as: linear.x => track speed; angular.z => articulation angle (PID)."""
        raw_linear = float(msg.linear.x)
        raw_angular = float(msg.angular.z)

        # 1) Smooth linear velocity with a slew limiter (normalized command in [-1, 1])
        target_linear = max(-1.0, min(1.0, raw_linear))
        max_step = max(0.0, self.linear_slew_rate) * self.dt
        delta = target_linear - self.linear_vel
        if delta > max_step:
            self.linear_vel += max_step
        elif delta < -max_step:
            self.linear_vel -= max_step
        else:
            self.linear_vel = target_linear

        # 2) Articulation PID: treat angular.z as target articulation angle (radians)
        # Clamp to safe limit
        phi_cmd = max(-self.articulation_max, min(self.articulation_max, raw_angular))
        self.artic_pid.setpoint = phi_cmd
        phi_meas = self.joint_positions[27]  # yaw joint current position
        phi_rate_cmd = self.artic_pid.update(phi_meas)  # rad/s limited by PID internal clamp

        # 3) Publish smoothed command for odometry consumers
        # linear.x = smoothed forward velocity; angular.z = articulation target angle (rad)
        pid_cmd = Twist()
        pid_cmd.linear.x = self.linear_vel
        pid_cmd.angular.z = phi_cmd
        self.cmd_vel_pid_pub.publish(pid_cmd)

        # 4) Update continuous joints
        dt = self.dt
        omega_roll = self.linear_vel / max(self.wheel_radius, 1e-6)

        # Rollers: 1..20 all same direction/speed
        for i in range(20):
            new_pos = self.joint_positions[i] + omega_roll * dt
            if math.isfinite(new_pos):
                self.joint_positions[i] = new_pos
            else:
                self.get_logger().warning(f"NaN in roller {i+1}, keeping previous position")

        # Sprockets/main wheels mirror same angular velocity
        self.joint_positions[20] += omega_roll * dt  # frontleft_wheel
        self.joint_positions[21] += omega_roll * dt  # backleft_wheel
        self.joint_positions[22] += omega_roll * dt  # frontright_wheel
        self.joint_positions[23] += omega_roll * dt  # backright_wheel

        # Trailer wheels passive
        self.joint_positions[24] = 0.0
        self.joint_positions[25] = 0.0

        # Articulation yaw updated by PID-produced rate
        self.joint_positions[27] += phi_rate_cmd * dt

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
        msg.effort = []  # Empty for now

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


if __name__ == "__main__":
    main()
