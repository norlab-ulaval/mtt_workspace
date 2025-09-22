#!/usr/bin/env python3

"""
MTT Joint Controller (Frame-Steer Articulated)

Converts cmd_vel commands to joint movements and updates joint_states with PID control.

Key behavior for articulated frame-steer:
- All 20 track rollers rotate in the same direction/speed based on linear.x only.
- Steering comes from the articulation joint (yaw) using angular.z as normalized steering input [-1, 1].
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
    """Simple PID controller implementation with optional external dt"""

    def __init__(self, kp=1.0, ki=0.0, kd=0.0, setpoint=0.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.setpoint = setpoint
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()

    def update(self, measured_value, dt: float | None = None):
        current_time = time.time()
        if dt is None:
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

        # Clamp output to reasonable limits (rad/s)
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
        # Physical rate limit for articulation (closed-loop) in rad/s (60° in 8s ≈ 0.131 rad/s)
        self.declare_parameter(
            "articulation_max_rate_rad_s", 0.131, ParameterDescriptor(description="Max articulation yaw rate (rad/s)")
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
        self.articulation_max_rate = abs(
            self.get_parameter("articulation_max_rate_rad_s").get_parameter_value().double_value
        )

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

        # Command targets and current state
        self.target_linear = 0.0  # normalized [-1,1]
        self.linear_vel = 0.0     # normalized [-1,1]
        self.phi_target = 0.0     # articulation target angle (rad)
        self.steer_input_norm = 0.0  # normalized steering input [-1, 1]
        self.angular_vel = 0.0
        self.dt = 0.02  # nominal 50 Hz
        self._last_integration_time = time.time()

        # Timer for publishing joint states
        self.timer = self.create_timer(self.dt, self.publish_joint_states)  # 50Hz

        self.get_logger().info("MTT Joint Controller started")

    def cmd_vel_callback(self, msg: Twist):
        """Store cmd targets: linear.x in [-1,1], angular.z normalized steering [-1,1]."""
        raw_linear = float(msg.linear.x)
        raw_angular = float(msg.angular.z)

        # Targets only; integration happens in the timer with consistent dt
        self.target_linear = max(-1.0, min(1.0, raw_linear))
        # Expect steering command as normalized [-1, 1]
        self.steer_input_norm = max(-1.0, min(1.0, raw_angular))
        # Map normalized input to articulation angle setpoint in radians
        self.phi_target = self.steer_input_norm * self.articulation_max
        self.artic_pid.setpoint = self.phi_target

    def publish_joint_states(self):
        """Integrate commands, update joints deterministically, publish joints and smoothed cmd."""
        now = time.time()
        dt = now - self._last_integration_time
        # Clamp dt: avoid huge jumps on pauses
        if not math.isfinite(dt) or dt <= 0.0:
            dt = self.dt
        dt = max(0.001, min(0.1, dt))
        self._last_integration_time = now

        # 1) Slew linear velocity toward target
        max_step = max(0.0, self.linear_slew_rate) * dt
        delta = self.target_linear - self.linear_vel
        if delta > max_step:
            self.linear_vel += max_step
        elif delta < -max_step:
            self.linear_vel -= max_step
        else:
            self.linear_vel = self.target_linear

        # 2) Articulation PID: produce yaw rate, integrate position
        phi_meas = self.joint_positions[27]  # current yaw
        phi_rate_cmd = self.artic_pid.update(phi_meas, dt)  # rad/s
        # Enforce physical articulation rate limit (closed-loop behavior)
        if self.articulation_max_rate > 0.0:
            if phi_rate_cmd > self.articulation_max_rate:
                phi_rate_cmd = self.articulation_max_rate
            elif phi_rate_cmd < -self.articulation_max_rate:
                phi_rate_cmd = -self.articulation_max_rate

        # 3) Update joints from rates
        omega_roll = self.linear_vel / max(self.wheel_radius, 1e-6)  # rad/s

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

        # Publish smoothed command for odometry consumers
        pid_cmd = Twist()
        pid_cmd.linear.x = self.linear_vel
        # angular.z carries normalized steering command [-1, 1] expected by wrapper/odometry
        pid_cmd.angular.z = float(self.steer_input_norm)
        self.cmd_vel_pid_pub.publish(pid_cmd)

        # Publish joint states
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.joint_positions
        msg.velocity = []
        msg.effort = []
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
