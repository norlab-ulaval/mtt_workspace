# MTT-154 System Architecture and Configuration Manual

## 1. System Architecture Overview

The software is organized into functional layers to separate hardware abstraction, state estimation, and high-level autonomy.

### 1.1 Hardware Abstraction Layer (mtt_driver)
The driver layer manages low-level communication with the physical components.
- **mtt_can_node**: Handles SocketCAN communication. It serializes `cmd_vel` into manufacturer-specific CAN frames for the motor controllers and deserializes telemetry data (battery, temperature, encoder counts).
- **mtt_articulation_sensor_node**: Interfaces with the STM32-based encoder at the pivot point to provide the articulation angle between the tractor and trailer.
- **mtt_odometry_node**: Computes the robot's dead-reckoning state. It integrates encoder data using the kinematic model of an articulated vehicle.
- **mtt_health_monitor_node**: Monitors system heartbeats and telemetry age. It triggers a safety stop if communication timeouts exceed specified thresholds.

### 1.2 Control and Arbitration Layer (mtt_control)
This layer manages the flow of command velocities and system states.
- **mtt_mode_manager_node**: Implements the system state machine (MANUAL, AUTO, ESTOP).
- **mtt_cmd_arbiter_node**: A priority-based multiplexer. It selects the active command source (e.g., Joystick vs. Autonomous planner) based on the current system mode.
- **mtt_manual_cmd_filter_node**: Applies kinematic constraints (acceleration/velocity limits) to manual inputs to ensure stability.

### 1.3 Perception Layer (mtt_perception)
Processes sensor data for downstream mapping and navigation.
- **cloud_merger_node**: Performs spatial filtering on the Lidar point cloud. It uses bounding boxes to remove points corresponding to the robot's own chassis and trailer, preventing self-mapping artifacts in the ICP algorithm.

### 1.4 Description and Transforms (mtt_description)
- Defines the kinematic chain via URDF and XACRO.
- Manages the Transform Tree (TF), ensuring spatial consistency between sensors (Hesai Lidar, GPS) and the robot's base frame.

## 2. Launch Interface and Parameters

The primary entry point for the live robot is `norlab_robot/launch/live_robot.launch.py`.

### 2.1 Primary Launch Arguments
The following arguments can be passed to the launch file to control system behavior:

- `setup_real_can` (bool, default: true): If true, executes shell commands to initialize the `can0` interface with the specified bitrate.
- `enable_sensors` (bool, default: true): Activates the drivers for the Lidar and GPS hardware.
- `enable_mapping` (bool, default: true): Starts the `norlab_icp_mapper` stack. Note: There is a `mapping_delay_seconds` (default: 10.0) to allow sensor streams to stabilize before initialization.
- `enable_perception` (bool, default: true): Enables the `cloud_merger_node` for Lidar cleaning.
- `gps_mode` (string, default: 'serial'): Specifies the connection type for the Reach RS unit ('serial' or 'tcp').
- `driver_params_file` (string): Path to the YAML file containing motor controller and odometry constants.

### 2.2 Key YAML Configuration Parameters
Configuration files are located in `demos/common/config/`.

#### mtt_driver_params.yaml
- `wheel_base`: Distance between the front and rear axles.
- `track_width`: Lateral distance between wheels.
- `encoder_resolution`: Ticks per revolution for the drive motors.
- `max_steering_angle`: Physical limit of the articulation joint in radians.

#### mtt_control.yaml
- `max_linear_velocity`: Maximum allowed speed in m/s.
- `max_angular_velocity`: Maximum allowed yaw rate in rad/s.
- `acceleration_limit`: Maximum linear acceleration to prevent mechanical strain.

#### mtt_health_monitor.yaml
- `telemetry_timeout_seconds`: Maximum allowed age of CAN telemetry before entering a fault state.
- `command_timeout_seconds`: Maximum age of the `cmd_vel` message before neutralizing motor output.

## 3. Deployment without Containerization

To deploy the stack on a native host, these independent processes must be initialized.

1.  **Communication Middleware (Zenoh)**
    Zenoh serves as the RMW (ROS Middleware) abstraction for cross-network communication.
    ```bash
    ros2 run rmw_zenoh_cpp rmw_zenohd
    ```

2.  **Base Robot Stack**
    Initializes hardware drivers, state estimation, and mapping.
    ```bash
    ros2 launch norlab_robot live_robot.launch.py setup_real_can:=true
    ```

3.  **Autonomous Navigation (WILN)**
    Initializes the teach-and-repeat logic and path followers.
    ```bash
    ros2 launch norlab_robot teach_repeat.launch.py
    ```


## 4. System Integration

The `mtt_core` provides the low-level API via standard ROS 2 topics:
- **Subscribed**: `/cmd_vel` (geometry_msgs/msg/Twist)
- **Published**: `/odom` (nav_msgs/msg/Odometry), `/tf` (tf2_msgs/msg/TFMessage), `/joint_states` (sensor_msgs/msg/JointState).

The `norlab_robot` and `WILN` packages utilize these interfaces to perform high-precision localization and trajectory tracking.
