```mermaid
flowchart TB
    %% Input Sources
    Joy["Joy Controller<br/>8BitDo/Xbox/PS4"]
    Auto["Autonomy Stack<br/>Nav2/Custom"]
    Manual["Manual Commands<br/>Direct ROS Topics"]

    %% ROS 2 Layer
    subgraph ROS2Layer ["ROS 2 Interface Layer"]
        direction TB
        JoyNode["joy_node<br/>sensor_msgs/Joy"]
        TeleopNode["mtt_teleop_joy.py<br/>Joystick Translation"]
        WrapperNode["mtt_ros_wrapper.py<br/>ROS to CAN Bridge"]
    end

    %% Message Topics
    subgraph Topics ["ROS Topics"]
        direction LR
        JoyTopic["/joy<br/>sensor_msgs/Joy"]
        CmdVelTopic["/cmd_vel<br/>geometry_msgs/Twist"]
        AuxTopic["/mtt_aux_cmd<br/>MttAuxCommand"]
        TachTopic["/mtt_tachometer<br/>MttTachometerData"]
        SpeedTopic["/mtt_speed<br/>std_msgs/Float64"]
        DistTopic["/mtt_distance<br/>std_msgs/Float64"]
        TempTopic["/mtt_temperature<br/>std_msgs/Float64MultiArray"]
        OdomTopic["/mtt_odometry<br/>nav_msgs/Odometry"]
    end

    %% CAN Driver Layer
    subgraph CANLayer ["CAN Driver Layer"]
        direction TB
        CanDriver["mtt_driver.py<br/>Pure Python CAN Interface"]
        CanBus[("CAN Bus<br/>can0 Interface")]
    end

    %% Vehicle Hardware
    MTT154["MTT-154 Vehicle<br/>Physical Hardware"]

    %% Connections from inputs
    Joy --> JoyNode
    Auto --> CmdVelTopic
    Manual --> CmdVelTopic
    Manual --> AuxTopic

    %% ROS node connections (Control)
    JoyNode --> JoyTopic
    JoyTopic --> TeleopNode
    TeleopNode --> CmdVelTopic
    TeleopNode --> AuxTopic
    
    %% Path to vehicle (Control)
    CmdVelTopic --> WrapperNode
    AuxTopic --> WrapperNode
    WrapperNode --> CanDriver
    
    %% CAN to vehicle (Control)
    CanDriver --> CanBus
    CanBus --> MTT154
    
    %% Telemetry feedback from vehicle
    MTT154 --> CanBus
    CanBus --> CanDriver
    CanDriver --> WrapperNode

    %% ROS node connections (Telemetry)
    WrapperNode --> TachTopic
    WrapperNode --> SpeedTopic
    WrapperNode --> DistTopic
    WrapperNode --> TempTopic
    WrapperNode --> OdomTopic

    %% Styling
    classDef inputClass fill:#ffffff,stroke:#01579b,stroke-width:2px,color:#000000
    classDef rosClass fill:#ffffff,stroke:#4a148c,stroke-width:2px,color:#000000
    classDef topicClass fill:#ffffff,stroke:#e65100,stroke-width:2px,color:#000000
    classDef canClass fill:#ffffff,stroke:#1b5e20,stroke-width:2px,color:#000000
    classDef vehicleClass fill:#ffffff,stroke:#b71c1c,stroke-width:3px,color:#000000

    class Joy,Auto,Manual inputClass
    class JoyNode,TeleopNode,WrapperNode rosClass
    class JoyTopic,CmdVelTopic,AuxTopic,TachTopic,SpeedTopic,DistTopic,TempTopic,OdomTopic topicClass
    class CanDriver,CanBus canClass
    class MTT154 vehicleClass
```

## Current MTT-154 Driver Architecture
*Updated for CAN Bus Specification v1.1 Compliance with Telemetry*

### Component Overview

**Input Sources:**
- **Joy Controller**: Physical joystick (8BitDo, Xbox, PS4) for manual teleoperation
- **Autonomy Stack**: Navigation systems (Nav2, custom) for autonomous operation
- **Manual Commands**: Direct ROS topic publishing for testing and development

**ROS 2 Interface Layer:**
- **joy_node**: Standard ROS joystick driver publishing sensor_msgs/Joy
- **mtt_teleop_joy.py**: Translates joystick input to vehicle commands (/cmd_vel, /mtt_aux_cmd)
- **mtt_ros_wrapper.py**: Bridges standard ROS topics to the CAN interface for control and publishes telemetry data from the vehicle

**ROS Topics:**
*Control Topics:*
- **/joy**: Raw joystick input (sensor_msgs/Joy)
- **/cmd_vel**: Standard velocity commands (geometry_msgs/Twist)
- **/mtt_aux_cmd**: Auxiliary commands like brake, winch, and dead-man switch (MttAuxCommand)

*Telemetry Topics:*
- **/mtt_speed**: Vehicle speed in km/h (std_msgs/Float64)
- **/mtt_distance**: Cumulative distance traveled in km (std_msgs/Float64)
- **/mtt_temperature**: Motor controller temperatures (std_msgs/Float64MultiArray)
- **/mtt_tachometer**: Raw, detailed telemetry from the CAN bus (MttTachometerData)
- **/mtt_odometry**: Standard odometry message for navigation stacks (nav_msgs/Odometry)

**CAN Driver Layer:**
- **mtt_driver.py**: Pure Python CAN interface with a threaded listener for receiving telemetry. No ROS dependencies
- **CAN Bus**: SocketCAN interface (e.g., can0) for physical vehicle communication

**Vehicle Hardware:**
- **MTT-154**: Physical vehicle with a CAN bus control and telemetry interface

### Key Features

1. **Standard ROS Integration**: Uses standard topics (/cmd_vel, /mtt_odometry) for easy integration
2. **Telemetry Publishing**: Provides real-time vehicle data (speed, distance, temperature) on ROS topics
3. **Odometry for Navigation**: Publishes standard nav_msgs/Odometry for use with Nav2 and SLAM
4. **Flexible Input**: Supports joysticks, autonomy stacks, and direct manual commands
5. **Safety First**: Implements a dead-man switch, E-stop functionality, and safe startup defaults
6. **Modular Design**: The ROS wrapper and pure Python CAN driver can be used independently
7. **CAN Bus v1.1 Compliance**: Adheres to the latest CAN bus specification for reliable control
8. **Speed/Distance Tracking**: Accurate odometry from tachometer data

### Safety Systems

- Dead man's switch requirement for motion
- Emergency stop functionality
- Safe default values on startup
- Proper shutdown procedures
- **Security switch management (CANBus_Specification.md v1.1)**: Bit 7 (0x80) for vehicle unlock
- **Temporary emergency stop patch**: Light control acts as E-stop mechanism
- **System readiness checks**: Both security switch and light state validation

### Data Flow

**Teleoperation**: Joy → joy_node → mtt_teleop_joy → Topics → mtt_ros_wrapper → mtt_driver → CAN → Vehicle
**Autonomy**: Nav2 Stack → /cmd_vel Topic → mtt_ros_wrapper → mtt_driver → CAN → Vehicle  
**Telemetry**: Vehicle → CAN → mtt_driver → mtt_ros_wrapper → Telemetry & Odometry Topics

### Current Specification Updates

**CAN Bus Compliance:**
- Security switch corrected to bit 7 (0x80) instead of bit 3 (0x08)
- Emergency stop patch: Light control temporarily acts as E-stop
- Tachometer data parsing from CAN 0x2FF (MSB-first byte order)
- Speed calculation using manufacturer-provided gear ratios

**New ROS Topics:**
- `/mtt_tachometer`: Complete tachometer data structure
- `/mtt_speed`: Real-time speed in km/h
- `/mtt_distance`: Cumulative distance traveled
- `/mtt_temperature`: Temperature sensor readings
- `/mtt_odometry`: Standard odometry message for navigation stacks (nav_msgs/Odometry)

**Enhanced Safety:**
- Dual safety mechanism: Security switch + light state validation
- Improved system readiness checks before motion commands
- Better error handling and fault detection

**Message Types:**
- Added `MttTachometerData.msg` for comprehensive telemetry
- Maintains compatibility with standard ROS message types  
