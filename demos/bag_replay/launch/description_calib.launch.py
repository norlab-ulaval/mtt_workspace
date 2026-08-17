"""
Robot description publisher for bag replay with calibration support.

Reads CALIBRATION env var (default: v1) and passes it to xacro so the correct
hesai_lidar_joint origin is used.  Passes the URDF via a Python dict parameter
to avoid shell quoting issues with XML attribute double-quotes.
"""
import os
import re
import subprocess

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    calibration = os.environ.get("CALIBRATION", "v1")
    workspace   = os.environ.get("WORKSPACE", "/home/mtt/ws")

    # Use the source tree directly — always mounted in the container.
    # calib_dir is also passed from source so edited calib_*.xacro files are
    # picked up without an install-tree rebuild (autosync_ws skips install/).
    xacro_path = os.path.join(
        workspace, "src", "mtt_core", "mtt_description", "urdf", "robot.urdf.xacro"
    )
    calib_dir = os.path.join(
        workspace, "src", "mtt_core", "mtt_description", "urdf", "calibrations"
    )

    xacro_result = subprocess.run(
        ["xacro", xacro_path, f"calibration:={calibration}", f"calib_dir:={calib_dir}"],
        capture_output=True,
        text=True,
        check=True,
    )

    # Rewrite file:// mesh paths → package:// so robot_state_publisher can serve them.
    urdf_string = re.sub(
        r'file://[^"]*share/mtt_description/',
        'package://mtt_description/',
        xacro_result.stdout,
    )

    use_sim_time = os.environ.get("USE_SIM_TIME", "true").lower() in ("1", "true", "yes")

    actions = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="both",
            parameters=[
                {"robot_description": urdf_string},
                {"use_sim_time": use_sim_time},
            ],
        )
    ]

    if not use_sim_time:
        # Keep the same live visualization chain that was used by
        # norlab_robot/description.launch.py before description ownership was
        # split into this standalone service.  The runtime builder publishes
        # the measured joints while joint_state_publisher fills every movable
        # URDF joint and exposes the conventional /joint_states topic.
        actions.append(
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="both",
                parameters=[
                    {
                        "rate": 50,
                        "robot_description": urdf_string,
                        "use_sim_time": use_sim_time,
                        "source_list": ["runtime_joint_states"],
                    }
                ],
            )
        )

    return LaunchDescription(actions)
