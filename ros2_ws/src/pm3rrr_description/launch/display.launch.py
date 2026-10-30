"""Show the robot model in RViz (static home pose).

    ros2 launch pm3rrr_description display.launch.py

For a moving robot use:  ros2 launch pm3rrr_bringup sim_rviz.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc = get_package_share_directory("pm3rrr_description")
    robot_description = ParameterValue(
        Command(["xacro ", os.path.join(desc, "urdf", "pm3rrr.urdf.xacro")]), value_type=str)
    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}]),
        # all joints = 0 is the assembled home pose (see docs/learning/03_cad_to_urdf.md)
        Node(package="joint_state_publisher", executable="joint_state_publisher"),
        Node(package="rviz2", executable="rviz2", arguments=["-d", os.path.join(desc, "rviz", "pm3rrr.rviz")]),
    ])
