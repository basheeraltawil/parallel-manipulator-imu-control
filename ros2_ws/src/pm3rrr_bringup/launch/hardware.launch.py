"""Real robot: Arduino firmware over USB + controller + RViz.

    ros2 launch pm3rrr_bringup hardware.launch.py port:=/dev/ttyUSB0 scenario:=imu

The hand-held MPU6050 data come from the firmware (I lines) -> imu_teleop.
SAFETY: read docs/industrialization/07_safety_and_compliance.md first.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def scenario_nodes(context):
    s = LaunchConfiguration("scenario").perform(context)
    cfg = os.path.join(get_package_share_directory("pm3rrr_control"), "config", "controller.yaml")
    if s == "none":
        return []
    if s == "imu":
        return [Node(package="pm3rrr_control", executable="imu_teleop", parameters=[cfg])]
    return [Node(package="pm3rrr_control", executable="scenario", parameters=[{"scenario": s, "loop": False}])]


def generate_launch_description():
    desc = get_package_share_directory("pm3rrr_description")
    cfg = os.path.join(get_package_share_directory("pm3rrr_control"), "config", "controller.yaml")
    robot_description = ParameterValue(
        Command(["xacro ", os.path.join(desc, "urdf", "pm3rrr.urdf.xacro")]), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("scenario", default_value="imu"),
        Node(package="pm3rrr_control", executable="serial_bridge", output="screen",
             parameters=[cfg, {"port": LaunchConfiguration("port")}]),
        Node(package="pm3rrr_control", executable="controller", output="screen",
             parameters=[cfg, {"output_mode": "hardware"}]),
        # RViz shows the COMMANDED robot (servos have no position feedback)
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}]),
        Node(package="pm3rrr_control", executable="controller", name="pm3rrr_controller_viz",
             parameters=[cfg, {"output_mode": "kinematic"}]),
        Node(package="rviz2", executable="rviz2", arguments=["-d", os.path.join(desc, "rviz", "pm3rrr.rviz")]),
        OpaqueFunction(function=scenario_nodes),
    ])
