"""Kinematic simulation in RViz (no physics) - the fastest way to see the robot move.

    ros2 launch pm3rrr_bringup sim_rviz.launch.py scenario:=figure8
    ros2 launch pm3rrr_bringup sim_rviz.launch.py scenario:=none     # then publish targets yourself:
    ros2 topic pub -1 /pm3rrr/target_pose geometry_msgs/msg/Pose2D "{x: 0.01, y: 0.005, theta: 0.1}"

scenario: figure8 | microplate | contour | rotation_limit | imu | none
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def scenario_nodes(context):
    s = LaunchConfiguration("scenario").perform(context)
    if s == "none":
        return []
    if s == "imu":
        cfg = os.path.join(get_package_share_directory("pm3rrr_control"), "config", "controller.yaml")
        return [Node(package="pm3rrr_control", executable="fake_hand_imu"),
                Node(package="pm3rrr_control", executable="imu_teleop", parameters=[cfg])]
    return [Node(package="pm3rrr_control", executable="scenario", parameters=[{"scenario": s}])]


def generate_launch_description():
    desc = get_package_share_directory("pm3rrr_description")
    cfg = os.path.join(get_package_share_directory("pm3rrr_control"), "config", "controller.yaml")
    robot_description = ParameterValue(
        Command(["xacro ", os.path.join(desc, "urdf", "pm3rrr.urdf.xacro")]), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value="figure8"),
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}]),
        Node(package="pm3rrr_control", executable="controller", output="screen",
             parameters=[cfg, {"output_mode": "kinematic"}]),
        Node(package="rviz2", executable="rviz2", arguments=["-d", os.path.join(desc, "rviz", "pm3rrr.rviz")],
             condition=IfCondition(LaunchConfiguration("rviz"))),
        OpaqueFunction(function=scenario_nodes),
    ])
