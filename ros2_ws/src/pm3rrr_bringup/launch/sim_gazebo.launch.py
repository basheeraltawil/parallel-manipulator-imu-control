"""Physics simulation: Gazebo + ros2_control + controller + RViz + scenario.

    ros2 launch pm3rrr_bringup sim_gazebo.launch.py scenario:=microplate

scenario: figure8 | microplate | contour | rotation_limit | imu | none
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def scenario_nodes(context):
    s = LaunchConfiguration("scenario").perform(context)
    sim = {"use_sim_time": True}
    if s == "none":
        return []
    if s == "imu":
        cfg = os.path.join(get_package_share_directory("pm3rrr_control"), "config", "controller.yaml")
        return [Node(package="pm3rrr_control", executable="fake_hand_imu", parameters=[sim]),
                Node(package="pm3rrr_control", executable="imu_teleop", parameters=[cfg, sim])]
    return [Node(package="pm3rrr_control", executable="scenario", parameters=[{"scenario": s}, sim])]


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("pm3rrr_control"), "config", "controller.yaml")
    rviz_cfg = os.path.join(get_package_share_directory("pm3rrr_description"), "rviz", "pm3rrr.rviz")
    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory("pm3rrr_gazebo"), "launch", "gazebo.launch.py")),
        launch_arguments={"gui": LaunchConfiguration("gui")}.items())
    sim = {"use_sim_time": True}
    nodes = [
        Node(package="pm3rrr_control", executable="controller", output="screen",
             parameters=[cfg, {"output_mode": "gazebo"}, sim]),
        Node(package="pm3rrr_control", executable="fk_monitor", parameters=[sim]),
        Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_cfg], parameters=[sim],
             condition=IfCondition(LaunchConfiguration("rviz"))),
        OpaqueFunction(function=scenario_nodes),
    ]
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value="figure8"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        gazebo,
        TimerAction(period=6.0, actions=nodes),     # wait for the controllers to be active
    ])
