"""Gazebo Classic simulation of the 3-RRR robot with ros2_control.

    ros2 launch pm3rrr_gazebo gazebo.launch.py [gui:=true]

Starts: gzserver(+gzclient), robot_state_publisher (URDF with loop closures),
spawns the robot (assembled: all joints 0 = home pose), then the
joint_state_broadcaster and the effort-based joint_trajectory_controller.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc = get_package_share_directory("pm3rrr_description")
    gz = get_package_share_directory("pm3rrr_gazebo")
    xacro_file = os.path.join(desc, "urdf", "pm3rrr.urdf.xacro")
    robot_description = ParameterValue(
        Command(["xacro ", xacro_file, " use_gazebo:=true use_ros2_control:=true"]), value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")),
        launch_arguments={"world": os.path.join(gz, "worlds", "pm3rrr_table.world"),
                          "gui": LaunchConfiguration("gui"), "pause": "false"}.items())

    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
               parameters=[{"robot_description": robot_description, "use_sim_time": True}])
    spawn = Node(package="gazebo_ros", executable="spawn_entity.py", output="screen",
                 arguments=["-topic", "robot_description", "-entity", "pm3rrr"])
    jsb = Node(package="controller_manager", executable="spawner",
               arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"])
    jtc = Node(package="controller_manager", executable="spawner",
               arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"])

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", description="start gzclient"),
        gazebo, rsp, spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[jtc])),
    ])
