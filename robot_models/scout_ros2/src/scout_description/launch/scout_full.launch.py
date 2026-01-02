import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 1. Path to your Xacro file
    pkg_path = get_package_share_directory("scout_description")
    xacro_file = os.path.join(pkg_path, "urdf", "scout_full.xacro")

    # 2. Convert Xacro to XML Robot Description
    robot_description_content = Command("xacro " + xacro_file)

    # 3. Path to your RViz config (the one you moved earlier)
    rviz_config_file = os.path.join(pkg_path, "rviz", "display.rviz")

    return LaunchDescription(
        [
            # Robot State Publisher Node (Processes URDF)
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description_content}],
            ),
            # Joint State Publisher GUI (Gives you sliders to move wheels)
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
            ),
            # RViz2 Node
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config_file],
                output="screen",
            ),
        ]
    )
