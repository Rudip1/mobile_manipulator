import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    GroupAction,
)
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # -----------------------------
    # Paths
    # -----------------------------
    pkg_scout_description = get_package_share_directory("scout_description")
    pkg_scout_gazebo = get_package_share_directory("scout_gazebo")

    xacro_file = os.path.join(
        pkg_scout_gazebo, "gazebo_urdf", "scout_full_gazebo.xacro"
    )
    robot_description_content = Command("xacro " + xacro_file)

    rviz_config_file = os.path.join(pkg_scout_description, "rviz", "display.rviz")

    # -----------------------------
    # Environment Variables
    # -----------------------------
    gazebo_model_path = SetEnvironmentVariable(
        "GAZEBO_MODEL_PATH", pkg_scout_description
    )
    gazebo_resource_path = SetEnvironmentVariable(
        "GAZEBO_RESOURCE_PATH", "/usr/share/gazebo-11"
    )

    # -----------------------------
    # Nodes
    # -----------------------------
    # Robot State Publisher
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_content}],
    )

    # Joint State Publisher (optional, for sliders)
    jsp_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
    )

    # Gazebo server
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py"
            )
        ),
        launch_arguments={"verbose": "true"}.items(),
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "scout_v2",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.1",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            gazebo_model_path,
            gazebo_resource_path,
            rsp_node,
            jsp_node,
            gazebo_server,
            spawn_entity,
            rviz_node,
        ]
    )
