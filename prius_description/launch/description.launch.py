#!/usr/bin/python3

from os.path import join

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    this_package_path = get_package_share_directory("prius_description")
    namespace = LaunchConfiguration("namespace")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {
                "robot_description": Command(
                    [
                        "xacro ",
                        join(this_package_path, "urdf/prius/prius.sdf.xacro"),
                        " use_rviz:=",
                        "true",
                        " namespace:=",
                        namespace,
                    ]
                )
            }
        ],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="/"),
            robot_state_publisher,
            joint_state_publisher,
        ]
    )
