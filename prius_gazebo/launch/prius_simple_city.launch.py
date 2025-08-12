#!/usr/bin/python3

from os.path import join
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import AppendEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)

    this_package_path = get_package_share_directory("prius_gazebo")
    world_name = LaunchConfiguration("world_name", default="simple_city")
    world_file = LaunchConfiguration(
        "world_file", default=[join(this_package_path, "worlds/"), world_name, ".sdf"]
    )
    x_pose = LaunchConfiguration("x_pose", default=-40.0)
    y_pose = LaunchConfiguration("y_pose", default=-1.9)
    z_pose = LaunchConfiguration("z_pose", default=5.5)
    yaw = LaunchConfiguration("yaw", default=1.57)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(this_package_path, "launch", "empty_world.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "world_name": world_name,
            "world_file": world_file,
        }.items(),
    )

    spawn_prius = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(this_package_path, "launch", "spawn_prius.launch.py")
        ),
        launch_arguments={
            "world_name": world_name,
            "x": x_pose,
            "y": y_pose,
            "z": z_pose,
            "yaw": yaw,
        }.items(),
    )

    return LaunchDescription(
        [
            AppendEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH", value=join(this_package_path, "worlds")
            ),
            AppendEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH", value=join(this_package_path, "models")
            ),
            DeclareLaunchArgument("use_sim_time", default_value=use_sim_time),
            DeclareLaunchArgument("world_name", default_value=world_name),
            DeclareLaunchArgument("world_file", default_value=world_file),
            DeclareLaunchArgument("x_pose", default_value=x_pose),
            DeclareLaunchArgument("y_pose", default_value=y_pose),
            DeclareLaunchArgument("z_pose", default_value=z_pose),
            DeclareLaunchArgument("yaw", default_value=yaw),
            gz_sim,
            spawn_prius,
        ]
    )
