import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("prius_viz"), "rviz2", "model_view.rviz"]
    )

    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="prius_viz_model_rviz",
        arguments=["-d", rviz_config_file],
        output="screen",
    )

    launch_prius_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("prius_description"),
                "launch/description.launch.py",
            )
        ),
    )

    return LaunchDescription([
            node_rviz,
            launch_prius_description,
        ]
    )
