#!/usr/bin/python3

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.substitutions.command import Command
from launch.substitutions.find_executable import FindExecutable
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    EqualsSubstitution,
    NotEqualsSubstitution,
    TextSubstitution,
    PythonExpression,
)
from launch_ros.descriptions import ParameterFile
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml, ReplaceString

def generate_launch_description():
    this_package_path = get_package_share_directory('prius_gazebo')
    prius_description = get_package_share_directory('prius_description')
    params_file = os.path.join(this_package_path, 'configs', 'prius_bridge.yaml')

    namespace = LaunchConfiguration('namespace')
    robotname = LaunchConfiguration('robotname')
    world_name = LaunchConfiguration('world_name')
    robot_sdf = LaunchConfiguration('robot_sdf')
    pose = {'x': LaunchConfiguration('x_pose', default='0.00'),
            'y': LaunchConfiguration('y_pose', default='0.00'),
            'z': LaunchConfiguration('z_pose', default='0.00'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='/',
        description='Top-level namespace')

    declare_world_name_cmd = DeclareLaunchArgument(
        'world_name',
        default_value='',
        description='gz world name')

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robotname',
        default_value='prius',
        description='name of the robot')

    declare_robot_sdf_cmd = DeclareLaunchArgument(
        'robot_sdf',
        default_value=os.path.join(prius_description, 'urdf', 'prius', 'prius.sdf.xacro'),
        description='Full path to robot sdf file to spawn the robot in gazebo')

    declare_robot_x_pose_cmd = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='x position of the robot')

    declare_robot_y_pose_cmd = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='y position of the robot')

    declare_robot_z_pose_cmd = DeclareLaunchArgument(
        'z_pose',
        default_value='0.0',
        description='z position of the robot')

    declare_robot_roll_cmd = DeclareLaunchArgument(
        'roll',
        default_value='0.0',
        description='roll position of the robot')

    declare_robot_pitch_cmd = DeclareLaunchArgument(
        'pitch',
        default_value='0.0',
        description='pitch position of the robot')

    declare_robot_yaw_cmd = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='yaw position of the robot')

    # Place holder
    param_substitutions = []

    params_file = ReplaceString(
        source_file=params_file,
        replacements={"<namespace>": ("")},
        condition=IfCondition(
            EqualsSubstitution(namespace, "/")
        ),
    )

    params_file = ReplaceString(
        source_file=params_file,
        replacements={"<namespace>": (namespace, "/")},
        condition=IfCondition(
            NotEqualsSubstitution(namespace, "/")
        ),
    )

    params_file = ReplaceString(
        source_file=params_file,
        replacements={"<world_name>": (world_name)},
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        parameters=[
            {
                'config_file': params_file,
                'expand_gz_topic_names': True,
                'use_sim_time': True,
            }
        ],
        output='screen',
    )

    spawn_model = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        namespace=namespace,
        arguments=[
            '-name', robotname,
            '-string', Command([
            FindExecutable(name='xacro'), ' ', 'namespace:=',
            LaunchConfiguration('namespace'), ' ', robot_sdf]),
            'use_rviz:=', ' ', 'false',
            '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
            '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']]
    )

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': True,
            # 'frame_prefix': [namespace, TextSubstitution(text="/")],
            'robot_description': Command(
                [
                    "xacro ", robot_sdf,
                    " namespace:=", namespace,
                    " use_rviz:=", "true",
                ]
            ),
        }
    ],
    )

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(prius_description, 'models'))
    set_env_vars_resources2 = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        str(Path(os.path.join(this_package_path)).parent.resolve()))

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_world_name_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_robot_sdf_cmd)
    ld.add_action(declare_robot_x_pose_cmd)
    ld.add_action(declare_robot_y_pose_cmd)
    ld.add_action(declare_robot_z_pose_cmd)
    ld.add_action(declare_robot_roll_cmd)
    ld.add_action(declare_robot_pitch_cmd)
    ld.add_action(declare_robot_yaw_cmd)

    ld.add_action(set_env_vars_resources)
    ld.add_action(set_env_vars_resources2)

    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(spawn_model)
    ld.add_action(bridge)
    return ld
