import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Add the path to the `utils` folder for loading YAML

import yaml


def load_yaml(file_path):
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"File not found: {file_path}")
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)


def generate_robot_nodes(context):
    # Retrieve launch arguments
    controller_name = LaunchConfiguration('controller_name').perform(context)
    config_file = LaunchConfiguration('robot_config_file').perform(context)

    # Load robot configuration YAML
    configs = load_yaml(config_file)
    nodes = []

    for item_name, cfg in configs.items():
        ns = str(cfg.get('namespace', ''))
        # Include Franka core bringup
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('franka_bringup'), 'launch', 'franka.launch.py'
                    ])
                ),
                launch_arguments={
                    'arm_id': str(cfg.get('arm_id', '')),
                    'arm_prefix': str(cfg.get('arm_prefix', '')),
                    'namespace': ns,
                    'urdf_file': str(cfg.get('urdf_file', '')),
                    'robot_ip': str(cfg.get('robot_ip', '')),
                    'use_fake_hardware': str(cfg.get('use_fake_hardware', '')),
                    'fake_sensor_commands': str(cfg.get('fake_sensor_commands', '')),
                    'joint_sources': ','.join(cfg.get('joint_sources', [])),
                    'joint_state_rate': str(cfg.get('joint_state_rate', '')),
                }.items(),
            )
        )
        # Spawn the specified controller with correct parameter file substitution
        nodes.append(
            Node(
                package='controller_manager',
                executable='spawner',
                namespace=ns,
                arguments=[
                    controller_name,
                    '--controller-manager-timeout', '30'
                ],
                parameters=[
                    PathJoinSubstitution([
                        FindPackageShare('franka_bringup'),
                        'config', 'controllers.yaml'
                    ])
                ],
                output='screen',
            )
        )

    # Optionally include RViz
    if any(str(cfg.get('use_rviz', False)).lower() == 'true' for cfg in configs.values()):
        nodes.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=[
                    '--display-config',
                    os.path.join(
                        get_package_share_directory('franka_description'),
                        'rviz', 'visualize_franka.rviz'
                    )
                ],
                output='screen',
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('franka_bringup'), 'config', 'franka.config.yaml'
            ]),
            description='Path to Franka robot YAML config'
        ),
        DeclareLaunchArgument(
            'controller_name',
            default_value='joint_velocity_example_controller',
            description='Controller to spawn'
        ),
        OpaqueFunction(function=generate_robot_nodes)
    ])
