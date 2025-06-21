from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    # 1) Include the standard visualize_franka.launch.py from franka_description
    franka_vis = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('franka_description'),
                'launch',
                'visualize_franka.launch.py'
            ])
        ),
        launch_arguments={
            'arm_id': 'fr3',
            'load_gripper': 'false'
        }.items()
    )

    # 2) Grab the robot_description XML from the xacro command used in the FR3 visualization launch
    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([
            get_package_share_directory('franka_description'),
            'robots',
            'fr3',
            'fr3.urdf.xacro'
        ]),
        ' arm_id:=fr3'
    ])

    # 3) Launch your JacobianComputer node, passing in robot_description
    jacobian_node = Node(
        package='my_fr3_control',
        executable='jacobian_computation_node',
        name='jacobian_computer_node',
        output='screen',
        parameters=[{
            # force this to be treated as a string parameter
            'robot_description':
                ParameterValue(robot_description, value_type=str)
        }]
    )

    proj_grad_node = Node(
        package='my_fr3_control',
        executable='projected_gradient_pos',
        name='projected_gradient_controller',
        output='screen',
        parameters=[{
            'T': 3.0,  # Trajectory duration
            'dt': 0.01,  # Control loop period
        }]
    )

    return LaunchDescription([
        franka_vis,
        jacobian_node,
        proj_grad_node
    ])