from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    # 1) Declare a launch argument for orientation
    declare_orientation_arg = DeclareLaunchArgument(
        'orientation',
        default_value='false',
        description='Whether to compute orientation (true or false)'
    )

    declare_acceleration_arg = DeclareLaunchArgument(
        'acceleration',
        default_value='true',
        description='Whether to compute pg on acceleration (true or false)'
    )

    # 2) Include the standard visualize_franka launch
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

    # 3) Generate the robot_description from xacro
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

    # 4) Launch the JacobianComputer node, passing orientation from launch argument
    jacobian_node = Node(
        package='my_fr3_control',
        executable='jacobian_computation_node',
        name='jacobian_computer_node',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str),
            'orientation': ParameterValue(LaunchConfiguration('orientation'), value_type=bool),
            'acceleration': ParameterValue(LaunchConfiguration('acceleration'), value_type=bool),
        }]
    )

    # 5) Launch the projected gradient orientation controller, using the same orientation argument

    if LaunchConfiguration('acceleration') == 'true':

        proj_grad_node = Node(
            package='my_fr3_control',
            executable='projected_gradient_acc_ori',
            name='projected_gradient_acc_ori_controller',
            output='screen',
            parameters=[{
                'T': 3.0,  # Trajectory duration
                'dt': 0.01,  # Control loop period
                'orientation': ParameterValue(LaunchConfiguration('orientation'), value_type=bool),
            }]
        )

    else:

        proj_grad_node = Node(
            package='my_fr3_control',
            executable='projected_gradient_ori',
            name='projected_gradient_ori_controller',
            output='screen',
            parameters=[{
                'T': 3.0,  # Trajectory duration
                'dt': 0.01,  # Control loop period
                'orientation': ParameterValue(LaunchConfiguration('orientation'), value_type=bool),
            }]
        )
    
    return LaunchDescription([
        declare_orientation_arg,
        declare_acceleration_arg,
        franka_vis,
        jacobian_node,
        proj_grad_node
    ])
