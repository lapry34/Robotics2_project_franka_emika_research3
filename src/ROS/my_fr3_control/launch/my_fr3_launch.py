from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node



def generate_launch_description():
    # 1) Declare a launch argument for orientation
    
    isRG_v = True
    acceleration_v = True
    circular_v = True
    orientation_v = True

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
    worker_node = Node(
        package='my_fr3_control',
        executable='worker_node',
        name='worker_node',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str),
            'orientation': orientation_v,
            'acceleration': acceleration_v,
        }]
    )

    # 5) Launch the projected gradient orientation controller, using the same orientation argument


    name = 'RG' if isRG_v else 'PG'
    name += '_acc' if acceleration_v else '_vel'
    name += '_ori' if orientation_v else '_pos'
    T = 18.0 if circular_v else 2.5
    dt = 0.005

    if acceleration_v:

        master_node = Node(
            package='my_fr3_control',
            executable='acceleration_controller_node',
            name=name,
            output='screen',
            parameters=[{
                'T': T,  # Trajectory duration
                'dt': dt,  # Control loop period
                'orientation': orientation_v,
                'is_RG': isRG_v,
                'circular': circular_v
            }]
        )

    else:

        master_node = Node(
            package='my_fr3_control',
            executable='velocity_controller_node',
            name=name,
            output='screen',
            parameters=[{
                'T': T,  # Trajectory duration
                'dt': dt,  # Control loop period
                'orientation': orientation_v,
                'is_RG': isRG_v,
                'circular': circular_v
            }]
        )




    return LaunchDescription([

        franka_vis,
        worker_node,
        master_node,
    ])
