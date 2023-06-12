from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, ExecuteProcess
import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('robot_description')
    config_file = os.path.join(share_dir, 'config', 'omni_controller.yaml')
    xacro_file = os.path.join(share_dir, 'urdf', 'robot.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()
    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf}
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'pause': 'false'
        }.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'robot',
            '-topic', 'robot_description'
        ],
        output='screen'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    control_node = Node(
        package='omni_control',
        executable='omni_control_node',
        output = 'screen'
    )

    cntrl_mngr = Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'robot_description': robot_urdf}
                        ,config_file],
            name='ros2_control_node',
        )
    
    cntrl_spnr =  Node(
            package='controller_manager',
            executable='spawner',
            name='controller_spawner',
            arguments=['omni_control_node', '--controller-manager', 'ros2_control_node'],
            output='screen'
        )
    
    # cntrl_spnr2 = Node(
    #         package='controller_manager',
    #         executable='controller_manager',
    #         name='controller_manager',
    #         output='screen'
    #     )
    
    return LaunchDescription([
        gazebo_server,
        gazebo_client,
        robot_state_publisher_node,
        joint_state_publisher_node,
        urdf_spawn_node,
        rviz_node,
        control_node,
        # cntrl_mngr,
        # cntrl_spnr,
        # cntrl_spnr2,
    ])