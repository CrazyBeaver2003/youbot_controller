import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    youbot_control_dir = get_package_share_directory('youbot_control')
    rviz_config_path = os.path.join(youbot_control_dir, 'rviz', 'nav_config.rviz')

    # Robot Launch
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(youbot_control_dir, 'launch', 'robot_launch.py')
        )
    )

    # SLAM Launch
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(youbot_control_dir, 'launch', 'slam_launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Navigation Launch (delayed, no map/amcl)
    navigation_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(youbot_control_dir, 'launch', 'nav_only_launch.py')
                ),
                launch_arguments={
                    'use_sim_time':'true',
                    'autostart' : 'true',
                }.items()
            )
        ]
    )

    # Detection Node
    detection_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='youbot_control',
                executable='pt_detection_node',
                name='detection_node',
                output='screen',
                parameters=[
                    {'confidence_threshold': 0.5},
                    {'iou_threshold': 0.4},
                    {'device': 0},
                ]
            )
        ]
    )

    # Object Coordinate Finder
    object_coordinate_finder = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='youbot_control',
                executable='object_coordinate_finder',
                name='object_coordinate_finder',
                output='screen',
                parameters=[
                    {'use_sim_time': True}
                ]
            )
        ]
    )

    # Controllers
    arm_controller = Node(
        package='youbot_control',
        executable='arm_controller',
        name='arm_controller',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ]
    )
    gripper_controller = Node(
        package='youbot_control',
        executable='gripper_controller_node',
        name='gripper_controller',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ]
    )

    # Mission Controller (The Brain)
    mission_controller = TimerAction(
        period=10.0, # Wait for nav and others
        actions=[
            Node(
                package='youbot_control',
                executable='mission_controller',
                name='mission_controller',
                output='screen',
                parameters=[
                    {'use_sim_time': True}
                ]
            )
        ]
    )

    rviz_launch = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        robot_launch,
        slam_launch,
        arm_controller,
        gripper_controller,
        detection_node,
        object_coordinate_finder,
        navigation_launch,
        mission_controller,
        rviz_launch,
    ])
