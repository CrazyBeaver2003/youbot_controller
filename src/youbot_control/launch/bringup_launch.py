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

    # Лаунч робота (запускается сразу)
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(youbot_control_dir, 'launch', 'robot_launch.py')
        )
    )

    # Лаунч навигации (запускается с задержкой 5 секунд)
    navigation_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(youbot_control_dir, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time':'true',
                    'autostart' : 'true',
                }.items()
            )
        ]
    )

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
                    {'device': 0},  # 0 для GPU, 'cpu' для CPU
                ]
            )
        ]
    )

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

    goal_send_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='youbot_control',
                executable='goal_send_node',
                name='goal_send_node',
                output='screen'
            )
        ]
    )
    
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
    pickup_action_server = Node(
        package='youbot_control',
        executable='pickup_action_server',
        name='pickup_action_server',
        output='screen',
        parameters=[
            {'use_sim_time': True}
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
        # Сразу запускаем
        robot_launch,
        arm_controller,
        gripper_controller,
        pickup_action_server,
        # С задержками
        detection_node,
        object_coordinate_finder,
        navigation_launch,
        goal_send_node,
        rviz_launch,
    ])