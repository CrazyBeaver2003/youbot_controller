import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    youbot_control_dir = get_package_share_directory('youbot_control')
    rviz_config_path = os.path.join(youbot_control_dir, 'rviz', 'nav_config.rviz')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Arguments
    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file to load. If empty, SLAM is used.'
    )
    
    map_file = LaunchConfiguration('map')
    
    # Check if map file is provided (not empty)
    # We use PythonExpression to check if map string is not empty
    has_map = PythonExpression(["'", map_file, "' != ''"])
    no_map = PythonExpression(["'", map_file, "' == ''"])

    # Robot Launch
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(youbot_control_dir, 'launch', 'robot_launch.py')
        )
    )

    # SLAM Launch (Only if NO map provided)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(youbot_control_dir, 'launch', 'slam_launch.py')
        ),
        condition=IfCondition(no_map),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Navigation Launch (Only if NO map provided - uses SLAM map)
    navigation_launch_slam = TimerAction(
        period=5.0,
        condition=IfCondition(no_map),
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
    
    # Localization Nodes (Only if MAP provided)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            os.path.join(youbot_control_dir, 'config', 'nav2_params.yaml'),
            {'yaml_filename': map_file},
            {'use_sim_time': True}
        ],
        condition=IfCondition(has_map)
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            os.path.join(youbot_control_dir, 'config', 'nav2_params.yaml'),
            {'use_sim_time': True}
        ],
        condition=IfCondition(has_map)
    )

    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},
            {'node_names': ['map_server', 'amcl']}
        ],
        condition=IfCondition(has_map)
    )

    # Navigation Launch (Reused for both SLAM and Map modes)
    # If Map provided, we launch it alongside localization
    navigation_launch_map = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(youbot_control_dir, 'launch', 'nav_only_launch.py')
        ),
        condition=IfCondition(has_map),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
            'params_file': os.path.join(youbot_control_dir, 'config', 'nav2_params.yaml')
        }.items()
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

    rviz_launch = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        map_file_arg,
        robot_launch,
        slam_launch,
        navigation_launch_slam,
        map_server_node,
        amcl_node,
        lifecycle_manager_localization,
        navigation_launch_map,
        arm_controller,
        gripper_controller,
        detection_node,
        object_coordinate_finder,
        rviz_launch,
    ])
