import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Получаем пути к пакетам
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    youbot_control_dir = get_package_share_directory('youbot_control')
    
    # Пути к файлам конфигурации
    params_file = os.path.join(youbot_control_dir, 'config', 'nav2_params_std.yaml')
    map_file = os.path.join(youbot_control_dir, 'maps', 'map.yaml')  # Замените на ваш файл карты
    
    # Аргументы запуска
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    
    return LaunchDescription([
        # Объявление аргументов
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'),
        
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'),
        
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Full path to the ROS2 parameters file'),
        
        DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description='Full path to map yaml file'),
        
        # Включаем стандартный bringup Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': autostart,
            }.items()
        ),
    ])