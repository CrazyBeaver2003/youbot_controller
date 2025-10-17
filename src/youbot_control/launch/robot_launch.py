import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    pkg_dir = get_package_share_directory('youbot_control')
    robot_description_path = os.path.join(pkg_dir, 'resource', 'youbot.urdf')

    webots = WebotsLauncher(
        world=os.path.join(pkg_dir, 'worlds', 'youbot1.wbt')
    )
    
    my_robot_driver = WebotsController(
        robot_name='my_robot',
        parameters=[
            {'robot_description': robot_description_path},
        ],
        respawn=True
    )

    odometry_publisher = Node(
        package='youbot_control',
        executable='youbot_odometry',
    )
    
    return LaunchDescription([
        webots,
        my_robot_driver,
        odometry_publisher,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            ),
        ),
    ])

