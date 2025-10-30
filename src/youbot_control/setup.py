from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'youbot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.wbt')),
        ('share/' + package_name + '/resource', ['resource/youbot.urdf']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/maps', glob('maps/*')),
        ('share/' + package_name + '/models', glob('models/*')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='timoha',
    maintainer_email='timagadenov1@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'youbot_driver = youbot_control.drivers.youbot_driver:main',
            
            'youbot_odometry = youbot_control.nodes.youbot_odometry:main',
            'goal_send_node = youbot_control.nodes.goal_send_node:main',

            'onnx_detection_node = youbot_control.detection.onnx_executor:main',
            'pt_detection_node = youbot_control.detection.pt_detection_node:main',
            'object_coordinate_finder = youbot_control.detection.object_coordinate_finder:main',
            
            'sensor_test = youbot_control.tests.sensor_test:main',
        ],
    },
)
