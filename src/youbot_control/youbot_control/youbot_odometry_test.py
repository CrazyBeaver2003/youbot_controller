import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math


class YouBotOdometry:
    def __init__(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())

        # Initialize GPS and IMU sensors
        self.__gps = self.__robot.getDevice('gps')
        self.__gps.enable(self.__timestep)
        
        self.__imu = self.__robot.getDevice('inertial unit')
        self.__imu.enable(self.__timestep)
        
        self.__gyro = self.__robot.getDevice('gyro')
        self.__gyro.enable(self.__timestep)

        # Initialize ROS2 node
        rclpy.init(args=None)
        self.__node = rclpy.create_node('youbot_odometry_node')
        
        # Create odometry publisher
        self.__odom_publisher = self.__node.create_publisher(Odometry, 'odom', 10)
        
        # Create TF broadcaster
        self.__tf_broadcaster = TransformBroadcaster(self.__node)
        
        # Initial position
        self.__prev_x = 0.0
        self.__prev_y = 0.0
        self.__prev_z = 0.0

    def step(self):
        # Process ROS2 events
        rclpy.spin_once(self.__node, timeout_sec=0)
        
        # Get GPS position
        gps_values = self.__gps.getValues()
        x = gps_values[0]
        y = gps_values[1]
        z = gps_values[2]
        
        # Get IMU orientation (roll, pitch, yaw)
        imu_values = self.__imu.getRollPitchYaw()
        roll = imu_values[0]
        pitch = imu_values[1]
        yaw = imu_values[2]
        
        # Get gyroscope angular velocities
        gyro_values = self.__gyro.getValues()
        angular_x = gyro_values[0]
        angular_y = gyro_values[1]
        angular_z = gyro_values[2]
        
        # Calculate linear velocities
        dt = self.__timestep / 1000.0
        vx = (x - self.__prev_x) / dt if dt > 0 else 0.0
        vy = (y - self.__prev_y) / dt if dt > 0 else 0.0
        vz = (z - self.__prev_z) / dt if dt > 0 else 0.0
        
        # Update previous position
        self.__prev_x = x
        self.__prev_y = y
        self.__prev_z = z
        
        # Create and publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.__node.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Set position
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = z
        
        # Convert roll, pitch, yaw to quaternion
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        
        # Set velocities
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.linear.z = vz
        odom_msg.twist.twist.angular.x = angular_x
        odom_msg.twist.twist.angular.y = angular_y
        odom_msg.twist.twist.angular.z = angular_z
        
        self.__odom_publisher.publish(odom_msg)
        
        # Broadcast TF transform
        tf_msg = TransformStamped()
        tf_msg.header.stamp = odom_msg.header.stamp
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_link'
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = z
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        
        self.__tf_broadcaster.sendTransform(tf_msg)