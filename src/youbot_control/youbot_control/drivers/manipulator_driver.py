import rclpy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Float64MultiArray
import math

class ManipulatorDriver:
    def init(self, webots_node, properties):

        """
        Initialization function for manipulator driver node.

        Initializes robot, arm motors, and arm sensors.
        Creates a ROS2 node, a transform broadcaster, a joint state publisher,
        and a target position subscriber.

        :param webots_node: WeBOTS node object
        :param properties: Properties of the node
        :return: None
        """
        self.__robot = webots_node.robot

        self.__arm_1_motor = self.__robot.getDevice('arm1')
        self.__arm_2_motor = self.__robot.getDevice('arm2')
        self.__arm_3_motor = self.__robot.getDevice('arm3')
        self.__arm_4_motor = self.__robot.getDevice('arm4')
        self.__arm_5_motor = self.__robot.getDevice('arm5')
        self.__arm_motors = [
            self.__arm_1_motor,
            self.__arm_2_motor,
            self.__arm_3_motor,
            self.__arm_4_motor,
            self.__arm_5_motor
        ]
        for motor in self.__arm_motors:
            # Устанавливаем начальную позицию (home position)
            motor.setPosition(0.0)
            # Устанавливаем скорость для движения к целевой позиции
            motor.setVelocity(1.0)
        self.__arm_1_sensor = self.__robot.getDevice('arm1sensor')
        self.__arm_2_sensor = self.__robot.getDevice('arm2sensor')
        self.__arm_3_sensor = self.__robot.getDevice('arm3sensor')
        self.__arm_4_sensor = self.__robot.getDevice('arm4sensor')
        self.__arm_5_sensor = self.__robot.getDevice('arm5sensor')
        self.__arm_sensors = [
            self.__arm_1_sensor,
            self.__arm_2_sensor,
            self.__arm_3_sensor,
            self.__arm_4_sensor,
            self.__arm_5_sensor
        ]
        for sensor in self.__arm_sensors:
            sensor.enable(int(self.__robot.getBasicTimeStep()))

        self.__node = rclpy.create_node('manipulator_driver_node')

        self.__tf_broadcaster = TransformBroadcaster(self.__node)
        self.__joint_state_publisher = self.__node.create_publisher(
            JointState,
            'arm_joint_states', 
            10)
        self.__target_subscriber = self.__node.create_subscription(
            Float64MultiArray,
            'arm_target_joints_positions',
            self.__target_callback,
            10
        )

        self.__joint_commands = [0.0] * 5
        
    def __target_callback(self, msg):
        try:
            if len(msg.data) == len(self.__arm_motors):
                self.__joint_commands = list(msg.data)
                self.__node.get_logger().info(f'Received arm target: {[f"{x:.3f}" for x in self.__joint_commands]}')
            else:
                self.__node.get_logger().error(f"Received target positions length {len(msg.data)} does not match number of arm motors {len(self.__arm_motors)}.")
        except Exception as e:
            self.__node.get_logger().error(f"Error in target callback: {e}")
    
    def step(self):
        """
        Step function for manipulator driver node.

        Publishes joint state of the manipulator and sends position commands to the arm motors.
        Also sends a transform message with the position of the manipulator relative to base_link.
        """
        rclpy.spin_once(self.__node, timeout_sec=0)

        joint_positions = []
        for sensor in self.__arm_sensors:
            joint_positions.append(sensor.getValue())
        
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.__node.get_clock().now().to_msg()
        joint_state_msg.name = ['arm1', 'arm2', 'arm3', 'arm4', 'arm5']
        joint_state_msg.position = joint_positions

        self.__joint_state_publisher.publish(joint_state_msg)

        for i, motor in enumerate(self.__arm_motors):
            motor.setPosition(self.__joint_commands[i])
        
        transform = TransformStamped()
        transform.header.stamp = self.__node.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'arm_link'
        
        # Позиция манипулятора относительно base_link
        transform.transform.translation.x = 0.16
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.144 - 0.253 # Высота до второй оси вращений минус длина от первой оси до второй

        # Ориентация манипулятора относительно base_link
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        self.__tf_broadcaster.sendTransform(transform)
        
    