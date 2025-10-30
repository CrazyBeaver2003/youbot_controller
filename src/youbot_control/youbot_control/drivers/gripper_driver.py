import rclpy
from std_msgs.msg import Float64MultiArray

class GripperDriver:
    def init(self, webots_node, properties):

        self.__robot = webots_node.robot
        self.__left_finger_motor = self.__robot.getDevice('finger::left')
        self.__right_finger_motor = self.__robot.getDevice('finger::right')

        self.__finger_motors = [
            self.__left_finger_motor,
            self.__right_finger_motor
        ]

        for motor in self.__finger_motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)

        self.__left_finger_sensor = self.__robot.getDevice('finger::leftsensor')
        self.__right_finger_sensor = self.__robot.getDevice('finger::rightsensor')
        self.__finger_sensors = [
            self.__left_finger_sensor,
            self.__right_finger_sensor
        ]

        for sensor in self.__finger_sensors:
            sensor.enable(int(self.__robot.getBasicTimeStep()))
        
        self.__target_position = None

        self.__node = rclpy.create_node('gripper_driver_node')
        self.__position_subscriber = self.__node.create_subscription(
            Float64MultiArray,
            'target_position',
            self.__position_callback,
            10)
        self.state_publisher = self.__node.create_publisher(
            Float64MultiArray,
            'gripper_state',
            10)
        
    def __position_callback(self, msg):
        position = msg.data
        self.__node.get_logger().info(f'Setting gripper position: {position}')
        try:
            self.__target_position = position
        except Exception as e:
            self.__node.get_logger().error(f'Error setting gripper position: {e}')
    
    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        #Отправка команд на моторы
        for i, motor in enumerate(self.__finger_motors):
            if self.__target_position is not None:
                motor.setPosition(self.__target_position[i])

        # Публикуем состояние захвата
        positions = {}
        for i, sensor in enumerate(self.__finger_sensors):
            positions[i] = sensor.getValue()

        # 0 - левый палец, 1 - правый палец
        msg = Float64MultiArray()
        msg.data = list(positions.values())
        self.state_publisher.publish(msg)