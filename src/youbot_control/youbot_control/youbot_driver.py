import rclpy
from geometry_msgs.msg import Twist

class YouBotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__wheel_1 = self.__robot.getDevice('wheel1')
        self.__wheel_2 = self.__robot.getDevice('wheel2')
        self.__wheel_3 = self.__robot.getDevice('wheel3')
        self.__wheel_4 = self.__robot.getDevice('wheel4')

        self.__wheel_1.setPosition(float('inf'))
        self.__wheel_1.setVelocity(0.0)
        self.__wheel_2.setPosition(float('inf'))
        self.__wheel_2.setVelocity(0.0)
        self.__wheel_3.setPosition(float('inf'))
        self.__wheel_3.setVelocity(0.0)
        self.__wheel_4.setPosition(float('inf'))
        self.__wheel_4.setVelocity(0.0)

        self.__wheel_radius = 0.05

        self.__lx = 0.228  # половина расстояния между колесами по X
        self.__ly = 0.158  # половина расстояния между колесами по Y

        self.__target_twist = Twist()

        # Создаем собственный ROS2 узел для подписки на топик
        rclpy.init(args=None)
        self.__node = rclpy.create_node('youbot_driver_node')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
    
    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        # Обрабатываем ROS2 события
        rclpy.spin_once(self.__node, timeout_sec=0)

        linear_x = self.__target_twist.linear.x
        linear_y = -self.__target_twist.linear.y
        angular_z = -self.__target_twist.angular.z

        command_wheel_1 = (linear_x - linear_y - (self.__lx + self.__ly) * angular_z) / self.__wheel_radius
        command_wheel_2 = (linear_x + linear_y + (self.__lx + self.__ly) * angular_z) / self.__wheel_radius
        command_wheel_3 = (linear_x + linear_y - (self.__lx + self.__ly) * angular_z) / self.__wheel_radius
        command_wheel_4 = (linear_x - linear_y + (self.__lx + self.__ly) * angular_z) / self.__wheel_radius

        self.__wheel_1.setVelocity(command_wheel_1)
        self.__wheel_2.setVelocity(command_wheel_2)
        self.__wheel_3.setVelocity(command_wheel_3)
        self.__wheel_4.setVelocity(command_wheel_4)
