import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import numpy as np

class GripperControllerNode(Node):
    def __init__(self):
        super().__init__('gripper_controller')

        self.MIN_POS = 0.0
        self.MAX_POS = 0.025
        self.OFFSET_WHEN_LOCKED = 0.021
        self.MAX_GAP = 2 * self.MAX_POS + self.OFFSET_WHEN_LOCKED
        self.MIN_GAP = 2 * self.MIN_POS + self.OFFSET_WHEN_LOCKED

        self.current_gap = self.MAX_GAP
        self.target_gap = self.MAX_GAP

        self.gripper_gap_pub = self.create_publisher(
            Float64,
            'gripper_current_gap',
            10
        )

        self.gripper_pos_pub = self.create_publisher(
            Float64,
            'gripper_target_position',
            10
        )
        self.state_subscriber = self.create_subscription(
            Float64,
            'gripper_state',
            self.state_callback,
            10
        )
        self.target_gap_subscriber = self.create_subscription(
            Float64,
            'gripper_target_gap',
            self.target_gap_callback,
            10
        )
        self.get_logger().info('Gripper Controller initialized successfully')
    
    def target_gap_callback(self, msg):
        self.target_gap = msg.data
        self.target_gap = np.clip(self.target_gap, self.MIN_GAP, self.MAX_GAP)
        self.get_logger().info(f'Received target gap: {self.target_gap}')

        # Преобразуем зазор в позицию как в оригинальном коде
        v = np.clip((self.target_gap - self.OFFSET_WHEN_LOCKED) / 2.0, self.MIN_POS, self.MAX_POS)
        position_msg = Float64()
        position_msg.data = v
        self.gripper_pos_pub.publish(position_msg)


    def state_callback(self, msg):
        position = np.clip(msg.data, self.MIN_POS, self.MAX_POS)
        self.current_gap = 2 * position + self.OFFSET_WHEN_LOCKED
        gap_msg = Float64()
        gap_msg.data = self.current_gap
        self.gripper_gap_pub.publish(gap_msg)

def main(args=None):
    rclpy.init(args=args)
    gripper_controller_node = None
    try:
        gripper_controller_node = GripperControllerNode()
        rclpy.spin(gripper_controller_node)
    except Exception as e:
        print(f'Error occurred: {e}')
    finally:
        if gripper_controller_node:
            gripper_controller_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()