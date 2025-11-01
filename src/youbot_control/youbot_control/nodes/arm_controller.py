import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
from youbot_control.nodes.kinematic_solver import KukaYouBotKinematic

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')

        self.kinematics = KukaYouBotKinematic()

        self.joint_names = [
            'arm1',
            'arm2',
            'arm3',
            'arm4',
            'arm5'
        ]

        self.joint_limits = {
            'arm1': (-2.95, 2.95),
            'arm2': (-1.13, 1.13),
            'arm3': (-2.07, 2.07),
            'arm4': (-1.57, 1.57),
            'arm5': (-2.07, 2.07)
        }

        self.current_positions = [0.0] * 5

        # Publishers
        self.arm_target_publisher = self.create_publisher(
            Float64MultiArray,
            'arm_target_positions',
            10
        )

        self.arm_current_pose_publisher = self.create_publisher(
            Point,
            'arm_current_position_point',
            10,
        )

        # Subscribers
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            'arm_joint_states',
            self.joint_state_callback,
            10
        )
        self.arm_target_pose_subscriber = self.create_subscription(
            Pose,
            'arm_target_pose',
            self.arm_target_pose_callback,
            10
        )

        self.get_logger().info("Arm Controller Node Initialized")

    def joint_state_callback(self, msg):
        """Обновляет текущие позиции суставов и публикует позицию схвата."""
        try:
            if set(msg.name) == set(self.joint_names):
                for i, joint_name in enumerate(self.joint_names):
                    idx = msg.name.index(joint_name)
                    self.current_positions[i] = msg.position[idx]
                T = self.kinematics.forward_kinematics(joint_angles=self.current_positions)
                current_position = T[:3, 3]
                msg_point = Point()
                msg_point.x = current_position[0]
                msg_point.y = current_position[1]
                msg_point.z = current_position[2]
                self.arm_current_pose_publisher.publish(msg_point)
        except Exception as e:
            self.get_logger().error(f"Error in joint state callback: {e}")

    def arm_target_pose_callback(self, msg):
        """Обрабатывает целевую позу и вычисляет углы через IK."""
        try:
            x = float(msg.position.x)
            y = float(msg.position.y)
            z = float(msg.position.z)

            qx = msg.orientation.x
            qy = msg.orientation.y
            qz = msg.orientation.z
            qw = msg.orientation.w

            # rotation_matrix = self.quaternion_to_rotation_matrix([qx, qy, qz, qw])

            self.get_logger().info(f'Received target pose: x={x:.3f}, y={y:.3f}, z={z:.3f}')

            #Inverse kinematics solver
            target_position = np.array([x, y, z])
            target_orientation = self.compute_target_orientation(target_position, -np.pi/4)

            target_pose = np.eye(4)
            target_pose[:3, :3] = target_orientation
            target_pose[:3, 3] = target_position
            
            joint_angles, success, error = self.kinematics.inverse_kinematics(target_pose=target_pose)

            if not success:
                self.get_logger().info(f'Численный метод не сошелся. Ошибка: {error}')

            target_positions = []
            for i, (angle, name) in enumerate(zip(joint_angles, self.joint_names)):
                lower_limit, upper_limit = self.joint_limits[name]
                clamped_angle = np.clip(angle, lower_limit, upper_limit)
                target_positions.append(clamped_angle)
            
            self.get_logger().info(f'Sending target joint positions: {target_positions}')
            self.send_target_positions(target_positions)

        except Exception as e:
            self.get_logger().error(f"Error in arm target pose callback: {e}")

    def send_target_positions(self, positions):
        """Публикует целевые углы суставов."""
        try:
            msg = Float64MultiArray()
            msg.data = positions
            self.arm_target_publisher.publish(msg)
            self.get_logger().info(f'Sent target joint positions: {positions}')
        except Exception as e:
            self.get_logger().error(f"Error in send_target_positions: {e}")


    def quaternion_to_rotation_matrix(self, q):
        """
        Конвертирует кватернион в матрицу поворота.
        
        Args:
            q: [x, y, z, w] кватернион
        Returns:
            3x3 матрица поворота
        """
        x, y, z, w = q

        norm = np.sqrt(x*x + y*y + z*z + w*w)
        if norm > 0:
            x, y, z, w = x/norm, y/norm, z/norm, w/norm

        # Calculate rotation matrix elements
        xx, yy, zz = x*x, y*y, z*z
        xy, xz, yz = x*y, x*z, y*z
        wx, wy, wz = w*x, w*y, w*z

        rotation_matrix = np.array([
            [1 - 2*(yy + zz),     2*(xy - wz),       2*(xz + wy)],
            [2*(xy + wz),         1 - 2*(xx + zz),   2*(yz - wx)],
            [2*(xz - wy),         2*(yz + wx),       1 - 2*(xx + yy)]
        ])
        return rotation_matrix
        
    def compute_target_orientation(self, position, wrist_angle):

        # New z axis direct from zero to target position
        z_axis = np.array(position)
        z_axis[2] = 0

        # Normalize z axis
        if np.linalg.norm(z_axis) < 1e-6:
            z_axis = np.array([0, 0, 1])
        else:
            z_axis = z_axis / np.linalg.norm(z_axis)
        
        x_axis = np.array([0.0, 0.0, 1.0])

        if abs(np.dot(x_axis, z_axis)) > 0.99:
            # If x axis is already parallel to z axis, choose a different axis
            x_axis = np.array([1.0, 0.0, 0.0])
        
        y_axis = np.cross(z_axis, x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)

        rotation_matrix = np.column_stack([x_axis, y_axis, z_axis])

        # Apply wrist rotation around y axis
        if abs(wrist_angle) > 1e-6:
            cos_a = np.cos(wrist_angle)
            sin_a = np.sin(wrist_angle)
            Ry = np.array([
                [cos_a, 0, sin_a],
                [0, 1, 0],
                [-sin_a, 0, cos_a]
            ])
            rotation_matrix = rotation_matrix @ Ry
        return rotation_matrix

def main(args=None):
    rclpy.init(args=args)
    arm_controller = None
    
    try:
        arm_controller = ArmController()
        rclpy.spin(arm_controller)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        # ✅ Безопасная обработка ошибок
        if arm_controller:
            arm_controller.get_logger().error(f"Error in main: {e}")
        else:
            print(f"Error initializing ArmController: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if arm_controller:
            arm_controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()


