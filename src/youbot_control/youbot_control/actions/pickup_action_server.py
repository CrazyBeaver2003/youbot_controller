import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import Pose, Point, PoseStamped, PointStamped
from sensor_msgs.msg import JointState
import numpy as np
from youbot_interfaces.action import PickupObject
from youbot_interfaces.srv import NavigateToObject
from youbot_interfaces.msg import BoundingBoxArray
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from nav2_msgs.action import NavigateToPose
from enum import Enum
import math

class PickupState(Enum):
    MOVING_TO_OBJECT = 1
    OPENING_GRIPPER = 2
    MOVING_TO_TARGET = 3
    CLOSING_GRIPPER = 4
    LIFTING_OBJECT = 5
    MOVING_TO_RELEASE_POINT = 6
    MOVING_ARM_TO_RELEASE = 7
    RELEASING_OBJECT = 8
    LIFTING_ARM_AFTER_RELEASE = 9
    MOVING_TO_HOME = 10
    COMPLETED = 11

class PickupAction(Node):
    def __init__(self):
        super().__init__('pickup_action_server')

        self._action_server = ActionServer(
            self,
            PickupObject,
            'pickup_object',
            self.execute_callback,
        )
        
        # Service Client для навигации к объекту
        self.navigate_client = self.create_client(
            NavigateToObject,
            'navigate_to_object'
        )
        
        # Action Client для отслеживания навигации
        self.nav_action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        
        # Subscribers
        self.gripper_current_point_subscriber = self.create_subscription(
            Point,
            'arm_current_position_point',
            self.gripper_current_point_callback,
            10
        )
        self.gripper_gap_subscriber = self.create_subscription(
            Float64,
            'gripper_current_gap',
            self.gripper_gap_callback,
            10
        )
        
        # Подписка на обнаруженные объекты
        self.detected_objects_subscriber = self.create_subscription(
            BoundingBoxArray,
            '/detected_objects',
            self.detected_objects_callback,
            10
        )
        
        # Подписка на координаты объектов (от object_coordinate_finder)
        self.object_coordinates_subscriber = self.create_subscription(
            PointStamped,
            '/object_coordinates',
            self.object_coordinates_callback,
            10
        )

        # Publishers
        self.arm_target_publisher = self.create_publisher(
            Pose,
            'arm_target_pose',
            10
        )

        self.gripper_target_gap_publisher = self.create_publisher(
            Float64,
            'gripper_target_gap',
            10
        )

        # Constants
        self.MIN_GAP = 0.021
        self.GRIPPER_OPEN_GAP = 0.071
        self.GRIPPER_CLOSED_GAP = 0.035
        self.POSITION_TOLERANCE = 0.02
        self.GRIPPER_TOLERANCE = 0.005
        self.YOUBOT_PLATE_POINT = {'x': -0.3, 'y': 0.0, 'z': 0.18}
        self.YOUBOT_RELEASE_POINT = {'x': 0.4, 'y': 0.0, 'z': 0.22}
        
        # Смещение манипулятора относительно base_link (из manipulator_driver)
        self.ARM_LINK_OFFSET = {
            'x': 0.16,
            'y': 0.0,
            'z': 0.144 - 0.253  # -0.109
        }
        
        # State variables
        self.current_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.current_gripper_gap = 0.0
        
        # Navigation state
        self.navigation_goal_reached = False
        self.navigation_in_progress = False
        self.nav_goal_handle = None
        
        # Object detection state
        self.detected_objects = []
        self.target_object_visible = False
        self.target_object_name = ""
        self.object_coordinates = None  # Координаты объекта в base_link
        
        self.get_logger().info('Pickup Object Action Server запущен')
    
    def navigation_feedback_callback(self, feedback_msg):
        """Callback для feedback от навигации"""
        feedback = feedback_msg.feedback
        # Можно логировать прогресс навигации
        # self.get_logger().info(f'Навигация: расстояние до цели = {feedback.distance_remaining:.2f}м')
    
    def navigation_done_callback(self, future):
        """Callback при завершении навигации"""
        status = future.result().status
        if status == 4:  # SUCCEEDED
            self.get_logger().info('✓ Навигационная цель достигнута!')
            self.navigation_goal_reached = True
            self.navigation_in_progress = False
        elif status == 5:  # CANCELED
            self.get_logger().warn('⚠ Навигация отменена')
            self.navigation_in_progress = False
        elif status == 6:  # ABORTED
            self.get_logger().error('✗ Навигация прервана')
            self.navigation_in_progress = False
    
    def send_navigation_goal(self, object_name):
        """Отправка навигационной цели к объекту через NavigateToPose action"""
        if not self.navigate_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Сервис navigate_to_object недоступен!')
            return False
        
        request = NavigateToObject.Request()
        request.waypoint_name = object_name
        
        self.get_logger().info(f'Получение координат объекта: {object_name}')
        
        future = self.navigate_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                # Отправляем NavigateToPose action
                if not self.nav_action_client.wait_for_server(timeout_sec=5.0):
                    self.get_logger().error('NavigateToPose action server недоступен!')
                    return False
                
                goal_msg = NavigateToPose.Goal()
                goal_msg.pose = response.pose
                
                self.get_logger().info(f'Отправка навигационной цели к {object_name}')
                self.navigation_in_progress = True
                self.navigation_goal_reached = False
                
                # Отправляем goal с callback'ами
                send_goal_future = self.nav_action_client.send_goal_async(
                    goal_msg,
                    feedback_callback=self.navigation_feedback_callback
                )
                send_goal_future.add_done_callback(self.navigation_goal_response_callback)
                
                return True
            else:
                self.get_logger().error(f'✗ Ошибка получения координат: {response.message}')
                return False
        else:
            self.get_logger().error('Не удалось вызвать сервис navigate_to_object')
            return False
    
    def navigation_goal_response_callback(self, future):
        """Callback при принятии/отклонении навигационной цели"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Навигационная цель отклонена!')
            self.navigation_in_progress = False
            return
        
        self.get_logger().info('Навигационная цель принята')
        self.nav_goal_handle = goal_handle
        
        # Запрашиваем результат
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_done_callback)
    
    def check_navigation_goal_reached(self):
        """Проверка достижения навигационной цели"""
        return self.navigation_goal_reached
    
    def check_object_visible(self, object_name):
        """
        Проверка видимости объекта по имени
        
        Args:
            object_name: имя объекта для поиска (например, 'ball', 'apple', 'duck')
        
        Returns:
            bool: True если объект виден
        """
        # Ищем объект в списке обнаруженных
        for bbox in self.detected_objects:
            # Сравниваем имя класса (приводим к нижнему регистру для надежности)
            if bbox.class_name.lower() == object_name.lower():
                self.get_logger().info(
                    f'✓ Объект "{object_name}" обнаружен с уверенностью {bbox.confidence:.2f}'
                )
                return True
        
        return False
    
    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Получена цель захвата объекта: {goal_handle.request.object_name}')
        
        # Сбрасываем все флаги состояний перед началом нового задания
        if hasattr(self, '_coords_warning_shown'):
            delattr(self, '_coords_warning_shown')
        if hasattr(self, '_gripper_closing_logged'):
            delattr(self, '_gripper_closing_logged')
        if hasattr(self, '_lifting_logged'):
            delattr(self, '_lifting_logged')
        if hasattr(self, '_release_navigation_started'):
            delattr(self, '_release_navigation_started')
        if hasattr(self, '_moving_to_release_logged'):
            delattr(self, '_moving_to_release_logged')
        if hasattr(self, '_gripper_opening_logged'):
            delattr(self, '_gripper_opening_logged')
        if hasattr(self, '_lifting_after_release_logged'):
            delattr(self, '_lifting_after_release_logged')
        if hasattr(self, '_home_navigation_started'):
            delattr(self, '_home_navigation_started')

        feedback_msg = PickupObject.Feedback()
        object_name = goal_handle.request.object_name
        
        # Отправка навигационной цели к объекту
        if not self.send_navigation_goal(object_name):
            goal_handle.abort()
            result = PickupObject.Result()
            result.success = False
            result.message = f'Не удалось начать навигацию к объекту {object_name}'
            result.final_gap = 0.0
            return result

        # TODO: Остальная логика состояний
        current_state = PickupState.MOVING_TO_OBJECT

        while current_state != PickupState.COMPLETED:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Захват объекта отменен')
                result = PickupObject.Result()
                result.success = False
                result.message = 'Захват объекта отменен'
                result.final_gap = 0.0
                return result

            # Выполнение состояний
            if current_state == PickupState.MOVING_TO_OBJECT:
                # Обновляем feedback
                feedback_msg.current_state = '[1/10] MOVING_TO_OBJECT'
                feedback_msg.status_message = f'Движение к объекту {object_name}...'
                feedback_msg.current_x = self.current_position['x']
                feedback_msg.current_y = self.current_position['y']
                feedback_msg.current_z = self.current_position['z']
                
                # Проверяем достижение навигационной цели
                if self.check_navigation_goal_reached():
                    self.get_logger().info('✓ Переход к следующему состоянию: OPENING_GRIPPER')
                    current_state = PickupState.OPENING_GRIPPER
            
            elif current_state == PickupState.OPENING_GRIPPER:
                # Обновляем feedback
                feedback_msg.current_state = '[2/10] OPENING_GRIPPER'
                feedback_msg.status_message = f'Проверка видимости объекта {object_name}...'
                feedback_msg.current_x = self.current_position['x']
                feedback_msg.current_y = self.current_position['y']
                feedback_msg.current_z = self.current_position['z']
                
                # Проверяем видимость объекта
                if self.check_object_visible(object_name):
                    # Объект виден - открываем захват
                    self.get_logger().info(f'✓ Объект {object_name} виден, открываю захват')
                    gap_msg = Float64()
                    gap_msg.data = self.GRIPPER_OPEN_GAP
                    self.gripper_target_gap_publisher.publish(gap_msg)
                    
                    # Проверяем, открылся ли захват
                    if abs(self.current_gripper_gap - self.GRIPPER_OPEN_GAP) < self.GRIPPER_TOLERANCE:
                        self.get_logger().info('✓ Захват открыт')
                        current_state = PickupState.MOVING_TO_TARGET
                else:
                    self.get_logger().warn(f'⚠ Объект {object_name} не виден, ожидание...')
            
            elif current_state == PickupState.MOVING_TO_TARGET:
                # Обновляем feedback
                feedback_msg.current_state = '[3/10] MOVING_TO_TARGET'
                feedback_msg.status_message = f'Движение манипулятора к объекту {object_name}...'
                feedback_msg.current_x = self.current_position['x']
                feedback_msg.current_y = self.current_position['y']
                feedback_msg.current_z = self.current_position['z']
                
                # Проверяем наличие координат объекта
                if self.object_coordinates is not None:
                    # Координаты объекта в base_link
                    obj_x_base = self.object_coordinates.x
                    obj_y_base = self.object_coordinates.y
                    obj_z_base = self.object_coordinates.z
                    
                    # Трансформируем из base_link в arm_link (систему координат первого звена)
                    obj_x_arm = obj_x_base - self.ARM_LINK_OFFSET['x']
                    obj_y_arm = obj_y_base - self.ARM_LINK_OFFSET['y']
                    obj_z_arm = obj_z_base - self.ARM_LINK_OFFSET['z']
                    
                    # Вычисляем целевую позицию (98% от расстояния)
                    target_x = obj_x_arm * 1.05
                    target_y = obj_y_arm * 1
                    target_z = obj_z_arm * 1
                    
                    self.get_logger().info(
                        f'Координаты объекта: base_link({obj_x_base:.3f}, {obj_y_base:.3f}, {obj_z_base:.3f}) '
                        f'→ arm_link({obj_x_arm:.3f}, {obj_y_arm:.3f}, {obj_z_arm:.3f})'
                    )
                    self.get_logger().info(
                        f'Целевая позиция манипулятора : '
                        f'x={target_x:.3f}, y={target_y:.3f}, z={target_z:.3f}'
                    )
                    
                    # Отправляем целевую позу манипулятору (в системе координат arm_link)
                    pose_msg = Pose()
                    pose_msg.position.x = target_x
                    pose_msg.position.y = target_y
                    pose_msg.position.z = target_z
                    # Ориентация (по умолчанию)
                    pose_msg.orientation.w = 1.0
                    
                    self.arm_target_publisher.publish(pose_msg)
                    
                    # Проверяем достижение целевой позиции
                    distance = math.sqrt(
                        (target_x - self.current_position['x'])**2 +
                        (target_y - self.current_position['y'])**2 +
                        (target_z - self.current_position['z'])**2
                    )
                    
                    if distance < self.POSITION_TOLERANCE:
                        self.get_logger().info(f'✓ Манипулятор достиг целевой позиции (расстояние: {distance:.3f}м)')
                        current_state = PickupState.CLOSING_GRIPPER
                else:
                    # Логируем для отладки (только один раз, чтобы не спамить)
                    if not hasattr(self, '_coords_warning_shown'):
                        self.get_logger().warn('⚠ Координаты объекта не получены. Проверьте:')
                        self.get_logger().warn('  1. Запущен ли object_coordinate_finder')
                        self.get_logger().warn('  2. Публикуется ли топик /object_coordinates')
                        self.get_logger().warn('  3. Обнаруживается ли объект в /detected_objects')
                        self._coords_warning_shown = True
            
            elif current_state == PickupState.CLOSING_GRIPPER:
                # Обновляем feedback
                feedback_msg.current_state = '[4/10] CLOSING_GRIPPER'
                feedback_msg.status_message = f'Закрытие захвата для захвата объекта {object_name}...'
                feedback_msg.current_x = self.current_position['x']
                feedback_msg.current_y = self.current_position['y']
                feedback_msg.current_z = self.current_position['z']
                
                # Отправляем команду на закрытие захвата
                gap_msg = Float64()
                gap_msg.data = self.GRIPPER_CLOSED_GAP
                self.gripper_target_gap_publisher.publish(gap_msg)
                
                # Проверяем, закрылся ли захват
                if abs(self.current_gripper_gap - self.GRIPPER_CLOSED_GAP) < self.GRIPPER_TOLERANCE:
                    self.get_logger().info(f'✓ Захват закрыт (gap={self.current_gripper_gap:.4f}м), объект захвачен')
                    current_state = PickupState.LIFTING_OBJECT
                else:
                    # Логируем текущее состояние захвата
                    if not hasattr(self, '_gripper_closing_logged'):
                        self.get_logger().info(f'Закрытие захвата... Текущий gap: {self.current_gripper_gap:.4f}м, цель: {self.GRIPPER_CLOSED_GAP:.4f}м')
                        self._gripper_closing_logged = True
            
            elif current_state == PickupState.LIFTING_OBJECT:
                # Обновляем feedback
                feedback_msg.current_state = '[5/10] LIFTING_OBJECT'
                feedback_msg.status_message = f'Подъём объекта {object_name}...'
                feedback_msg.current_x = self.current_position['x']
                feedback_msg.current_y = self.current_position['y']
                feedback_msg.current_z = self.current_position['z']
                
                # Поднимаем объект на безопасную высоту
                lift_height = 0.3  # Высота подъёма
                
                # Отправляем команду на подъём
                pose_msg = Pose()
                pose_msg.position.x = 0.1
                pose_msg.position.y = 0.0
                pose_msg.position.z = lift_height
                pose_msg.orientation.w = 1.0
                
                self.arm_target_publisher.publish(pose_msg)
                
                # Проверяем достижение высоты
                if abs(self.current_position['z'] - lift_height) < self.POSITION_TOLERANCE:
                    self.get_logger().info(f'✓ Объект поднят на высоту {lift_height:.3f}м')
                    # Сбрасываем флаг навигации перед переходом к следующему состоянию
                    self.navigation_goal_reached = False
                    current_state = PickupState.MOVING_TO_RELEASE_POINT
                else:
                    if not hasattr(self, '_lifting_logged'):
                        self.get_logger().info(f'Подъём объекта... Текущая высота: {self.current_position["z"]:.3f}м, цель: {lift_height:.3f}м')
                        self._lifting_logged = True
            
            elif current_state == PickupState.MOVING_TO_RELEASE_POINT:
                # Обновляем feedback
                feedback_msg.current_state = '[6/10] MOVING_TO_RELEASE_POINT'
                feedback_msg.status_message = 'Движение к точке освобождения...'
                feedback_msg.current_x = self.current_position['x']
                feedback_msg.current_y = self.current_position['y']
                feedback_msg.current_z = self.current_position['z']
                
                # Отправляем навигационную цель к release_point
                if not hasattr(self, '_release_navigation_started'):
                    self.get_logger().info('Начинаю движение к точке release_point')
                    if self.send_navigation_goal('release_point'):
                        self._release_navigation_started = True
                    else:
                        self.get_logger().error('Не удалось начать навигацию к release_point')
                        # Завершаем с ошибкой
                        goal_handle.abort()
                        result = PickupObject.Result()
                        result.success = False
                        result.message = 'Не удалось доехать до точки освобождения'
                        result.final_gap = self.current_gripper_gap
                        return result
                
                # Проверяем достижение release_point
                if self.check_navigation_goal_reached():
                    self.get_logger().info('✓ Робот достиг точки release_point')
                    current_state = PickupState.MOVING_ARM_TO_RELEASE
            
            elif current_state == PickupState.MOVING_ARM_TO_RELEASE:
                # Обновляем feedback
                feedback_msg.current_state = '[7/10] MOVING_ARM_TO_RELEASE'
                feedback_msg.status_message = 'Перемещение манипулятора к точке освобождения...'
                feedback_msg.current_x = self.current_position['x']
                feedback_msg.current_y = self.current_position['y']
                feedback_msg.current_z = self.current_position['z']
                
                # Отправляем команду на перемещение манипулятора к точке освобождения
                pose_msg = Pose()
                pose_msg.position.x = self.YOUBOT_RELEASE_POINT['x']
                pose_msg.position.y = self.YOUBOT_RELEASE_POINT['y']
                pose_msg.position.z = self.YOUBOT_RELEASE_POINT['z']
                pose_msg.orientation.w = 1.0
                
                self.arm_target_publisher.publish(pose_msg)
                
                # Проверяем достижение позиции освобождения
                distance = math.sqrt(
                    (self.YOUBOT_RELEASE_POINT['x'] - self.current_position['x'])**2 +
                    (self.YOUBOT_RELEASE_POINT['y'] - self.current_position['y'])**2 +
                    (self.YOUBOT_RELEASE_POINT['z'] - self.current_position['z'])**2
                )
                
                if distance < self.POSITION_TOLERANCE:
                    self.get_logger().info(f'✓ Манипулятор достиг точки освобождения (расстояние: {distance:.3f}м)')
                    current_state = PickupState.RELEASING_OBJECT
                else:
                    if not hasattr(self, '_moving_to_release_logged'):
                        self.get_logger().info(
                            f'Движение к точке освобождения... Расстояние: {distance:.3f}м, '
                            f'Цель: ({self.YOUBOT_RELEASE_POINT["x"]:.3f}, {self.YOUBOT_RELEASE_POINT["y"]:.3f}, {self.YOUBOT_RELEASE_POINT["z"]:.3f})'
                        )
                        self._moving_to_release_logged = True
            
            elif current_state == PickupState.RELEASING_OBJECT:
                # Обновляем feedback
                feedback_msg.current_state = '[8/10] RELEASING_OBJECT'
                feedback_msg.status_message = f'Освобождение объекта {object_name}...'
                feedback_msg.current_x = self.current_position['x']
                feedback_msg.current_y = self.current_position['y']
                feedback_msg.current_z = self.current_position['z']
                
                # Отправляем команду на открытие захвата
                gap_msg = Float64()
                gap_msg.data = self.GRIPPER_OPEN_GAP
                self.gripper_target_gap_publisher.publish(gap_msg)
                
                # Проверяем, открылся ли захват
                if abs(self.current_gripper_gap - self.GRIPPER_OPEN_GAP) < self.GRIPPER_TOLERANCE:
                    self.get_logger().info(f'✓ Захват открыт (gap={self.current_gripper_gap:.4f}м), объект освобождён')
                    current_state = PickupState.LIFTING_ARM_AFTER_RELEASE
                else:
                    # Логируем текущее состояние захвата
                    if not hasattr(self, '_gripper_opening_logged'):
                        self.get_logger().info(f'Открытие захвата... Текущий gap: {self.current_gripper_gap:.4f}м, цель: {self.GRIPPER_OPEN_GAP:.4f}м')
                        self._gripper_opening_logged = True
            
            elif current_state == PickupState.LIFTING_ARM_AFTER_RELEASE:
                # Обновляем feedback
                feedback_msg.current_state = '[9/10] LIFTING_ARM_AFTER_RELEASE'
                feedback_msg.status_message = 'Подъём манипулятора перед возвратом...'
                feedback_msg.current_x = self.current_position['x']
                feedback_msg.current_y = self.current_position['y']
                feedback_msg.current_z = self.current_position['z']
                
                # Поднимаем манипулятор на безопасную высоту перед возвратом
                safe_height = 0.4
                
                # Отправляем команду на подъём
                pose_msg = Pose()
                pose_msg.position.x = self.current_position['x']
                pose_msg.position.y = self.current_position['y']
                pose_msg.position.z = safe_height
                pose_msg.orientation.w = 1.0
                
                self.arm_target_publisher.publish(pose_msg)
                
                # Проверяем достижение высоты
                if abs(self.current_position['z'] - safe_height) < self.POSITION_TOLERANCE:
                    self.get_logger().info(f'✓ Манипулятор поднят на безопасную высоту {safe_height:.3f}м')
                    # Сбрасываем флаг навигации перед переходом к возврату домой
                    self.navigation_goal_reached = False
                    current_state = PickupState.MOVING_TO_HOME
                else:
                    if not hasattr(self, '_lifting_after_release_logged'):
                        self.get_logger().info(f'Подъём манипулятора... Текущая высота: {self.current_position["z"]:.3f}м, цель: {safe_height:.3f}м')
                        self._lifting_after_release_logged = True
            
            elif current_state == PickupState.MOVING_TO_HOME:
                # Обновляем feedback
                feedback_msg.current_state = '[10/10] MOVING_TO_HOME'
                feedback_msg.status_message = 'Возврат в домашнюю позицию...'
                feedback_msg.current_x = self.current_position['x']
                feedback_msg.current_y = self.current_position['y']
                feedback_msg.current_z = self.current_position['z']
                
                # Отправляем навигационную цель на возврат домой
                if not hasattr(self, '_home_navigation_started'):
                    self.get_logger().info('Начинаю возврат в точку home')
                    if self.send_navigation_goal('home'):
                        self._home_navigation_started = True
                    else:
                        self.get_logger().error('Не удалось начать навигацию к home')
                        # Завершаем с ошибкой
                        goal_handle.abort()
                        result = PickupObject.Result()
                        result.success = False
                        result.message = 'Не удалось вернуться в home'
                        result.final_gap = self.current_gripper_gap
                        return result
                
                # Проверяем достижение home
                if self.check_navigation_goal_reached():
                    self.get_logger().info('✓ Робот вернулся в точку home')
                    current_state = PickupState.COMPLETED
            
            # Отправка feedback
            goal_handle.publish_feedback(feedback_msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Успешное завершение
        goal_handle.succeed()
        result = PickupObject.Result()
        result.success = True
        result.message = f'Объект {object_name} успешно обработан'
        result.final_gap = 0.0
        return result

    def gripper_gap_callback(self, msg: Float64):
        """Callback для текущего раскрытия захвата"""
        self.current_gripper_gap = msg.data

    def gripper_current_point_callback(self, msg: Point):
        """Callback для текущей позиции манипулятора"""
        self.current_position['x'] = msg.x
        self.current_position['y'] = msg.y
        self.current_position['z'] = msg.z
    
    def detected_objects_callback(self, msg: BoundingBoxArray):
        """Callback для обнаруженных объектов"""
        self.detected_objects = msg.boxes
        
        # Логируем обнаруженные объекты (опционально)
        if len(self.detected_objects) > 0:
            object_names = [bbox.class_name for bbox in self.detected_objects]
            # self.get_logger().debug(f'Обнаружены объекты: {", ".join(object_names)}')
    
    def object_coordinates_callback(self, msg: PointStamped):
        """Callback для координат объекта в системе base_link"""
        # Сохраняем координаты объекта
        self.object_coordinates = msg.point
        
        self.get_logger().info(
            f'✓ Получены координаты объекта в base_link: '
            f'x={msg.point.x:.3f}, y={msg.point.y:.3f}, z={msg.point.z:.3f}'
        )


def main(args=None):
    rclpy.init(args=args)
    action_server = PickupAction()
    
    try:
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        pass
    
    action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
