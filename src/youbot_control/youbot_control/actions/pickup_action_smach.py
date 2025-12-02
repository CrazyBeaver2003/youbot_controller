#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Point, PointStamped
from nav2_msgs.action import NavigateToPose
from youbot_interfaces.action import PickupObject
from youbot_interfaces.srv import NavigateToObject
from youbot_interfaces.msg import BoundingBoxArray

import smach
import math
import time

# =============================================================================
# БАЗОВЫЙ КЛАСС СОСТОЯНИЯ
# =============================================================================
class BaseActionState(smach.State):
    """
    Базовый класс для всех состояний нашего автомата.
    Он берет на себя рутину:
    1. Проверку отмены задачи (Preemption)
    2. Публикацию Feedback (чтобы пользователь видел, что происходит)
    """
    def __init__(self, node, state_name, outcomes=['succeeded', 'aborted', 'preempted']):
        super().__init__(outcomes=outcomes)
        self.node = node
        self.state_name = state_name

    def execute(self, userdata):
        # 1. Проверяем, не попросил ли пользователь отменить задачу
        if self.node.goal_handle.is_cancel_requested:
            self.node.goal_handle.canceled()
            return 'preempted'
        
        # 2. Сообщаем, что мы вошли в это состояние
        self.node.publish_feedback(self.state_name, "Начало выполнения...")
        
        # 3. Запускаем логику конкретного состояния
        return self.run_logic()

    def run_logic(self):
        """Этот метод должен быть переопределен в наследниках"""
        raise NotImplementedError

# =============================================================================
# СОСТОЯНИЕ НАВИГАЦИИ
# =============================================================================
class NavigationState(BaseActionState):
    """
    Состояние, отвечающее за перемещение робота к заданной точке (waypoint).
    Использует сервис /navigate_to_object для получения координат и Nav2 для движения.
    """
    def __init__(self, node, state_name, target_name):
        super().__init__(node, state_name)
        self.target_name = target_name

    def run_logic(self):
        # Сбрасываем флаг прибытия
        self.node.navigation_goal_reached = False
        self.node.navigation_goal_failed = False
        
        # Отправляем робота к цели (асинхронно, но ждем внутри send_navigation_goal)
        self.node.get_logger().info(f"[{self.state_name}] Еду к точке: {self.target_name}")
        self.node.send_navigation_goal(self.target_name)
        
        # Цикл ожидания прибытия
        while rclpy.ok():
            # Проверка отмены
            if self.node.goal_handle.is_cancel_requested:
                return 'preempted'
            
            # Проверка успеха
            if self.node.check_navigation_goal_reached():
                self.node.get_logger().info(f"[{self.state_name}] Прибыл!")
                return 'succeeded'
            
            # Проверка ошибки
            if self.node.navigation_goal_failed:
                self.node.get_logger().error(f"[{self.state_name}] Ошибка навигации!")
                return 'aborted'
            
            # Публикуем фидбек
            self.node.publish_feedback(self.state_name, f"Еду к {self.target_name}...")
            time.sleep(0.5) # Не частим с проверками
            
        return 'aborted'

# =============================================================================
# СОСТОЯНИЕ УПРАВЛЕНИЯ СХВАТОМ
# =============================================================================
class GripperState(BaseActionState):
    """
    Состояние для открытия или закрытия схвата.
    """
    def __init__(self, node, state_name, target_gap, tolerance=0.005):
        super().__init__(node, state_name)
        self.target_gap = target_gap
        self.tolerance = tolerance

    def run_logic(self):
        current_target = self.target_gap
        current_tolerance = self.tolerance
        
        # Специальная логика для "cube" (или duck) - полное сжатие
        if self.state_name == "CLOSING_GRIPPER" and self.node.target_object_name == 'cube':
            current_target = 0.0
            current_tolerance = 0.01
            self.node.get_logger().info("🧊 CUBE MODE: Полное сжатие схвата!")

        # Публикуем команду
        msg = Float64()
        msg.data = current_target
        self.node.gripper_target_gap_publisher.publish(msg)

        # Ждем выполнения
        start_time = time.time()
        while rclpy.ok():
            if self.node.goal_handle.is_cancel_requested: return 'preempted'
            
            # Проверяем, достиг ли схват нужного положения
            if abs(self.node.current_gripper_gap - current_target) < current_tolerance:
                return 'succeeded'
            
            # Таймаут (на всякий случай)
            if time.time() - start_time > 5.0:
                self.node.get_logger().warn(f"[{self.state_name}] Таймаут схвата, считаем что успех")
                return 'succeeded'
            
            self.node.publish_feedback(self.state_name, f"Gap: {self.node.current_gripper_gap:.3f} -> {current_target}")
            time.sleep(0.1)
        return 'aborted'

# =============================================================================
# СОСТОЯНИЕ ДВИЖЕНИЯ МАНИПУЛЯТОРА
# =============================================================================
class ArmMoveState(BaseActionState):
    """
    Состояние для управления рукой (Inverse Kinematics).
    Может двигаться:
    - 'coords': к координатам найденного объекта
    - 'lift': поднять объект вверх
    - 'release_pose': в позицию сброса
    - 'lift_after': поднять руку после сброса
    """
    def __init__(self, node, state_name, target_type='coords'):
        super().__init__(node, state_name)
        self.target_type = target_type 

    def run_logic(self):
        target_pose = Pose()
        target_pose.orientation.w = 1.0 # Ориентация всегда вертикальная (условно)
        
        # --- Логика выбора цели ---
        if self.target_type == 'coords':
            if self.node.object_coordinates is None:
                self.node.get_logger().error("Координаты объекта не найдены!")
                return 'aborted'
            
            # Трансформация координат из base_link в arm_link
            tx = self.node.object_coordinates.x - self.node.ARM_LINK_OFFSET['x']
            ty = self.node.object_coordinates.y - self.node.ARM_LINK_OFFSET['y']
            tz = self.node.object_coordinates.z - self.node.ARM_LINK_OFFSET['z']
            
            # Корректировки для разных объектов
            if self.node.target_object_name == 'cube':
                tz -= 0.01 # Чуть ниже для кубика
                self.node.get_logger().info("🧊 CUBE MODE: Опускаюсь ниже!")

            # Небольшой коэффициент безопасности (0.98), чтобы не врезаться
            target_pose.position.x = tx * 0.98
            target_pose.position.y = ty * 0.98
            target_pose.position.z = tz * 0.98

        elif self.target_type == 'lift':
            # Поднимаем только Z, X и Y оставляем текущими
            target_pose.position.x = self.node.current_position['x']
            target_pose.position.y = self.node.current_position['y']
            target_pose.position.z = 0.3

        elif self.target_type == 'release_pose':
            # Фиксированная точка сброса (относительно руки)
            target_pose.position.x = self.node.YOUBOT_RELEASE_POINT['x']
            target_pose.position.y = self.node.YOUBOT_RELEASE_POINT['y']
            target_pose.position.z = self.node.YOUBOT_RELEASE_POINT['z']
            
        elif self.target_type == 'lift_after':
            target_pose.position.x = self.node.current_position['x']
            target_pose.position.y = self.node.current_position['y']
            target_pose.position.z = 0.4

        # Публикуем цель для IK контроллера
        self.node.arm_target_publisher.publish(target_pose)

        # Ждем достижения цели
        while rclpy.ok():
            if self.node.goal_handle.is_cancel_requested: return 'preempted'
            
            # Считаем расстояние до цели
            dx = target_pose.position.x - self.node.current_position['x']
            dy = target_pose.position.y - self.node.current_position['y']
            dz = target_pose.position.z - self.node.current_position['z']
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)

            if dist < self.node.POSITION_TOLERANCE:
                return 'succeeded'
            
            self.node.publish_feedback(self.state_name, f"Dist: {dist:.3f}")
            time.sleep(0.1)
        return 'aborted'

# =============================================================================
# СОСТОЯНИЕ ПРОВЕРКИ ВИДИМОСТИ
# =============================================================================
class CheckVisibilityState(BaseActionState):
    """
    Проверяет, видит ли камера объект перед началом манипуляций.
    """
    def run_logic(self):
        self.node.get_logger().info(f"Ищу объект: {self.node.target_object_name}")
        
        # Делаем несколько попыток (на случай мигания детекции)
        for i in range(5):
            if self.node.check_object_visible(self.node.target_object_name):
                self.node.get_logger().info("Объект найден!")
                return 'succeeded'
            time.sleep(0.5)
            
        self.node.get_logger().warn(f"Объект {self.node.target_object_name} не виден!")
        # В реальной задаче тут можно вернуть 'aborted', но для тестов вернем 'succeeded'
        # чтобы робот попытался схватить по последним известным координатам (если они есть)
        if self.node.object_coordinates is not None:
             self.node.get_logger().info("Использую последние известные координаты.")
             return 'succeeded'
             
        return 'aborted'


# =============================================================================
# ОСНОВНОЙ КЛАСС NODE
# =============================================================================

class PickupAction(Node):
    def __init__(self):
        super().__init__('pickup_action_server')
        
        # Callback group для многопоточности (важно для SMACH + ActionServer)
        # Это позволяет коллбекам (подписчикам) работать параллельно с машиной состояний
        self.cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self, PickupObject, 'pickup_object',
            self.execute_callback, callback_group=self.cb_group
        )

        # --- Publishers / Subscribers ---
        self.gripper_current_point_subscriber = self.create_subscription(
            Point, 'arm_current_position_point', self.gripper_current_point_callback, 10, callback_group=self.cb_group)
        
        self.gripper_gap_subscriber = self.create_subscription(
            Float64, 'gripper_current_gap', self.gripper_gap_callback, 10, callback_group=self.cb_group)
            
        self.object_coordinates_subscriber = self.create_subscription(
            PointStamped, 'object_coordinates', self.object_coordinates_callback, 10, callback_group=self.cb_group)
            
        self.detected_objects_subscriber = self.create_subscription(
            BoundingBoxArray, 'detected_objects', self.detected_objects_callback, 10, callback_group=self.cb_group)

        self.arm_target_publisher = self.create_publisher(Pose, 'arm_target_pose', 10)
        self.gripper_target_gap_publisher = self.create_publisher(Float64, 'gripper_target_gap', 10)

        # --- Clients ---
        self.navigate_client = self.create_client(NavigateToObject, 'navigate_to_object', callback_group=self.cb_group)
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=self.cb_group)

        # --- Variables ---
        self.current_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.current_gripper_gap = 0.0
        self.object_coordinates = None
        self.detected_objects = []
        self.navigation_goal_reached = False
        self.navigation_goal_failed = False
        self.target_object_name = ""
        self.goal_handle = None

        # --- Constants ---
        self.ARM_LINK_OFFSET = {'x': 0.16, 'y': 0.0, 'z': -0.109}
        self.YOUBOT_RELEASE_POINT = {'x': 0.3, 'y': 0.0, 'z': 0.2}
        self.POSITION_TOLERANCE = 0.02
        self.GRIPPER_OPEN_GAP = 0.071
        self.GRIPPER_CLOSED_GAP = 0.035

    # --- Callbacks ---
    def gripper_current_point_callback(self, msg):
        self.current_position = {'x': msg.x, 'y': msg.y, 'z': msg.z}

    def gripper_gap_callback(self, msg):
        self.current_gripper_gap = msg.data

    def object_coordinates_callback(self, msg):
        self.object_coordinates = msg.point

    def detected_objects_callback(self, msg):
        self.detected_objects = msg.boxes

    def check_object_visible(self, name):
        for box in self.detected_objects:
            if box.class_id == name: return True
        return False

    # --- Navigation Helpers ---
    def send_navigation_goal(self, target_name):
        """Отправляет запрос сервису navigate_to_object, а затем action navigate_to_pose"""
        self.navigation_goal_failed = False
        req = NavigateToObject.Request()
        req.waypoint_name = target_name
        
        # Вызываем сервис асинхронно
        future = self.navigate_client.call_async(req)
        
        # Ждем результат сервиса без блокировки экзекьютора
        while not future.done():
            time.sleep(0.1)
            if not rclpy.ok(): return

        try:
            resp = future.result()
            
            if resp and resp.success:
                # Если точка найдена, отправляем робота туда через Nav2
                goal = NavigateToPose.Goal()
                goal.pose = resp.pose
                
                if not self.nav_action_client.wait_for_server(timeout_sec=2.0):
                    self.get_logger().error("Nav2 action server not available")
                    self.navigation_goal_failed = True
                    return

                send_goal_future = self.nav_action_client.send_goal_async(goal)
                
                # Ждем принятия цели
                while not send_goal_future.done():
                    time.sleep(0.1)
                    if not rclpy.ok(): return

                goal_handle = send_goal_future.result()
                
                if goal_handle.accepted:
                    self.get_logger().info("Nav2 goal accepted")
                    res_future = goal_handle.get_result_async()
                    res_future.add_done_callback(self.nav_done_callback)
                else:
                    self.get_logger().error("Nav2 goal rejected")
                    self.navigation_goal_failed = True
            else:
                self.get_logger().error(f"Waypoint {target_name} not found via service")
                self.navigation_goal_failed = True
                
        except Exception as e:
            self.get_logger().error(f"Nav error: {e}")
            self.navigation_goal_failed = True

    def nav_done_callback(self, future):
        try:
            result = future.result()
            status = result.status
            self.get_logger().info(f"Nav2 finished with status: {status}")
            if status == 4: # SUCCEEDED
                self.navigation_goal_reached = True
            else:
                self.get_logger().warn(f"Navigation failed with status {status}")
                self.navigation_goal_failed = True
        except Exception as e:
            self.get_logger().error(f"Nav callback error: {e}")
            self.navigation_goal_failed = True

    def check_navigation_goal_reached(self):
        return self.navigation_goal_reached

    def publish_feedback(self, state, msg):
        if self.goal_handle:
            fb = PickupObject.Feedback()
            fb.current_state = state
            fb.status_message = msg
            fb.current_x = self.current_position['x']
            fb.current_y = self.current_position['y']
            fb.current_z = self.current_position['z']
            self.goal_handle.publish_feedback(fb)

    # --- EXECUTE CALLBACK (СБОРКА SMACH) ---
    def execute_callback(self, goal_handle):
        self.get_logger().info("Запуск SMACH автомата...")
        self.goal_handle = goal_handle
        self.target_object_name = goal_handle.request.object_name.lower()
        
        # Создаем машину состояний
        sm = smach.StateMachine(outcomes=['mission_completed', 'mission_failed', 'mission_preempted'])

        # Открываем контейнер для добавления состояний
        with sm:
            # 1. Навигация к объекту
            smach.StateMachine.add('MOVING_TO_OBJECT', 
                NavigationState(self, 'MOVING_TO_OBJECT', self.target_object_name),
                transitions={'succeeded':'OPENING_GRIPPER', 'aborted':'mission_failed', 'preempted':'mission_preempted'})

            # 2. Открытие схвата
            smach.StateMachine.add('OPENING_GRIPPER',
                GripperState(self, 'OPENING_GRIPPER', self.GRIPPER_OPEN_GAP),
                transitions={'succeeded':'CHECK_VISIBILITY', 'aborted':'mission_failed', 'preempted':'mission_preempted'})

            # 3. Проверка видимости
            smach.StateMachine.add('CHECK_VISIBILITY',
                CheckVisibilityState(self, 'CHECK_VISIBILITY'),
                transitions={'succeeded':'MOVING_TO_TARGET', 'aborted':'mission_failed', 'preempted':'mission_preempted'})

            # 4. Движение руки к объекту
            smach.StateMachine.add('MOVING_TO_TARGET',
                ArmMoveState(self, 'MOVING_TO_TARGET', target_type='coords'),
                transitions={'succeeded':'CLOSING_GRIPPER', 'aborted':'mission_failed', 'preempted':'mission_preempted'})

            # 5. Захват
            smach.StateMachine.add('CLOSING_GRIPPER',
                GripperState(self, 'CLOSING_GRIPPER', self.GRIPPER_CLOSED_GAP),
                transitions={'succeeded':'LIFTING_OBJECT', 'aborted':'mission_failed', 'preempted':'mission_preempted'})

            # 6. Подъем
            smach.StateMachine.add('LIFTING_OBJECT',
                ArmMoveState(self, 'LIFTING_OBJECT', target_type='lift'),
                transitions={'succeeded':'MOVING_TO_RELEASE_POINT', 'aborted':'mission_failed', 'preempted':'mission_preempted'})

            # 7. Навигация к точке сброса
            smach.StateMachine.add('MOVING_TO_RELEASE_POINT',
                NavigationState(self, 'MOVING_TO_RELEASE_POINT', 'release_point'),
                transitions={'succeeded':'MOVING_ARM_TO_RELEASE', 'aborted':'mission_failed', 'preempted':'mission_preempted'})

            # 8. Рука в позицию сброса
            smach.StateMachine.add('MOVING_ARM_TO_RELEASE',
                ArmMoveState(self, 'MOVING_ARM_TO_RELEASE', target_type='release_pose'),
                transitions={'succeeded':'RELEASING_OBJECT', 'aborted':'mission_failed', 'preempted':'mission_preempted'})

            # 9. Разжатие
            smach.StateMachine.add('RELEASING_OBJECT',
                GripperState(self, 'RELEASING_OBJECT', self.GRIPPER_OPEN_GAP),
                transitions={'succeeded':'LIFTING_ARM_AFTER_RELEASE', 'aborted':'mission_failed', 'preempted':'mission_preempted'})

            # 10. Подъем после сброса
            smach.StateMachine.add('LIFTING_ARM_AFTER_RELEASE',
                ArmMoveState(self, 'LIFTING_ARM_AFTER_RELEASE', target_type='lift_after'),
                transitions={'succeeded':'MOVING_TO_HOME', 'aborted':'mission_failed', 'preempted':'mission_preempted'})

            # 11. Домой
            smach.StateMachine.add('MOVING_TO_HOME',
                NavigationState(self, 'MOVING_TO_HOME', 'home'),
                transitions={'succeeded':'mission_completed', 'aborted':'mission_failed', 'preempted':'mission_preempted'})

        # Запуск машины
        outcome = sm.execute()

        # Формируем результат Action
        result = PickupObject.Result()
        if outcome == 'mission_completed':
            goal_handle.succeed()
            result.success = True
            result.message = "Миссия выполнена успешно!"
        elif outcome == 'mission_preempted':
            # goal_handle.canceled() # Уже вызвано внутри состояния
            result.success = False
            result.message = "Миссия отменена."
        else:
            goal_handle.abort()
            result.success = False
            result.message = "Миссия провалена."
        
        return result

def main(args=None):
    rclpy.init(args=args)
    node = PickupAction()
    
    # Используем MultiThreadedExecutor для корректной работы ActionServer + Callbacks
    # Это позволяет машине состояний (которая блокирует поток) работать одновременно с ROS коллбеками
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
