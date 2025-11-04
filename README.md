# YouBot Object Pickup System

Автономная система захвата и транспортировки объектов для робота KUKA YouBot с использованием ROS2, Nav2, компьютерного зрения (YOLO) и обратной кинематики.

![Демонстрация работы](demo.gif)

## 📋 Содержание

- [Установка](#установка)
- [Использование](#использование)
- [Архитектура](#архитектура)
- [Отладка](#отладка)

##  Установка

### Зависимости
```bash
cd ~/webots_ws

# ROS2 пакеты
sudo apt install ros-humble-slam-toolbox ros-humble-nav2-bringup
rosdep install --from-paths src --ignore-src -r -y

# Python пакеты
pip install ultralytics opencv-python numpy scipy
```

### Сборка
```bash
colcon build --symlink-install
source install/setup.bash
```

### Модель YOLO
Поместите модель в: `~/webots_ws/src/youbot_control/models/best.pt`

## 🚀 Использование

### Запуск системы
```bash
# Запустить всю систему
ros2 launch youbot_control bringup_launch.py
```

### Захват объекта
```bash
# Захватить объект
ros2 action send_goal --feedback /pickup_object youbot_interfaces/action/PickupObject "{object_name: 'ball'}"

# Доступные объекты: 'ball', 'apple', 'cube'
```

### SLAM и карты
```bash
# Построить карту
ros2 launch youbot_control slam_launch.py

# Управление роботом
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Сохранить карту
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name: {data: 'src/youbot_control/maps/map'}"
```

## 🏗️ Архитектура

### Конечный автомат (10 состояний)
```
1. MOVING_TO_OBJECT          → Навигация к объекту
2. OPENING_GRIPPER            → Открытие схвата
3. MOVING_TO_TARGET           → Движение манипулятора
4. CLOSING_GRIPPER            → Захват
5. LIFTING_OBJECT             → Подъём (0.3м)
6. MOVING_TO_RELEASE_POINT    → Навигация к точке освобождения
7. MOVING_ARM_TO_RELEASE      → Движение манипулятора
8. RELEASING_OBJECT           → Открытие схвата
9. LIFTING_ARM_AFTER_RELEASE  → Подъём (0.4м)
10. MOVING_TO_HOME            → Возврат домой
```

### Компоненты
- **pt_detection_node** - YOLO детекция
- **object_coordinate_finder** - 2D→3D координаты
- **arm_controller** - обратная кинематика
- **pickup_action_server** - координация захвата

### Waypoints
Редактируйте `youbot_control/nodes/goal_send_node.py`:
```python
self.waypoints = {
    'home': [0.0, 0.0, 0.0],
    'ball': [6.3, -5.0, 0.0],
    'apple': [6.5, 4.2, 0.85],
    'cube': [-3.7, -7.7, 0.0],
    'release_point': [6.8, 0.2, -1.57]
}
```

## 🐛 Отладка

```bash
# Проверка детекции
ros2 topic echo /detected_objects

# Проверка координат
ros2 topic echo /object_coordinates

# Состояние манипулятора
ros2 topic echo /arm_current_position_point

# Состояние схвата
ros2 topic echo /gripper_current_gap

# Навигация вручную
ros2 service call /navigate_to_object youbot_interfaces/srv/NavigateToObject "{waypoint_name: 'home'}"

# Управление схватом
ros2 topic pub --once /gripper_target_gap std_msgs/Float64 "data: 0.071"  # открыть
ros2 topic pub --once /gripper_target_gap std_msgs/Float64 "data: 0.021"    # закрыть
```

### Проблемы

**RViz не запускается из VS Code:**
```bash
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
export LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0
```

**Объект не обнаруживается:**
- Проверьте модель YOLO: `models/best.pt`
- Проверьте confidence threshold (0.5)

---

 [CrazyBeaver2003](https://github.com/CrazyBeaver2003)



