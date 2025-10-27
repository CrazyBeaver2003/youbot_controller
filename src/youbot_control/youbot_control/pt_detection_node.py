#!/usr/bin/env python3

import rclpy
import os
import time
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO

from youbot_interfaces.msg import BoundingBox, BoundingBoxArray


class PTObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('pt_object_detection_node')

        self.package_dir = get_package_share_directory('youbot_control')
        self.model_path = os.path.join(self.package_dir, 'models', 'best.pt')
        
        self.declare_parameter('model_path', self.model_path)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.4)
        self.declare_parameter('device', 0)  # 'cpu' или 0 для GPU

        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.iou_threshold = self.get_parameter('iou_threshold').value
        self.device = self.get_parameter('device').value

        # Загружаем PT модель (YOLO)
        try:
            self.model = YOLO(str(self.model_path))
            self.get_logger().info(f'✓ Модель загружена: {self.model_path}')
        except Exception as e:
            self.get_logger().error(f'✗ Ошибка загрузки модели: {e}')
            raise

        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_color',
            self.image_callback,
            10
        )

        self.bbox_publisher = self.create_publisher(
            BoundingBoxArray,
            '/detected_objects',
            10
        )

        self.image_publisher = self.create_publisher(
            Image,
            '/detection_image',
            10
        )

        self.bridge = CvBridge()
        self.inference_times = []
        self.frame_count = 0

        self.get_logger().info('PT Object Detection Node initialized.')

    def image_callback(self, msg):
        """Callback для обработки изображения"""
        try:
            self.frame_count += 1
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            start_time = time.time()

            # Запускаем инференс
            results = self.model(
                frame,
                conf=self.confidence_threshold,
                iou=self.iou_threshold,
                device=self.device,
                verbose=False
            )

            inference_time = (time.time() - start_time) * 1000
            self.inference_times.append(inference_time)

            # Публикуем результаты
            bbox_array = BoundingBoxArray()
            bbox_array.header = msg.header

            frame_with_boxes = frame.copy()

            if results and len(results) > 0:
                result = results[0]

                if result.boxes is not None:
                    for box in result.boxes:
                        # Получаем координаты
                        xyxy = box.xyxy[0]  # [x_min, y_min, x_max, y_max]
                        conf = float(box.conf[0])
                        class_id = int(box.cls[0])
                        class_name = self.model.names[class_id]

                        # Конвертируем в center, size формат
                        x_min, y_min, x_max, y_max = float(xyxy[0]), float(xyxy[1]), float(xyxy[2]), float(xyxy[3])
                        center_x = (x_min + x_max) / 2.0
                        center_y = (y_min + y_max) / 2.0
                        size_x = x_max - x_min
                        size_y = y_max - y_min
                        area = size_x * size_y

                        # Создаем BoundingBox сообщение
                        bbox = BoundingBox()
                        bbox.center_x = center_x
                        bbox.center_y = center_y
                        bbox.size_x = size_x
                        bbox.size_y = size_y
                        bbox.area = area
                        bbox.confidence = conf
                        bbox.class_name = class_name

                        bbox_array.boxes.append(bbox)

                        # Рисуем бокс на изображении
                        x_min_int, y_min_int = int(x_min), int(y_min)
                        x_max_int, y_max_int = int(x_max), int(y_max)
                        
                        cv2.rectangle(frame_with_boxes, (x_min_int, y_min_int),
                                    (x_max_int, y_max_int), (0, 255, 0), 2)

                        # Рисуем текст
                        label = f"{class_name}: {conf:.2f}"
                        cv2.putText(frame_with_boxes, label,
                                  (x_min_int, y_min_int - 10),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                  (0, 255, 0), 2)

            # Публикуем результаты
            self.bbox_publisher.publish(bbox_array)

            # Публикуем изображение с боксами
            image_msg = self.bridge.cv2_to_imgmsg(frame_with_boxes,
                                                 encoding='bgr8')
            image_msg.header = msg.header
            self.image_publisher.publish(image_msg)

            # Выводим статистику каждые 30 кадров
            if len(self.inference_times) >= 30:
                avg_time = np.mean(self.inference_times)
                min_time = np.min(self.inference_times)
                max_time = np.max(self.inference_times)
                fps = 1000 / avg_time
                
                self.get_logger().info(
                    f'Frame #{self.frame_count} | '
                    f'Avg: {avg_time:.1f}ms | '
                    f'Min: {min_time:.1f}ms | '
                    f'Max: {max_time:.1f}ms | '
                    f'FPS: {fps:.1f} | '
                    f'Objects: {len(bbox_array.boxes)}'
                )
                self.inference_times = []

        except Exception as e:
            self.get_logger().error(f'Ошибка при обработке изображения: {e}')
            import traceback
            traceback.print_exc()


def main(args=None):
    rclpy.init(args=args)
    node = PTObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
