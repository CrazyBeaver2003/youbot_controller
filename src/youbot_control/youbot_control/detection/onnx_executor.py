#!/usr/bin/env python3

import rclpy
import os
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import onnxruntime as rt
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from youbot_interfaces.msg import BoundingBox, BoundingBoxArray

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        self.package_dir = get_package_share_directory('youbot_control')
        self.model_path = os.path.join(self.package_dir, 'models', 'best.onnx')
        self.declare_parameter('model_path', self.model_path)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.4)
        self.declare_parameter('input_size', 640)

        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.iou_threshold = self.get_parameter('iou_threshold').value
        self.input_size = self.get_parameter('input_size').value

        self.class_names = [
            'person', 'bicycle', 'car', 'motorcycle', 'airplane',
            'bus', 'train', 'truck', 'boat', 'traffic light',
            'apple', 'ball', 'duck',
        ]
        
        self.session = rt.InferenceSession(
            str(Path(self.model_path)),
            providers=['CPUExecutionProvider']
        )
        self.input_name = self.session.get_inputs()[0].name
        self.output_names = [o.name for o in self.session.get_outputs()]

        self.image_subscriber = self.create_subscription(
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

        self.get_logger().info('Object Detection Node initialized.')
    
    def preprocess(self, image):
        """Подготовка изображения для модели"""
        h,w = image.shape[:2]
        
        # Масштабируем, сохраняя aspect ratio(соотношение сторон)
        scale = self.input_size/max(h,w)
        new_h, new_w = int(h*scale), int(w*scale)

        resized_image = cv2.resize(image, (new_w, new_h))

        # Добавляем padding
        canvas = np.full((self.input_size, self.input_size, 3), 114, dtype=np.uint8)
        y_offset = (self.input_size - new_h) // 2
        x_offset = (self.input_size - new_w) // 2
        canvas[y_offset:y_offset+new_h, x_offset:x_offset+new_w] = resized_image

        # Нормализуем и конвертируем в float32
        blob = cv2.dnn.blobFromImage(
            canvas,
            scalefactor=1/255.0,
            size=(self.input_size, self.input_size),
            swapRB=True,
        )

        return blob, scale, (x_offset, y_offset)

    def postprocess(self, outputs, scale, offsets, frame_shape):
        """
        Обработка выходов ONNX модели
        Формат выхода: [1, 7, 8400]
        - координаты: [0:4] = [x, y, w, h]
        - objectness: [4] 
        - классы: [5:7] (2 класса)
        """
        output = outputs[0]  # [1, 7, 8400]
        
        if len(output.shape) == 3:
            output = output[0]  # [7, 8400]
        
        # Транспонируем для удобства: [8400, 7]
        output = output.transpose(1, 0)
        
        boxes = []
        confidences = []
        class_ids = []
        
        for pred in output:
            # pred = [x, y, w, h, objectness, class_score1, class_score2]
            cx, cy, w, h = pred[:4]
            objectness = pred[4]
            
            # Пропускаем слабые предсказания
            if objectness < self.confidence_threshold:
                continue
            
            # Получаем вероятности классов
            class_scores = pred[5:]
            class_id = np.argmax(class_scores)
            class_confidence = class_scores[class_id]
            
            # Финальная уверенность
            confidence = objectness * class_confidence
            
            if confidence < self.confidence_threshold:
                continue
            
            # Конвертируем координаты из центра в углы
            x_min = cx - w / 2
            y_min = cy - h / 2
            x_max = cx + w / 2
            y_max = cy + h / 2
            
            # Убираем масштабирование и смещение
            x_offset, y_offset = offsets
            x_min = (x_min - x_offset) / scale
            y_min = (y_min - y_offset) / scale
            x_max = (x_max - x_offset) / scale
            y_max = (y_max - y_offset) / scale
            
            # Обрезаем координаты до границ изображения
            h_img, w_img = frame_shape[:2]
            x_min = max(0, min(w_img, x_min))
            y_min = max(0, min(h_img, y_min))
            x_max = max(0, min(w_img, x_max))
            y_max = max(0, min(h_img, y_max))
            
            if x_max > x_min and y_max > y_min:  # Проверяем валидность
                boxes.append([x_min, y_min, x_max, y_max])
                confidences.append(float(confidence))
                class_ids.append(int(class_id))
        
        # NMS (Non-Maximum Suppression)
        detections = []
        if len(boxes) > 0:
            indices = cv2.dnn.NMSBoxes(
                boxes,
                confidences,
                self.confidence_threshold,
                self.iou_threshold
            )
            
            if len(indices) > 0:
                for i in indices.flatten():
                    detections.append({
                        'box': boxes[i],
                        'confidence': confidences[i],
                        'class_id': class_ids[i]
                    })
        
        return detections
    
    def draw_boxes(self, image, detections):
        """Рисуем найденные объекты на изображении"""
        frame = image.copy()
        for det in detections:
            x_min, y_min, x_max, y_max = det['box']
            conf = det['confidence']
            class_id = det['class_id']

            x_min = max(0, int(x_min))
            y_min = max(0, int(y_min))
            x_max = min(frame.shape[1], int(x_max))
            y_max = min(frame.shape[0], int(y_max))

            # Рисуем прямоугольник
            color = (0, 255, 0)
            cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), color, 2)

            # Рисуем текст с классом и confidence
            label = f"{self.class_names[class_id]}: {conf:.2f}"
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(frame, label, (x_min, y_min - 10), font, 0.5, color, 2)
        return frame
    def image_callback(self, msg):
        """Callback для обработки изображения"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w = frame.shape[:2]

            blob, scale, offsets = self.preprocess(frame)

            outputs = self.session.run(
                self.output_names,
                {self.input_name: blob}
            )

            detections = self.postprocess(outputs, scale, offsets, frame.shape)

            bbox_array = BoundingBoxArray()
            bbox_array.header = msg.header

            for det in detections:
                x_min, y_min, x_max, y_max = det['box']

                # Конвертируем из (x_min, y_min, x_max, y_max) 
                # в (center_x, center_y, size_x, size_y)
                center_x = (x_min + x_max) / 2.0
                center_y = (y_min + y_max) / 2.0
                size_x = x_max - x_min
                size_y = y_max - y_min
                area = size_x * size_y

                bbox = BoundingBox()
                bbox.center_x = float(center_x)
                bbox.center_y = float(center_y)
                bbox.size_x = float(size_x)
                bbox.size_y = float(size_y)
                bbox.area = float(area)
                bbox.confidence = det['confidence']
                bbox.class_name = self.class_names[det['class_id']]

                bbox_array.boxes.append(bbox)

            self.bbox_publisher.publish(bbox_array)

            # Рисуем и публикуем изображение
            frame_with_boxes = self.draw_boxes(frame, detections)
            image_msg = self.bridge.cv2_to_imgmsg(frame_with_boxes, encoding='bgr8')
            image_msg.header = msg.header
            self.image_publisher.publish(image_msg)

            self.get_logger().debug(
                f'Обнаружено объектов: {len(detections)}'
            )
        except Exception as e:
            self.get_logger().error(f'Ошибка при обработке изображения: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()