#!/usr/bin/env python3
"""
yolo_detector.py
================
Subscribes to RGB image from camera.
Runs YOLOv8 inference.
Publishes detected bounding boxes as vision_msgs/Detection2DArray.

Subscribes:
    /camera_head/image  (sensor_msgs/Image)

Publishes:
    /detected_object/bounding_box  (vision_msgs/Detection2DArray)
    /detected_object/debug_image   (sensor_msgs/Image)
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge

import cv2
from ultralytics import YOLO


class YoloDetector(Node):

    def __init__(self):
        super().__init__('yolo_detector')

        # ── parameters ────────────────────────────────────────────
        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('device', 'cpu')

        model_path  = self.get_parameter('model').value
        self.conf   = self.get_parameter('confidence').value
        device      = self.get_parameter('device').value

        # ── YOLOv8 model ──────────────────────────────────────────
        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.model = YOLO(model_path)
        self.model.to(device)
        self.get_logger().info('YOLO model loaded')

        # ── cv_bridge ─────────────────────────────────────────────
        self.bridge = CvBridge()

        # ── subscriber ────────────────────────────────────────────
        self.sub = self.create_subscription(
            Image,
            '/camera_head/image',
            self.image_callback,
            10)

        # ── publishers ────────────────────────────────────────────
        self.det_pub = self.create_publisher(
            Detection2DArray,
            '/detected_object/bounding_box',
            10)

        self.debug_pub = self.create_publisher(
            Image,
            '/detected_object/debug_image',
            10)

        self.get_logger().info('YoloDetector node ready')

    def image_callback(self, msg: Image):

        # 1. Convert ROS Image → numpy array (BGR for OpenCV)
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 2. Run YOLO inference
        results = self.model.predict(
            source=frame,
            conf=self.conf,
            verbose=False)

        # 3. Build Detection2DArray message
        det_array = Detection2DArray()
        det_array.header = msg.header  # same timestamp as image

        detections = results[0].boxes  # all boxes from first (only) image

        if detections is not None and len(detections) > 0:
            for box in detections:
                # box.xyxy → [x1, y1, x2, y2] in pixels
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                conf  = float(box.conf[0])
                cls   = int(box.cls[0])
                label = self.model.names[cls]

                # Build single Detection2D
                det = Detection2D()
                det.header = msg.header

                # Bounding box centre and size
                det.bbox.center.position.x = (x1 + x2) / 2.0
                det.bbox.center.position.y = (y1 + y2) / 2.0
                det.bbox.size_x = float(x2 - x1)
                det.bbox.size_y = float(y2 - y1)

                # Class and confidence
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = label
                hyp.hypothesis.score    = conf
                det.results.append(hyp)

                det_array.detections.append(det)

                # Draw box on frame for debug image
                cv2.rectangle(frame,
                              (int(x1), int(y1)),
                              (int(x2), int(y2)),
                              (0, 255, 0), 2)
                cv2.putText(frame,
                            f'{label} {conf:.2f}',
                            (int(x1), int(y1) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 255, 0), 2)

        else:
            self.get_logger().debug('No detections in this frame')

        # 4. Publish detections
        self.det_pub.publish(det_array)

        # 5. Publish debug image
        debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()