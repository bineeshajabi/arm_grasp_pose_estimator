#!/usr/bin/env python3
"""
depth_processor.py
==================
Subscribes to depth image, camera info, and bounding box.
Computes 3D grasp pose using:
  - ROI extraction
  - Bilateral filter (depth noise removal)
  - 3D centroid from camera intrinsics
  - Surface normal via PCA
  - Grasp orientation from surface normal

Subscribes:
    /camera_head/depth_image           (sensor_msgs/Image)
    /camera_head/camera_info           (sensor_msgs/CameraInfo)
    /detected_object/bounding_box      (vision_msgs/Detection2DArray)

Publishes:
    /grasp_pose    (geometry_msgs/PoseStamped)
    /grasp_score   (std_msgs/Float32)
    /grasp_debug   (sensor_msgs/Image)
"""

import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs

import numpy as np
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from vision_msgs.msg import Detection2DArray


class DepthProcessor(Node):

    def __init__(self):
        super().__init__('depth_processor')

        # ── parameters ────────────────────────────────────────────
        self.declare_parameter('gripper_width', 0.08)
        self.declare_parameter('min_grasp_score', 0.4)
        self.declare_parameter('bilateral_d', 9)
        self.declare_parameter('bilateral_sigma_color', 0.1)
        self.declare_parameter('bilateral_sigma_space', 10.0)

        self.source_frame = 'camera_head_depth_optical_frame'
        self.target_frame = 'base_link'

        self.gripper_width   = self.get_parameter('gripper_width').value
        self.min_score       = self.get_parameter('min_grasp_score').value
        self.bil_d           = self.get_parameter('bilateral_d').value
        self.bil_sig_color   = self.get_parameter('bilateral_sigma_color').value
        self.bil_sig_space   = self.get_parameter('bilateral_sigma_space').value

        # ── state ─────────────────────────────────────────────────
        self.bridge       = CvBridge()
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.camera_info  = None   # filled once on first camera_info message
        self.latest_bbox  = None   # filled on each detection message

        # ── subscribers ───────────────────────────────────────────
        self.create_subscription(
            CameraInfo,
            '/camera_head/camera_info',
            self.camera_info_callback,
            10)

        self.create_subscription(
            Detection2DArray,
            '/detected_object/bounding_box',
            self.bbox_callback,
            10)

        self.create_subscription(
            Image,
            '/camera_head/depth_image',
            self.depth_callback,
            10)

        # ── publishers ────────────────────────────────────────────
        self.pose_pub  = self.create_publisher(PoseStamped, '/grasp_pose',  10)
        self.score_pub = self.create_publisher(Float32,     '/grasp_score', 10)
        self.debug_pub = self.create_publisher(Image,       '/grasp_debug', 10)

        self.get_logger().info('DepthProcessor node ready')

    # ── callbacks ─────────────────────────────────────────────────

    def camera_info_callback(self, msg: CameraInfo):
        """Store intrinsics once — they don't change."""
        if self.camera_info is None:
            self.camera_info = {
                'fx': msg.k[0],
                'fy': msg.k[4],
                'cx': msg.k[2],
                'cy': msg.k[5],
                'frame_id': msg.header.frame_id
            }
            self.get_logger().info(
                f'Intrinsics loaded: fx={self.camera_info["fx"]:.2f} '
                f'cx={self.camera_info["cx"]:.2f}')

    def bbox_callback(self, msg: Detection2DArray):
        """Store latest bounding box."""
        if len(msg.detections) > 0:
            self.latest_bbox = msg.detections[0]  # take highest confidence detection
        else:
            self.latest_bbox = None

    def depth_callback(self, msg: Image):
        """Main processing — runs on every depth frame."""

        # wait until we have intrinsics and a detection
        if self.camera_info is None:
            self.get_logger().warn('Waiting for camera_info...', throttle_duration_sec=5.0)
            return

        if self.latest_bbox is None:
            self.get_logger().debug('No detection yet — skipping frame')
            return

        # 1. Convert depth image to float32 numpy array (values in metres)
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        depth = depth.astype(np.float32)

        # replace zeros and inf with NaN so they're ignored in calculations
        depth[depth == 0]              = np.nan
        depth[~np.isfinite(depth)]     = np.nan

        # 2. Extract ROI from bounding box
        bbox   = self.latest_bbox.bbox
        cx_px  = bbox.center.position.x
        cy_px  = bbox.center.position.y
        half_w = bbox.size_x / 2.0
        half_h = bbox.size_y / 2.0

        h, w = depth.shape
        x1 = max(0, int(cx_px - half_w))
        x2 = min(w, int(cx_px + half_w))
        y1 = max(0, int(cy_px - half_h))
        y2 = min(h, int(cy_px + half_h))

        depth_roi = depth[y1:y2, x1:x2].copy()

        if depth_roi.size == 0:
            self.get_logger().warn('Empty ROI — bounding box outside image bounds')
            return

        # 3. Bilateral filter on depth ROI
        # replace NaN with 0 for OpenCV, filter, then restore NaN
        depth_roi_clean          = np.nan_to_num(depth_roi, nan=0.0)
        depth_filtered           = cv2.bilateralFilter(
                                       depth_roi_clean.astype(np.float32),
                                       d=self.bil_d,
                                       sigmaColor=self.bil_sig_color,
                                       sigmaSpace=self.bil_sig_space)
        depth_filtered[depth_roi_clean == 0] = np.nan

        # 4. Build 3D point cloud from ROI
        points = []
        fx = self.camera_info['fx']
        fy = self.camera_info['fy']
        cx = self.camera_info['cx']
        cy = self.camera_info['cy']

        for row in range(depth_filtered.shape[0]):
            for col in range(depth_filtered.shape[1]):
                z = depth_filtered[row, col]
                if np.isnan(z) or z <= 0.0:
                    continue
                # convert back to full image coordinates
                u = col + x1
                v = row + y1
                X = (u - cx) * z / fx
                Y = (v - cy) * z / fy
                Z = z
                points.append([X, Y, Z])

        if len(points) < 10:
            self.get_logger().warn(f'Too few valid depth points: {len(points)}')
            return

        points = np.array(points)  # shape (N, 3)

        # 5. Compute centroid
        centroid = np.mean(points, axis=0)

        # 6. Surface normal via PCA
        centred     = points - centroid
        cov         = np.cov(centred.T)
        eigenvalues, eigenvectors = np.linalg.eigh(cov)
        # smallest eigenvalue → eigenvector perpendicular to surface = normal
        normal = eigenvectors[:, 0]

        # ensure normal points toward camera (positive Z in camera frame)
        if normal[2] < 0:
            normal = -normal

        # 7. Grasp orientation — align gripper Z with surface normal
        z_axis  = np.array([0.0, 0.0, 1.0])
        axis    = np.cross(z_axis, normal)
        sin_ang = np.linalg.norm(axis)
        cos_ang = np.dot(z_axis, normal)

        if sin_ang < 1e-6:
            # normal already aligned with Z — no rotation needed
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
        else:
            axis    = axis / sin_ang          # normalise rotation axis
            angle   = np.arctan2(sin_ang, cos_ang)
            s       = np.sin(angle / 2.0)
            qx      = axis[0] * s
            qy      = axis[1] * s
            qz      = axis[2] * s
            qw      = np.cos(angle / 2.0)

        # 8. Grasp score — ratio of valid depth points in ROI
        total_pixels = depth_roi.size
        valid_pixels = np.sum(~np.isnan(depth_filtered))
        score        = float(valid_pixels) / float(total_pixels)

        # 9. Publish grasp pose
        pose_msg                       = PoseStamped()
        pose_msg.header.stamp          = msg.header.stamp
        pose_msg.header.frame_id       = self.source_frame
        pose_msg.pose.position.x       = float(centroid[0])
        pose_msg.pose.position.y       = float(centroid[1])
        pose_msg.pose.position.z       = float(centroid[2])
        pose_msg.pose.orientation.x    = qx
        pose_msg.pose.orientation.y    = qy
        pose_msg.pose.orientation.z    = qz
        pose_msg.pose.orientation.w    = qw
        try:
            pose_in_base = self.tf_buffer.transform(
                pose_msg,
                self.target_frame,
                timeout=rclpy.duration.Duration(seconds=0.1))
            self.pose_pub.publish(pose_in_base)
            self.get_logger().info(
                f'Grasp pose in base_link: '
                f'({pose_in_base.pose.position.x:.3f}, '
                f'{pose_in_base.pose.position.y:.3f}, '
                f'{pose_in_base.pose.position.z:.3f})m  '
                f'score={score:.2f}')
        except tf2_ros.LookupException:
            self.get_logger().warn('TF not ready yet', throttle_duration_sec=2.0)
            self.pose_pub.publish(pose_msg)
        except tf2_ros.ExtrapolationException:
            self.get_logger().warn('TF timestamp mismatch', throttle_duration_sec=2.0)
            self.pose_pub.publish(pose_msg)

        # 10. Publish grasp score
        score_msg       = Float32()
        score_msg.data  = score
        self.score_pub.publish(score_msg)

        # 11. Publish debug image — depth ROI visualised
        self._publish_debug(depth, x1, y1, x2, y2, centroid, score, msg.header)

    def _publish_debug(self, depth, x1, y1, x2, y2, centroid, score, header):
        """Publish colourised depth image with bounding box drawn."""

        # normalise depth to 0-255 for visualisation
        depth_vis = depth.copy()
        valid     = ~np.isnan(depth_vis)
        if valid.any():
            d_min = np.nanmin(depth_vis)
            d_max = np.nanmax(depth_vis)
            depth_vis = np.nan_to_num(depth_vis, nan=0.0)
            if d_max > d_min:
                depth_vis = ((depth_vis - d_min) / (d_max - d_min) * 255).astype(np.uint8)
            else:
                depth_vis = np.zeros_like(depth_vis, dtype=np.uint8)
        else:
            depth_vis = np.zeros(depth.shape, dtype=np.uint8)

        # colourise — closer=red, further=blue
        depth_colour = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)

        # draw bounding box
        cv2.rectangle(depth_colour, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # draw centroid projected back to pixel
        fx = self.camera_info['fx']
        fy = self.camera_info['fy']
        cx = self.camera_info['cx']
        cy = self.camera_info['cy']
        u_c = int(centroid[0] * fx / centroid[2] + cx)
        v_c = int(centroid[1] * fy / centroid[2] + cy)
        cv2.circle(depth_colour, (u_c, v_c), 8, (255, 255, 255), -1)

        # draw score text
        cv2.putText(depth_colour,
                    f'score={score:.2f}',
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 0), 2)

        debug_msg         = self.bridge.cv2_to_imgmsg(depth_colour, encoding='bgr8')
        debug_msg.header  = header
        self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()