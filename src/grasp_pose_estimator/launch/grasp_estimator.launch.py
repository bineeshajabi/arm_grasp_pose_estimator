#!/usr/bin/env python3
"""
grasp_estimator.launch.py
=========================
Launches the full grasp pose estimation pipeline:
  - yolo_detector     : detects objects in RGB image
  - depth_processor   : computes 3D grasp pose from depth + bounding box

Usage:
  ros2 launch grasp_pose_estimator grasp_estimator.launch.py
  ros2 launch grasp_pose_estimator grasp_estimator.launch.py confidence:=0.3
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    # ── launch arguments ──────────────────────────────────────────
    confidence_arg = DeclareLaunchArgument(
        'confidence',
        default_value='0.5',
        description='YOLO detection confidence threshold (0.0-1.0)')

    model_arg = DeclareLaunchArgument(
        'model',
        default_value='yolov8n.pt',
        description='YOLOv8 model weights file')

    gripper_width_arg = DeclareLaunchArgument(
        'gripper_width',
        default_value='0.08',
        description='Gripper width in metres for grasp scoring')

    # ── nodes ─────────────────────────────────────────────────────
    yolo_detector_node = Node(
        package='grasp_pose_estimator',
        executable='yolo_detect',
        name='yolo_detector',
        output='screen',
        parameters=[{
            'confidence': LaunchConfiguration('confidence'),
            'model':      LaunchConfiguration('model'),
            'device':     'cpu',
        }]
    )

    depth_processor_node = Node(
        package='grasp_pose_estimator',
        executable='depth_process',
        name='depth_processor',
        output='screen',
        parameters=[{
            'gripper_width':          LaunchConfiguration('gripper_width'),
            'min_grasp_score':        0.4,
            'bilateral_d':            9,
            'bilateral_sigma_color':  0.1,
            'bilateral_sigma_space':  10.0,
        }]
    )

    #RViz node 
    rviz_grasp_node=Node(
		 executable ='rviz2',
		 name       ='rviz2_arm_grasp',
		 output     ='screen',
		 arguments  =['-d', PathJoinSubstitution([FindPackageShare('grasp_pose_estimator'),'rviz','grasp.rviz'])
    ])
    

    return LaunchDescription([
        confidence_arg,
        model_arg,
        gripper_width_arg,
        yolo_detector_node,
        depth_processor_node,
        rviz_grasp_node
    ])