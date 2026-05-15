from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_yolo', default_value='false',
            description='Use YOLOv8 model (true) or CV-simulation mode (false)'),

        DeclareLaunchArgument(
            'yolo_model_path', default_value='yolov8n.pt',
            description='Path to the YOLOv8 model weights'),

        DeclareLaunchArgument(
            'confidence_threshold', default_value='0.35',
            description='Minimum confidence threshold for detections'),

        DeclareLaunchArgument(
            'inference_rate', default_value='5.0',
            description='Max inference rate in Hz'),

        Node(
            package='vision_pkg',
            executable='defect_detection_node',
            name='defect_detection_node',
            output='screen',
            parameters=[{
                'use_yolo': LaunchConfiguration('use_yolo'),
                'yolo_model_path': LaunchConfiguration('yolo_model_path'),
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                'inference_rate': LaunchConfiguration('inference_rate'),
            }]
        ),
    ])
