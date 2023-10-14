
import os

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    param_config = PathJoinSubstitution(
        [FindPackageShare('cube_navigation'), 'config', 'depth2scan.yaml']
    )
    return LaunchDescription([
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            remappings=[('depth', '/camera/depth/image_rect_raw'),
                        ('depth_camera_info', '/camera/depth/camera_info'),
                        ('scan', '/depthscan')],
            parameters=[param_config])
    ])