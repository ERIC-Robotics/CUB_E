#!/usr/bin/python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_prefix

def generate_launch_description():

  pkg_ydlidar = get_package_share_directory('ydlidar_ros2_driver')

  
  lidar = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(pkg_ydlidar, 'launch', 'ydlidar_launch.py'),
    )
  )

  ld = LaunchDescription()

  ld.add_action(lidar)

  return ld
  

