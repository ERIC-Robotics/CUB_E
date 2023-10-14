#!/usr/bin/python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_prefix


def generate_launch_description():

  pkg_rtabmap = get_package_share_directory('rtabmap_launch')

  rtab = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(pkg_rtabmap, 'launch', 'rtabmap.launch.py'),
    ),
    remappings=[
      ("/camera/camera_info", "/camera/rgb/camera_info"),
      ("/camera/image_raw", "/camera/rgb/image_rect_color"),
      ("/camera/depth/image_raw", "/camera/depth_registered/image_raw")
    ]
  ) 
  
  return LaunchDescription([
    rtab
  ])