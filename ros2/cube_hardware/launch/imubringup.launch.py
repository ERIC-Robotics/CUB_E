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

  pkg_bno055 = get_package_share_directory('bno055')

  imu = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(pkg_bno055, 'launch', 'bno055.launch.py'),
    )
  ) 

  ld = LaunchDescription()

  ld.add_action(imu)

  return ld
  

