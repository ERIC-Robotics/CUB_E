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

  odometry = Node(
    package='cube_hardware',
    executable='odometry',
    output='screen'
  )

  cmdmotor = Node(
    package='cube_hardware',
    executable='cmdmotor',
    output='screen'
  )

  transform = Node(
    package='cube_hardware',
    executable='transform',
    output='screen'
  )

  ld = LaunchDescription()

  ld.add_action(odometry)
  ld.add_action(cmdmotor)
  ld.add_action(transform)

  return ld
  

