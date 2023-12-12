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
  pkg_ydlidar = get_package_share_directory('ydlidar_ros2_driver')
  pkg_cube_description = get_package_share_directory('cube_description')
  pkg_cube_navigation = get_package_share_directory('cube_navigation')
  
  robot_localization_file_path = os.path.join(pkg_cube_navigation, 'config/ekf.yaml') 

  state_pub = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(pkg_cube_description, 'launch', 'publish_urdf.launch.py'),
    )
  )

  imu = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(pkg_bno055, 'launch', 'bno055.launch.py'),
    )
  ) 
  
  lidar = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(pkg_ydlidar, 'launch', 'ydlidar_launch.py'),
    )
  )

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

  robot_localization_node = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[robot_localization_file_path]
  )

  # lidarpub = Node(
  #   package='cube_hardware',
  #   executable='lidarpub',
  #   output='screen'
  # )

  lidar_min = Node(
    package='cube_hardware',
    executable='lidar_min',
    output='screen'
  )

  lidar_saftey = Node(
    package='cube_hardware',
    executable='lidar_saftey',
    output='screen'
  )

  nav_feedback_sub = Node(
    package='cube_hardware',
    executable='nav_feedback_sub',
    output='screen'
  )


  ld = LaunchDescription()

  ld.add_action(state_pub)
  ld.add_action(odometry)
  ld.add_action(cmdmotor)
  ld.add_action(transform)
  ld.add_action(lidar)
  ld.add_action(lidar_min)
  ld.add_action(lidar_saftey)
  ld.add_action(imu)
  ld.add_action(nav_feedback_sub)
  ld.add_action(robot_localization_node)

  return ld
  

