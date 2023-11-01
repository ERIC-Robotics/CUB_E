#!/usr/bin/python3

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult, 
import rclpy

rclpy.init()

nav = BasicNavigator()

nav.cancelTask()

while True:
    result = nav.getResult()

    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')