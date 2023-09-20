#!/usr/bin/python3

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy


def main():
    rclpy.init()

    nav = BasicNavigator()

    nav.waitUntilNav2Active()
    
    nav.cancelTask()

    while not nav.isTaskComplete():
        pass

    result = nav.getResult()

    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')

if __name__ == '__main__':
    main()