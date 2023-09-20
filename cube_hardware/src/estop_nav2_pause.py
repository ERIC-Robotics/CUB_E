#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from nav2_msgs.action._navigate_to_pose import NavigateToPose  # noqa: F401

class EStopListener(Node):
    def __init__(self):
        super().__init__('estop_listener')
        self.nav_cancel_client = self.create_client(NavigateToPose, '/navigate_to_pose/cancel')  # Adjust the action name as needed
        self.es_status_sub = self.create_subscription(Int32, '/es_status', self.es_status_callback, 10)

    def es_status_callback(self, msg):
        es_status = msg.data
        if es_status == 1:
            # Perform action to cancel navigation
            self.cancel_navigation()

    def cancel_navigation(self):
        if not self.nav_cancel_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Navigation cancel service not available.')
            return

        cancel_request = NavigateToPose.Goal()
        cancel_goal_handle = self.nav_cancel_client.send_goal_async(cancel_request)
        rclpy.spin_until_future_complete(self, cancel_goal_handle)

def main(args=None):
    rclpy.init(args=args)
    estop_listener = EStopListener()
    rclpy.spin(estop_listener)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
