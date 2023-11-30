import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String

class NavigateToGoalClient:
    def __init__(self):
        self.node = rclpy.create_node('navigate_to_goal_client')
        self.action_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        self.feedback_publisher = self.node.create_publisher(String, 'nav_feedback', 10)

        self.send_goal()
        # self.feedback_publisher.publish(String(data="Succeeded"))
        self.published = False

    def send_goal(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'  # Set the frame_id according to your map frame
        goal_msg.pose.pose.position.x = -2.0  # Set the x-coordinate of the goal pose
        goal_msg.pose.pose.position.y = -3.0  # Set the y-coordinate of the goal pose
        goal_msg.pose.pose.orientation.w = 1.0  # Set the orientation of the goal pose

        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback, )
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        # print(f'Received feedback: {feedback_msg.feedback}')
        # print()
        # print(feedback_msg.feedback)
        # print(feedback_msg)
        if not self.published:
            self.feedback_publisher.publish(String(data="Executing"))
            self.published = True
        pass

    def goal_response_callback(self, future):
        goal_handle = future.result()
        print(goal_handle)
        if not goal_handle.accepted:
            print('Goal was rejected')
            self.feedback_publisher.publish(String(data="Aborted"))
            return

        print('Goal was accepted. Waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        print(result)
        # 0 — Unknown.
        # 1 — Accepted.
        # 2 — Executing.
        # 3 — Canceling.
        # 4 — Succeeded.
        # 5 — Canceled.
        # 6 — Aborted.    
        if result.status == 4:
            print('Goal was successful!')
            self.feedback_publisher.publish(String(data="Succeeded"))
            
        else:
            print(f'Goal failed with result code: {result}')
            self.feedback_publisher.publish(String(data="Aborted"))
        
        self.published = False
        
        # Shutdown the node after receiving the result
        self.node.get_logger().info('Shutting down...')
        self.node.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    navigate_to_goal_client = NavigateToGoalClient()
    rclpy.spin(navigate_to_goal_client.node)

if __name__ == '__main__':
    main()