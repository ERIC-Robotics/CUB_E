import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes

class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        self.action_client = self.create_action_client(NavigateToPose, 'navigate_to_pose')
        self.send_goal()

    def send_goal(self):
        self.wait_for_action_server()
        goal_1 = PoseStamped()
        # Set your goal coordinates for goal 1
        goal_1.pose.position.x = -2.0
        goal_1.pose.position.y = -2.0
        goal_1.pose.orientation.w = 1.0  # Facing forward

        goal_2 = PoseStamped()
        # Set your goal coordinates for goal 2
        goal_2.pose.position.x = -2.0
        goal_2.pose.position.y = 2.0
        goal_2.pose.orientation.w = 1.0  # Facing forward

        self.send_goal_request(goal_1)
        self.wait_for_goal_completion()

        # Once goal 1 is reached, send goal 2
        self.send_goal_request(goal_2)
        self.wait_for_goal_completion()

    def send_goal_request(self, goal_pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.action_client.wait_for_server()
        self.action_client.send_goal(goal_msg)

    def wait_for_goal_completion(self):
        self.get_logger().info('Waiting for goal to complete...')
        while not self.action_client.wait_for_result():
            rclpy.spin_once(self)

        result = self.action_client.get_result()
        if result.result == 0:
            self.get_logger().info('Goal achieved!')
        else:
            self.get_logger().warn('Goal failed with result code: {}'.format(result.result))

    def wait_for_action_server(self):
        self.get_logger().info('Waiting for action server...')
        while not self.action_client.wait_for_server():
            rclpy.spin_once(self)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationClient()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
