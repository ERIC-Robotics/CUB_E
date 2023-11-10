import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

class NavigationGoalPublisher(Node):
    def __init__(self):
        super().__init__('navigation_goal_publisher')
        self.nav_goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.nav_goal_1 = self.create_pose_goal(1.0, 1.0, 0.0)  # Replace with your coordinates
        self.nav_goal_2 = self.create_pose_goal(2.0, 2.0, 0.0)  # Replace with your coordinates
        self.timer = self.create_timer(5.0, self.publish_goals)

    def create_pose_goal(self, x, y, z):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        return pose

    def publish_goals(self):
        self.get_logger().info('Publishing Navigation Goals')

        # Publish the first navigation goal
        self.nav_goal_publisher.publish(self.nav_goal_1)
        self.get_logger().info('Published Goal 1')

        # Wait for the first goal to be reached
        while rclpy.ok():
            self.spin_once()
            if self.get_goal_status() == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Goal 1 Reached')
                break

        # Publish the second navigation goal
        self.nav_goal_publisher.publish(self.nav_goal_2)
        self.get_logger().info('Published Goal 2')

        # Wait for the second goal to be reached
        while rclpy.ok():
            self.spin_once()
            if self.get_goal_status() == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Goal 2 Reached')
                break

    def get_goal_status(self):
        # Implement a service or action client to get the current goal status
        # For simplicity, you can use a placeholder function for now.
        # Replace this function with the actual implementation.
        return GoalStatus.STATUS_SUCCEEDED

    def spin_once(self):
        rclpy.spin_once(self)

def main(args=None):
    rclpy.init(args=args)
    navigation_goal_publisher = NavigationGoalPublisher()
    rclpy.spin(navigation_goal_publisher)
    navigation_goal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
