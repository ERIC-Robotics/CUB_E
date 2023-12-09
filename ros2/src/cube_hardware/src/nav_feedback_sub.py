import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class NavFeedbackNode(Node):

    def __init__(self):
        super().__init__('nav_feedback_node')
        # Subscriber
        self.subscription = self.create_subscription(
            String,
            '/nav_feedback',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Publisher
        self.publisher = self.create_publisher(String, '/nav_feedback', 10)
        self.get_logger().info('Started nav_feedback_sub')

    def listener_callback(self, msg):
        self.get_logger().info('Received feedback: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    nav_feedback_node = NavFeedbackNode()
    rclpy.spin(nav_feedback_node)
    nav_feedback_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
