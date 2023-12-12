import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int64

class NavFeedbackNode(Node):

    def __init__(self):
        super().__init__('nav_feedback_node')
        # Subscriber
        self.subscription = self.create_subscription(
            Int64,
            '/nav_feedback_',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.feedback = Int64()

        self.pub = False

        # Publisher
        self.publisher = self.create_publisher(Int64, '/test_nav', 10)
        self.publisher_ = self.create_publisher(Int64, '/nav_feedback', 10)


        self.get_logger().info('Started nav_feedback_')

    def listener_callback(self, msg):
        self.get_logger().info('Received feedback: "%s"' % msg.data)
        self.publisher_.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    nav_feedback_node = NavFeedbackNode()

    while True:
            if nav_feedback_node.pub:
                nav_feedback_node.publisher.publish(nav_feedback_node.feedback)
                nav_feedback_node.pub = False
                nav_feedback_node.get_logger().info('Data published !!')
            rclpy.spin_once(nav_feedback_node)
    # rclpy.spin(nav_feedback_node)
    nav_feedback_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()