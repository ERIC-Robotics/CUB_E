# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseStamped

# class GoalPoseSubscriber(Node):

#     def __init__(self):
#         super().__init__('goal_pose_subscriber')
#         self.subscription = self.create_subscription(
#             PoseStamped,
#             '/goal_pose',
#             self.listener_callback,
#             10)
#         self.subscription  # prevent unused variable warning

#     def listener_callback(self, msg):
#         self.get_logger().info('Received goal pose: "{0}"'.format(msg))

# def main(args=None):
#     rclpy.init(args=args)
#     goal_pose_subscriber = GoalPoseSubscriber()
#     rclpy.spin(goal_pose_subscriber)
#     goal_pose_subscriber.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()



import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped

class PointToPosePublisher(Node):

    def __init__(self):
        super().__init__('point_to_pose_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose.position = msg.point
        pose_msg.pose.orientation.w = 1.0  # Assuming no orientation, quaternion (0,0,0,1)
        

        self.publisher_.publish(pose_msg)
        self.get_logger().info('Publishing: "{0}"'.format(pose_msg))

def main(args=None):
    rclpy.init(args=args)
    point_to_pose_publisher = PointToPosePublisher()
    rclpy.spin(point_to_pose_publisher)
    point_to_pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
