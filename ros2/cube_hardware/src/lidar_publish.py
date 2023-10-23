import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from builtin_interfaces.msg import Time

class ScanAndPublishNode(Node):

    def __init__(self):
        super().__init__('scan_and_publish')
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/ydlidar/scan',
            self.scan_callback,
            10  # QoS profile depth
        )
        self.newscan_publisher = self.create_publisher(
            LaserScan,
            '/scan',
            10  # QoS profile depth
        )
        self.newscan = LaserScan()

    def scan_callback(self, scan_msg):
        # Simply republish the received scan data
        self.newscan = scan_msg
        print(scan_msg.header.stamp.sec)
        self.newscan.header.stamp = self.get_clock().now().to_msg()
        self.newscan_publisher.publish(self.newscan)

def main(args=None):
    rclpy.init(args=args)
    scan_and_publish_node = ScanAndPublishNode()

    try:
        rclpy.spin(scan_and_publish_node)
    except KeyboardInterrupt:
        pass

    scan_and_publish_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()