import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Imu
from std_msgs.msg import Int64
import tf_transformations

class ImuSubscriber(Node):

    def __init__(self):
        super().__init__('inclination_saftey')
        self.subscription = self.create_subscription(
            Imu,
            '/bno055/imu',
            self.imu_callback,
            10)
        self.publisher = self.create_publisher(Int64, '/es_status/software/control', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.latest_pitch = 0
        self.latest_yaw = 0

    def imu_callback(self, msg):
        quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w)
        euler = tf_transformations.euler_from_quaternion(quaternion)
        self.latest_pitch = euler[0]
        self.latest_yaw = euler[1]

    def timer_callback(self):
        control_msg = Int64()
        if abs(self.latest_pitch) > 25 or abs(self.latest_yaw) > 25:
            control_msg.data = 1
        else:
            control_msg.data = 0
            
        self.publisher.publish(control_msg)

def main(args=None):
    rclpy.init(args=args)
    imu_subscriber = ImuSubscriber()
    rclpy.spin(imu_subscriber)
    imu_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
