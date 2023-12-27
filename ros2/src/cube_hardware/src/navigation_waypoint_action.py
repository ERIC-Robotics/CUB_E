import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int64
import tkinter as tk

class NavigateThroughPosesClient(Node):

    def __init__(self):
        super().__init__('navigate_through_poses_client')
        self._action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose_rviz', 
            self.goal_pose_callback,
            10
        )
        self.feedback_publisher = self.create_publisher(Int64, 'nav_feedback_', 10)
        self.feedback_publisher_timer = self.create_timer(0.5, self.feedback_publisher_timer_pub)
        self.waypoints = []
        self.nav_status = Int64()
        self.published = False

    def feedback_publisher_timer_pub(self):
        self.feedback_publisher.publish(self.nav_status)

    def goal_pose_callback(self, msg):
        self.waypoints.append(msg)
        print('Added to list')

    def send_goal(self, waypoints):
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = waypoints

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.nav_status.data = 0
            self.feedback_publisher.publish(self.nav_status)
            return
        
        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # result = future.result().result
        # self.get_logger().info('Result: {0}'.format(result))

        result = future.result()
        if result.status == 4:
            print('Goal was successful!')
            self.nav_status.data = 1
            self.feedback_publisher.publish(self.nav_status)
            
        else:
            print(f'Goal failed with result code: {result}')
            self.nav_status.data = 0
            self.feedback_publisher.publish(self.nav_status)
        
        self.published = False
        
        # Shutdown the node after receiving the result
        self.get_logger().info('Done...')

        rclpy.spin(self)

        self.destroy_node()
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        if not self.published:
            self.nav_status.data = 2
            self.feedback_publisher.publish(self.nav_status)
            self.published = True
        pass

def read_input(action_client):
    # Initialize Tkinter
    root = tk.Tk()
    root.geometry("200x100")
    root.title("Control Window")

    # Variable to track the button press
    button_pressed = False

    def on_button_click():
        nonlocal button_pressed
        button_pressed = True
        root.destroy()  

    button = tk.Button(root, text="Start WayPoints", command=on_button_click)
    button.pack(pady=20)

    def process_tkinter_events():
        root.update_idletasks()
        root.update()

    process_tkinter_events()

    while True:
        rclpy.spin_once(action_client, timeout_sec=0.1)
        
        process_tkinter_events()

        if button_pressed:
            print("Button pressed, exiting loop.")
            break




def main(args=None):
    rclpy.init(args=args)
    action_client = NavigateThroughPosesClient()
    print('Taking Input')
    read_input(action_client=action_client)
    print('Took Input')
    action_client.send_goal(action_client.waypoints)
    rclpy.spin(action_client)
    try:
        action_client.destroy_node()
        rclpy.shutdown()
    except:
        pass

if __name__ == '__main__':
    main()
