import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time

class OdomSubscriber(Node):

    def __init__(self):
        super().__init__('odom_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.linear_speed = 0.0
        self.timer = self.create_timer(1.0, self.read_linear_speed)  # 1Hz

    def odom_callback(self, msg):
        # Extracting line speed from “/odom” messages
        self.linear_speed = msg.twist.twist.linear.x
        self.angular_speed = msg.twist.twist.angular.z

    def read_linear_speed(self):
        # Print or record the current line speed
        self.get_logger().info(f'Current Linear Speed: {self.linear_speed:.2f} m/s\nCurrent Angular Speed: {self.angular_speed:.2f} m/s')

def main(args=None):
    rclpy.init(args=args)
    node = OdomSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()