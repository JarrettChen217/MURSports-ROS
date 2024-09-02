import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelPublisher(Node):

    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.publish_cmd_vel)
        self.get_logger().info('CmdVel Publisher Node has been started.')

    def publish_cmd_vel(self):
        msg = Twist()
        msg.linear.x = 0.5  # 设置线速度，正值表示向前移动
        msg.angular.z = 1.0  # 设置角速度，正值表示左转
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg}')

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()