import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_vel = Twist()
        self.active_keys = set()
        self.timer = self.create_timer(0.1, self.publish_cmd_vel)  # 10Hz

        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

    def publish_cmd_vel(self):
        if 'w' in self.active_keys:
            self.cmd_vel.linear.x = 2.0
        elif 's' in self.active_keys:
            self.cmd_vel.linear.x = -2.0
        else:
            self.cmd_vel.linear.x = 0.0

        if 'a' in self.active_keys:
            self.cmd_vel.angular.z = 3.0
        elif 'd' in self.active_keys:
            self.cmd_vel.angular.z = -3.0
        else:
            self.cmd_vel.angular.z = 0.0 

        self.publisher_.publish(self.cmd_vel)
        self.get_logger().info(f'Publishing: linear_x={self.cmd_vel.linear.x}, angular_z={self.cmd_vel.angular.z}')

    def on_press(self, key):
        try:
            if key.char in ['w', 'a', 's', 'd']:
                self.active_keys.add(key.char)
        except AttributeError:
            pass

    def on_release(self, key):
        try:
            if key.char in self.active_keys:
                self.active_keys.remove(key.char)

            if not self.active_keys:
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0
                self.publisher_.publish(self.cmd_vel)
                self.get_logger().info('Stopping robot: All keys released.')

            if key == keyboard.Key.esc:
                return False

        except AttributeError:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()