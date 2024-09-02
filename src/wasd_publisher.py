import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading

class CmdVelPublisher(Node):

    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_vel = Twist()  # initialize the twist.
        self.get_logger().info('CmdVel Publisher Node has been started.')
        
        # start an listening thread.
        thread = threading.Thread(target=self.listen_to_input)
        thread.daemon = True
        thread.start()

    def publish_cmd_vel(self):
        self.publisher_.publish(self.cmd_vel)
        self.get_logger().info(f'Publishing: {self.cmd_vel}')

    def listen_to_input(self):
        while rclpy.ok():
            try:
                user_input = input("Enter command (w/a/s/d for movement, x to stop): ").strip().lower()
                
                if user_input == 'w':
                    self.cmd_vel.linear.x += 0.1  # increase linear speed
                elif user_input == 's':
                    self.cmd_vel.linear.x -= 0.1  # reduce linear speed
                elif user_input == 'a':
                    self.cmd_vel.angular.z += 0.1  # turn left
                elif user_input == 'd':
                    self.cmd_vel.angular.z -= 0.1  # turn right
                elif user_input == 'x':
                    self.cmd_vel.linear.x = 0.0
                    self.cmd_vel.angular.z = 0.0
                else:
                    print("Unknown command. Use w/a/s/d for movement, x to stop.")

                self.publish_cmd_vel()  # publish the modified info to publisher.
                
            except Exception as e:
                self.get_logger().error(f'Error: {e}')
    
def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()