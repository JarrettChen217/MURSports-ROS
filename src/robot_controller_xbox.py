import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame
import time

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_vel = Twist()

        # initialize pygame.
        pygame.init()
        pygame.joystick.init()

        # confirm the joystick is connected.
        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No joystick detected!")
        else:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.get_logger().info(f"Joystick detected: {self.joystick.get_name()}")

        # a timer, call publish_cmd_vel() at a fixed time.
        self.timer = self.create_timer(0.1, self.publish_cmd_vel)  # 10Hz
        
        # constant of max velocity.
        self.max_linear = 4.0
        self.max_angular = 6.0

    def publish_cmd_vel(self):
        pygame.event.pump()  # update pygame queue.
        linear_speed = 0.0
        angular_speed = 0.0

        right_trigger = self.joystick.get_axis(5)  # RT button
        right_trigger += 1
        if(right_trigger > 0.01):
            linear_speed += self.max_linear*(right_trigger/2)
            # print(f"RT moved to {right_trigger}")
        left_trigger = self.joystick.get_axis(2)
        left_trigger += 1
        if(left_trigger > 0.01):
            linear_speed -= self.max_linear*(left_trigger/2)
            # print(f"LT moved to {left_trigger}")

        left_stick_x = self.joystick.get_axis(0)
        if(left_stick_x < -0.1):
            angular_speed += self.max_angular*left_stick_x
            # print(f"turn left {-left_stick_x}")
        elif(left_stick_x > 0.1):
            angular_speed += self.max_angular*left_stick_x
            # print(f"turn right {left_stick_x}")

        assert linear_speed > (-self.max_linear - 0.1)
        assert linear_speed < (self.max_linear + 0.1)

        assert angular_speed > (-self.max_angular - 0.1)
        assert angular_speed < (self.max_angular + 0.1)

        self.cmd_vel.linear.x = linear_speed
        if(linear_speed >= 0):
            self.cmd_vel.angular.z = -angular_speed
        else:
            # reversing angular in negative.
            self.cmd_vel.angular.z = angular_speed
        # publish the twist msg.
        self.publisher_.publish(self.cmd_vel)
        self.get_logger().info(f'Publishing: linear_x={self.cmd_vel.linear.x}, angular_z={self.cmd_vel.angular.z}')

    def shutdown(self):
        print("shutdowned")
        pygame.quit()  # shutdown pygame.

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
