import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rpyc import Service
from rpyc.utils.server import ThreadedServer

class RobotController(Node, Service):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_vel = Twist()

        # 初始化线速度和角速度
        self.linear_control = 0.0
        self.angular_control = 0.0

        # 速度常量
        self.max_linear = 4.0
        self.max_angular = 6.0

        # 定时器，定期发布cmd_vel
        # self.timer = self.create_timer(0.1, self.publish_cmd_vel)  # 10Hz

    def publish_cmd_vel(self):
        # 将控制信号映射到具体速度
        linear_speed = self.linear_control * self.max_linear
        angular_speed = self.angular_control * self.max_angular

        # 更新Twist消息
        self.cmd_vel.linear.x = linear_speed
        self.cmd_vel.angular.z = angular_speed

        # 发布Twist消息
        self.publisher_.publish(self.cmd_vel)
        self.get_logger().info(f'Publishing: linear_x={self.cmd_vel.linear.x}, angular_z={self.cmd_vel.angular.z}')

    # 暴露给客户端的控制方法
    def exposed_set_control(self, linear, angular):
        # 确保输入在 -1 到 1 范围内
        self.linear_control = max(-1.0, min(1.0, linear))
        self.angular_control = max(-1.0, min(1.0, angular))
        self.get_logger().info(f'Set control: linear={self.linear_control}, angular={self.angular_control}')

        # publish the result to twist
        self.publish_cmd_vel()

    def shutdown(self):
        print("shutdowned")

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()

    # 创建线程运行ROS节点
    import threading
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # 启动RPyC服务器
    server = ThreadedServer(node, port=18859, hostname='0.0.0.0')
    print("RPyC Server is listening on port 18859...")
    try:
        server.start()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
