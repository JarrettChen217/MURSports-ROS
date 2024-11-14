import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

# import rpyc for sending through RPC
from rpyc import Service
from rpyc.utils.server import ThreadedServer

class OdomSubscriber(Node, Service):
    def __init__(self):
        Node.__init__(self, 'odom_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.timer = self.create_timer(1.0, self.read_linear_speed)  # 1Hz

    def odom_callback(self, msg):
        # 从 "/odom" 消息中提取线速度
        self.linear_speed = msg.twist.twist.linear.x
        self.angular_speed = msg.twist.twist.angular.z

    def read_linear_speed(self):
        # 打印当前线速度
        self.get_logger().info(f'Current Linear Speed: {self.linear_speed:.2f} m/s\nCurrent Angular Speed: {self.angular_speed:.2f} m/s')

    def exposed_read_linear_speed(self):
        # 使客户端能够调用此方法以获取当前的线速度数据
        return self.linear_speed, self.angular_speed

def main(args=None):
    rclpy.init(args=args)
    odom_node = OdomSubscriber()

    # 创建一个线程来运行ROS节点
    import threading
    ros_thread = threading.Thread(target=rclpy.spin, args=(odom_node,), daemon=True)
    ros_thread.start()

    # 启动RPyC服务器
    server = ThreadedServer(odom_node, port=18861, hostname='0.0.0.0')
    print("RPyC Server is listening on port 18861...")
    try:
        server.start()
    except KeyboardInterrupt:
        pass
    finally:
        odom_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
