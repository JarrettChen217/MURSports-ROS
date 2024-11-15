import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
from rpyc import Service
from rpyc.utils.server import ThreadedServer

class LidarSubscriber(Node, Service):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/lidar_beam',  # LIDAR topic name
            self.lidar_callback,
            10)
        self.angles = None
        self.ranges = None

    def lidar_callback(self, msg):
        # 获取LIDAR数据
        self.angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        self.ranges = np.array(msg.ranges)
        
        # 将无穷大值替换为0，表示噪声
        self.ranges[self.ranges == np.inf] = 0

    # 暴露给客户端的获取LIDAR数据的方法
    def exposed_get_lidar_data(self):
        return self.angles, self.ranges

def main(args=None):
    rclpy.init(args=args)
    lidar_node = LidarSubscriber()

    # 创建ROS节点的线程
    import threading
    ros_thread = threading.Thread(target=rclpy.spin, args=(lidar_node,), daemon=True)
    ros_thread.start()

    # 启动RPyC服务器
    server = ThreadedServer(lidar_node, port=18858, hostname='0.0.0.0')
    print("RPyC Server is listening on port 18858...")
    try:
        server.start()
    except KeyboardInterrupt:
        pass
    finally:
        lidar_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
