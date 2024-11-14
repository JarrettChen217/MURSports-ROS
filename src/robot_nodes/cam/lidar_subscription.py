import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import numpy as np

class LidarSubscriber(Node):

    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/lidar_beam',  # LIDAR topic name
            self.lidar_callback,
            10)
        self.subscription  # avoid python garbage collection
        self.figure, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        plt.ion()
        plt.show()

    def lidar_callback(self, msg):
        # get lidar datas.
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        ranges = np.array(msg.ranges)

        # replace noises by 0
        ranges[ranges == np.inf] = 0

        # clear prev data.
        self.ax.clear()

        # set up locations
        self.ax.set_title("LIDAR Data Visualization")
        self.ax.set_theta_offset(np.pi / 2)  # set start angle, front is 0 degree
        self.ax.set_theta_direction(-1)  # clockwise span

        # draw LIDAR data flow.
        self.ax.plot(angles, ranges)

        # refresh image
        plt.draw()
        plt.pause(0.001)  # keep image display

def main(args=None):
    rclpy.init(args=args)
    lidar_subscriber = LidarSubscriber()

    try:
        rclpy.spin(lidar_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        lidar_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()