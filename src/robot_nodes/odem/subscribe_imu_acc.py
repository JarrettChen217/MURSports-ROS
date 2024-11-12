import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge
import cv2

class ImuSubscriber(Node):

    def __init__(self):
        super().__init__('imu_cam_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            '/front_stereo_camera/imu/data',
            self.imu_callback,
            10
        )
        self.bridge = CvBridge()  # For converting ROS image messages to Open CV format
        
        self.subscription
        self.get_logger().info('Imu Camera Subscriber Node has been started.')

    # def image_callback(self, msg):
    #     # Convert ROS image messages to OpenCV image format
    #     cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    #     if cv_image is None:
    #         print("Empty image!!!")

    #     # Display the image in the window.
    #     cv2.imshow("Imu Cam Image", cv_image)
    #     cv2.waitKey(1)  # Used to refresh the window so that the image display is updated

    #     # Images can also be saved or further processed
    #     # cv2.imwrite('frame.jpg', cv_image)

    # def depth_callback(self, msg):
    #     depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    #     depth_image_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)

    #     depth_colormap = cv2.applyColorMap(depth_image_normalized.astype(np.uint8), cv2.COLORMAP_JET)

    #     cv2.imshow("Imu Depth Image", depth_colormap)
    #     cv2.waitKey(1)

    def imu_callback(self, msg):
        print("I heard ", msg.linear_acceleration)

def main(args=None):
    rclpy.init(args=args)
    node = ImuSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()