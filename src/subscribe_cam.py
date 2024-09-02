import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/front_stereo_camera/left_rgb/image_raw',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()  # For converting ROS image messages to Open CV format
        self.get_logger().info('Image Subscriber Node has been started.')

    def image_callback(self, msg):
        # Convert ROS image messages to OpenCV image format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Display the image in the window.
        cv2.imshow("Frame", cv_image)
        cv2.waitKey(1)  # Used to refresh the window so that the image display is updated

        # Images can also be saved or further processed
        # cv2.imwrite('frame.jpg', cv_image)

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()