import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# import rpyc for sending through RPC
from rpyc import Service
from rpyc.utils.server import ThreadedServer
from cv_bridge import CvBridge
import cv2


class ImageSubscriber(Node, Service):

    def __init__(self):
        Node.__init__(self, 'image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/front_stereo_camera/left_rgb/image_raw',
            self.image_callback,
            10
        )
        # msg buffer initialization
        self.raw_image = None

        # image data conversion
        self.bridge = CvBridge()  # For converting ROS image messages to Open CV format
        
        self.get_logger().info('Image Subscriber Node has been started.')

    def image_callback(self, msg):
        self.raw_image = msg
    
    def exposed_read_image(self):
        # let server get this msg
        return self.raw_image


    def exposed_cv_image(self):
        # Convert ROS image messages to OpenCV image format
        cv_image = self.bridge.imgmsg_to_cv2(self.raw_image, desired_encoding='bgr8')
        # resized_image = cv2.resize(cv_image, (300, 300))

        # 将OpenCV图像编码为JPEG格式的字节流
        _, buffer = cv2.imencode('.jpg', cv_image)
        return buffer.tobytes()
        """
        # Display the image in the window.
        cv2.imshow("Frame", cv_image)
        cv2.waitKey(1)  # Used to refresh the window so that the image display is updated

        # Images can also be saved or further processed
        # cv2.imwrite('frame.jpg', cv_image)
        """


def main(args=None):
    rclpy.init(args=args)
    cam_node = ImageSubscriber()

    # create a threading for ROS node
    import threading
    ros_thread = threading.Thread(target=rclpy.spin, args=(cam_node,), daemon=True)
    ros_thread.start()

    # start the rpyc server
    server = ThreadedServer(cam_node, port=18860, hostname='0.0.0.0')
    print("RPyC Server is listening on port 18860...")
    try:
        server.start()
    except KeyboardInterrupt:
        pass
    finally:
        cam_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()