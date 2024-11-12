from robot_controller_xbox import RobotController
from read_odem import OdomSubscriber

from rclpy.executors import MultiThreadedExecutor
import rclpy

# import atexit

def main(args=None):
    rclpy.init(args=args)
    # initialize the nodes
    robot_controller_node = RobotController()
    # robot_cam_node = ImageSubscriber()
    robot_odem_node = OdomSubscriber()

    # initialize an executor, to manage multiple nodes.
    executor = MultiThreadedExecutor()

    executor.add_node(robot_controller_node)
    # atexit.register(robot_controller_node.shutdown)

    # executor.add_node(robot_cam_node)
    executor.add_node(robot_odem_node)

    try:
        # rclpy.spin(robot_controller_node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        robot_controller_node.shutdown()
        executor.shutdown()


    robot_controller_node.destroy_node()
    # robot_cam_node.destroy_node()
    robot_odem_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()