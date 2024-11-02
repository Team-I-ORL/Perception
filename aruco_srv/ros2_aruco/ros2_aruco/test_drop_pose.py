import cv2
import numpy as np
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from perception_interfaces.srv import Droppose
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor


def main(args=None):
    rclpy.init(args=args)

    node = Node('test_drop_pose_client')
    my_callback = MutuallyExclusiveCallbackGroup()
    client = node.create_client(Droppose, 'droppose_service', callback_group=my_callback)

    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting again...')

    request = Droppose.Request()

    def image_callback(msg):
        request.color_image = msg

    subscription = node.create_subscription(
        Image,
        '/head_camera/rgb/image_raw',
        image_callback,
        10
    )

    while request.color_image is None:
        rclpy.spin_once(node)

    # Set the ID for testing
    request.aruco_id = 1

    future = client.call_async(request)

    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(f'Received drop pose: {future.result().drop_pose}')
    else:
        node.get_logger().error('Failed to call service')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()