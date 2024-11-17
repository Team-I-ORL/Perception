
import time
import rclpy
from rclpy.node import Node
import rclpy.serialization
from std_msgs.msg import Int8

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose

from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from sensor_msgs.msg import Image
# from rclpy.wait_for import wait_for_message
from std_msgs.msg import String
from ros2_aruco.aruco_node import ArucoNode
from ros2_aruco.wait_for_msg import wait_for_message

from perception_interfaces.srv import FindObjInFrame
import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from ros2_aruco_interfaces.srv import ArucoPose

class ArucoPoseService(Node):

    def __init__(self):
        super().__init__('find_aruco_service')
        # By default current pose is None and only updated once Aruco is updated
        self.current_ids = None
        self.current_pose = None
        self.frame_id = None

        # TF Listener
        # self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Define the service
        self.server_callback_group = ReentrantCallbackGroup()
        self.srv = self.create_service(FindObjInFrame, 'find_aruco_in_frame', self.get_aruco_pose, callback_group=self.server_callback_group)

                # Create a service client for the Aruco pose server
        # my_callback = MutuallyExclusiveCallbackGroup()
        self.client_callback_group = ReentrantCallbackGroup()
        self.aruco_pose_client = self.create_client(ArucoPose, 'get_aruco_pose', callback_group=self.client_callback_group)

        # Wait for the service to be available
        while not self.aruco_pose_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Aruco pose server not available, waiting...')


    # Service Callback
    def get_aruco_pose(self, request, response):

        image = request.image
        id = request.id

        # Create a request for the Aruco pose server
        aruco_pose_request = ArucoPose.Request()
        aruco_pose_request.image = image
        aruco_pose_request.id = id

        res = self.aruco_pose_client.call(aruco_pose_request)
        print(res)
        if list(res.pixel_coords.data)[0] != 0 and list(res.pixel_coords.data)[1] != 0:
            print("Got the pose")
            drop_pose = res.pose
            self.get_logger().info(f'Received drop pose: {drop_pose}')
            
            response.x = int(list(res.pixel_coords.data)[0])
            response.y = int(list(res.pixel_coords.data)[1])

        else:
            self.get_logger().error('Failed to get drop pose from Aruco pose server')
            response.x = int(-1)
            response.y = int(-1)

        print(response)
        return response

def main(args=None):
    rclpy.init()

    # aruco_pose_service = ArucoPoseService()
    # rclpy.spin(aruco_pose_service)

    # rclpy.shutdown()
    aruco_service = ArucoPoseService()
    executor = MultiThreadedExecutor()
    executor.add_node(aruco_service)

    try:
        aruco_service.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        aruco_service.get_logger().info('Keyboard interrupt, shutting down.\n')

    aruco_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()