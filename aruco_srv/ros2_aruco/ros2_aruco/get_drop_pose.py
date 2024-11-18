import time
import rclpy
from rclpy.node import Node
import rclpy.serialization
from std_msgs.msg import Int8
from ros2_aruco_interfaces.srv import ArucoPose
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose

from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from sensor_msgs.msg import Image
# from rclpy.wait_for import wait_for_message
from std_msgs.msg import String
from ros2_aruco.aruco_node import ArucoNode
from ros2_aruco.wait_for_msg import wait_for_message

from perception_interfaces.srv import Droppose
import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class DropPoseService(Node):

    def __init__(self):
        super().__init__('get_drop_pose_service')
        # By default current pose is None and only updated once Aruco is updated
        self.current_ids = None
        self.current_pose = None
        self.frame_id = None

        # Define the service
        self.srv = self.create_service(Droppose, 'droppose_service', self.get_drop_pose)

        # Create a service client for the Aruco pose server
        my_callback = MutuallyExclusiveCallbackGroup()
        self.aruco_pose_client = self.create_client(ArucoPose, 'get_aruco_pose', callback_group=my_callback)

        # Wait for the service to be available
        while not self.aruco_pose_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Aruco pose server not available, waiting...')

    # Service Callback
    def get_drop_pose(self, request, response):

        image = request.color_image
        id = request.aruco_id

        # Create a request for the Aruco pose server
        aruco_pose_request = ArucoPose.Request()
        aruco_pose_request.image = image
        aruco_pose_request.id = id

        res = self.aruco_pose_client.call(aruco_pose_request)

        if res is not None:
            print("Got the pose")
            drop_pose = res.pose
            self.get_logger().info(f'Received drop pose: {drop_pose}')
            
            #if drop_pose.position.x >= 1.15:
            #    drop_pose.position.x = 0.95
            #elif drop_pose.position.x < 1.15:
            drop_pose.position.x -= 0.2
            # drop_pose.position.x -= 0.0
            

            if id == 7:
                drop_pose.position.z += 0.30
            else:
                drop_pose.position.z += 0.40
            drop_pose.orientation.x = 0.0
            drop_pose.orientation.y = 0.0
            drop_pose.orientation.z = 0.0
            drop_pose.orientation.w = 1.0 
            response.pose = drop_pose

        else:
            self.get_logger().error('Failed to get drop pose from Aruco pose server')

        return response

       
def main(args=None):
    rclpy.init()
    
    drop_pose_service = DropPoseService()
    executor = MultiThreadedExecutor()
    executor.add_node(drop_pose_service)

    try:
        drop_pose_service.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        drop_pose_service.get_logger().info('Keyboard interrupt, shutting down.\n')

    drop_pose_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
