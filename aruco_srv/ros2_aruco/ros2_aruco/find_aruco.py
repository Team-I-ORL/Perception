
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
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
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
        self.srv = self.create_service(FindObjInFrame, 'find_aruco_in_frame', self.get_aruco_pose)

                # Create a service client for the Aruco pose server
        my_callback = MutuallyExclusiveCallbackGroup()
        self.aruco_pose_client = self.create_client(ArucoPose, 'get_aruco_pose', callback_group=my_callback)

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
    #     image = request.image
    #     id = request.id

    #     start = time.time()
            
    #     try:
    #         # _, self.current_pose = self.ar_node.image_callback(image)
    #         import cv2.aruco as aruco

    #         # Convert ROS Image message to OpenCV image
    #         def ros_image_to_cv2(image_msg):
    #             bridge = CvBridge()
    #             return bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

    #         cv_image = ros_image_to_cv2(image)

    #         # Convert the image to grayscale
    #         gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    #         # Define the aruco dictionary and parameters
    #         aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
    #         parameters = aruco.DetectorParameters_create()

    #         # Detect the markers in the image
    #         corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    #         if ids is not None:
    #             self.current_pose.marker_ids = ids.flatten().tolist()
    #             self.current_pose.poses = []

    #             for corner in corners:
    #                 # Calculate the center of the marker
    #                 center_x = int((corner[0][0][0] + corner[0][2][0]) / 2)
    #                 center_y = int((corner[0][0][1] + corner[0][2][1]) / 2)

    #                 # Create a Pose object for the marker
    #                 pose = Pose()
    #                 pose.position.x = center_x
    #                 pose.position.y = center_y
    #                 pose.position.z = 0.0  # Assuming the marker is on a flat surface

    #                 self.current_pose.poses.append(pose)

    #         for i in range(len(self.current_pose.marker_ids)):
    #             if self.current_pose.marker_ids[i] == id:
    #                 marker_pose = self.current_pose.poses[i]
    #                 self.get_logger().info(f"ID: {id} found in the image")
    #                 break


    #     except Exception as e:
    #         self.get_logger().error(f"Error: {e}")
    #         # if (time.time()- start) > 5.0:
    #         self.get_logger().warn("Aruco not found or ID not present in the image. Returning None...")

    #         fail_pose = Pose()
    #         fail_pose.position.x = 0.0
    #         fail_pose.position.y = 0.0
    #         fail_pose.position.z = 0.0

    #         response.pose = fail_pose
    #         self.current_pose = None
    #         return response
        
    #     self.get_logger().info("Aruco Found")

    #     # Loop over the pose array and get the pose of ID needed
    #     # Check if aruco is detect and the required ID is present or not
    #     # print("maserker iddddddddddd ", self.current_pose.marker_ids)
    #     if self.current_pose is not None and id in self.current_pose.marker_ids:
    #         # Get the index of the required ID
    #         id_idx = self.current_pose.marker_ids.index(id)
    #         req_pose = self.current_pose.poses[id_idx]

    #         # Transform the pose to the desired frame
    #         while not self.tf_buffer.can_transform('base_link', self.current_pose.header.frame_id, rclpy.time.Time()):
    #             self.get_logger().info(f'Waiting for transform from {self.current_pose.header.frame_id} to base_link...')
    #             rclpy.spin_once(self, timeout_sec=0.1)
    #         transform = self.tf_buffer.lookup_transform('base_link', self.current_pose.header.frame_id, rclpy.time.Time())
    #         transformed_pose = do_transform_pose(req_pose, transform)

    #         # If the offset is needed based on ID
    #         if id == 1:
    #             transformed_pose.position.y += 0.05
    #             transformed_pose.position.x -= 0.05

    #         # if id == 2:
    #         #     transformed_pose.position.y -= 0.00

    #         if id == 3:
    #             transformed_pose.position.x -= 0.02
    #             transformed_pose.position.y -= 0.05

    #         transformed_pose.position.x -= 0.28
    #         transformed_pose.position.z -= 0.15

    #         response.pose = transformed_pose
    #         self.current_pose = None
    #         return response
        
    #     else:
    #         if (time.time()- start) > 5.0:
    #             self.get_logger().warn("Aruco not found or ID not present in the image. Returning None...")

    #             fail_pose = Pose()
    #             fail_pose.position.x = 0.0
    #             fail_pose.position.y = 0.0
    #             fail_pose.position.z = 0.0

    #             response.pose = fail_pose
    #             self.current_pose = None
    #             return response
                    
    #         self.current_pose = None
    #         # continue
    #             # response.pose = Pose()
    #             # self.current_pose = None
    #             # return response

    # # Subscriber Callback
    # def pose_callback(self, msg):
    #     self.headers = msg.header.frame_id
    #     self.current_ids = msg.marker_ids
    #     self.current_pose = msg.poses
    #     # self.get_logger().info(f'Current Pose: {self.current_pose}')

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