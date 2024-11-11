
import time
import rclpy
from rclpy.node import Node
import rclpy.serialization
from std_msgs.msg import Int8, Float64MultiArray, Int16MultiArray
from ros2_aruco_interfaces.srv import ArucoPose
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from ros2_aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from sensor_msgs.msg import Image
# from rclpy.wait_for import wait_for_message
from std_msgs.msg import String
from ros2_aruco.aruco_node import ArucoNode
from ros2_aruco.wait_for_msg import wait_for_message


class ArucoPoseService(Node):

    def __init__(self):
        super().__init__('aruco_pose_service')
        # By default current pose is None and only updated once Aruco is updated
        self.current_ids = None
        self.current_pose = None
        self.frame_id = None

        # TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subcribe to aruco pose message and keep updating with new pose
        # self.subscription = self.create_subscription(ArucoMarkers, '/aruco_markers', self.pose_callback,10)
        
        # Define the service
        self.srv = self.create_service(ArucoPose, 'get_aruco_pose', self.get_aruco_pose)

        self.ar_node = ArucoNode()
        rclpy.spin_once(self.ar_node)

    # Service Callback
    def get_aruco_pose(self, request, response):
        id = request.id        
        self.get_logger().info(f'Incoming request\nID: {id}')
        start = time.time()
        msg = request.image
        # state = True
        while self.current_pose is None:
            state, msg = wait_for_message(Image, self, "/head_camera/rgb/image_raw")
            if msg is None or msg.data is None:
                self.get_logger().info("NoneType Image Received")
                continue 

            self.get_logger().info("Image Recieved\nFinding Aruco...")
            
            try:
                _, self.current_pose, self.pixel_pose = self.ar_node.image_callback(msg)
            except Exception as e:
                self.get_logger().error(f"Error: {e}")
                # if (time.time()- start) > 5.0:
                self.get_logger().warn("Aruco not found or ID not present in the image. Returning None...")

                fail_pose = Pose()
                fail_pose.position.x = 0.0
                fail_pose.position.y = 0.0
                fail_pose.position.z = 0.0

                response.pose = fail_pose
                pixel_pose = Int16MultiArray()
                pixel_pose.data = [0, 0]
                response.pixel_coords = pixel_pose
                self.current_pose = None
                print(f"Error is - {e}")
                return response
                continue
            

            # Loop over the pose array and get the pose of ID needed
            # Check if aruco is detect and the required ID is present or not
            # print("maserker iddddddddddd ", self.current_pose.marker_ids)
            if self.current_pose is not None and id in self.current_pose.marker_ids:
                self.get_logger().info("Aruco Found")
                # Get the index of the required ID
                id_idx = self.current_pose.marker_ids.index(id)
                req_pose = self.current_pose.poses[id_idx]

                # Transform the pose to the desired frame
                while not self.tf_buffer.can_transform('torso_lift_link', self.current_pose.header.frame_id, rclpy.time.Time()):
                    self.get_logger().info(f'Waiting for transform from {self.current_pose.header.frame_id} to torso_lift_link...')
                    rclpy.spin_once(self, timeout_sec=0.1)
                transform = self.tf_buffer.lookup_transform('torso_lift_link', self.current_pose.header.frame_id, rclpy.time.Time())
                transformed_pose = do_transform_pose(req_pose, transform)

                # If the offset is needed based on ID
                # if id == 1:
                #     transformed_pose.position.y += 0.05
                #     transformed_pose.position.x -= 0.05

                # # if id == 2:
                # #     transformed_pose.position.y -= 0.00

                # if id == 3:
                #     transformed_pose.position.x -= 0.02
                #     transformed_pose.position.y -= 0.05

                # transformed_pose.position.x -= 0.28
                # transformed_pose.position.z -= 0.15

                response.pose = transformed_pose
                pixel_pose = Int16MultiArray()
                pixel_pose.data = self.pixel_pose[id]
                response.pixel_coords = pixel_pose
                self.current_pose = None
                print("Returning")
                return response
            
            else:
                if (time.time()- start) > 5.0:
                    self.get_logger().warn("Aruco not found or ID not present in the image. Returning None...")

                    fail_pose = Pose()
                    fail_pose.position.x = 0.0
                    fail_pose.position.y = 0.0
                    fail_pose.position.z = 0.0

                    response.pose = fail_pose
                    self.current_pose = None
                    return response
                        
                self.current_pose = None
                continue
                # response.pose = Pose()
                # self.current_pose = None
                # return response

    # Subscriber Callback
    def pose_callback(self, msg):
        self.headers = msg.header.frame_id
        self.current_ids = msg.marker_ids
        self.current_pose = msg.poses
        # self.get_logger().info(f'Current Pose: {self.current_pose}')

def main(args=None):
    rclpy.init()

    aruco_pose_service = ArucoPoseService()
    rclpy.spin(aruco_pose_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()