import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import TransformStamped, PoseStamped
import tf2_ros
import tf2_geometry_msgs
# Assume the service type is suc_pose_interface.srv.SucPose
from perception_interfaces.srv import Sucpose
import time

class SucPoseClient(Node):
    def __init__(self):
        super().__init__('suc_pose_client')
        self.cli = self.create_client(Sucpose, '/sucpose_service')
        self.img_sub = self.create_subscription(Image, '/head_camera/rgb/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/head_camera/depth/image_rect_raw', self.depth_callback, 10)

        self.pose_pub = self.create_publisher(PoseStamped, 'transformed_pose', 10)

        self.br = tf2_ros.TransformBroadcaster(self)

        self.image = None
        self.depth = None

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.req = Sucpose.Request()
        self.bridge = CvBridge()

        self.sent = False

    def image_callback(self, msg):
        # self.get_logger().info('Getting rgb')
        self.image = msg

    def depth_callback(self, msg):
        # self.get_logger().info('Getting depth')
        self.depth = msg
        # depth_np = self.bridge.imgmsg_to_cv2(msg,"passthrough")
        # depth_np = depth_np.astype(np.float32)
        # depth_np /= 1000
        # self.depth = self.bridge.cv2_to_imgmsg(depth_np,encoding="32FC1")


    def send_request(self, fromLocal=False):
        if fromLocal:
            self.send_local_request()
        else:
            self.send_ros_request()

    def send_ros_request(self):
        while self.image is None or self.depth is None:
            self.get_logger().info('Waiting for image, depth...')
            rclpy.spin_once(self, timeout_sec=10.0)
        
        seg_dir = "/home/siddharth/Downloads/frame0000_seg2.png"
        segmask_cv = cv2.imread(seg_dir, cv2.IMREAD_GRAYSCALE).astype(bool)
        segmask_cv = segmask_cv.astype(np.uint8) * 255
        segmask_msg = self.bridge.cv2_to_imgmsg(segmask_cv, encoding="mono8")

        self.req.color_image = self.image
        self.req.depth_image = self.depth
        self.req.segmask = segmask_msg
        camera_info = CameraInfo()
        self.req.camera_info = camera_info

        self.future = self.cli.call_async(self.req)

        self.future.add_done_callback(self.callback)

    def callback(self, future):
        try:
            pose = future.result().pose
            self.get_logger().info(f'Pose: {pose}')
            # Create a TransformStamped message for camera to base transformation
            tf_buffer = tf2_ros.Buffer()
            tf_listener = tf2_ros.TransformListener(tf_buffer, self)   
            transformed_pose = None
            try:
                if tf_buffer.can_transform('base_link', 'head_camera_rgb_optical_frame', rclpy.time.Time(), rclpy.duration.Duration(seconds=10.0)):
                    transform = tf_buffer.lookup_transform('base_link', 'head_camera_rgb_optical_frame', rclpy.time.Time(),timeout=rclpy.duration.Duration(seconds=10.0))
                    transformed_pose = tf2_geometry_msgs.do_transform_pose(pose, transform)
                    self.get_logger().info(f'Transformed Pose in Base Frame: {transformed_pose}')
                else:
                    self.get_logger().error('Transform not available between base_link and head_camera_rgb_optical_frame.')
                    return
                self.get_logger().info(f'Transformed Pose in Base Frame: {transformed_pose}')
                pose_stamped = PoseStamped()
                pose_stamped.header.stamp = self.get_clock().now().to_msg()
                pose_stamped.header.frame_id = 'base_link'
                pose_stamped.pose = transformed_pose
                self.pose_pub.publish(pose_stamped)
                
                # Broadcast the transform from base_link to the pose
                transform_stamped = TransformStamped()
                transform_stamped.header.stamp = self.get_clock().now().to_msg()
                transform_stamped.header.frame_id = 'base_link'
                transform_stamped.child_frame_id = 'pose_frame'
                transform_stamped.transform.translation.x = transformed_pose.position.x
                transform_stamped.transform.translation.y = transformed_pose.position.y
                transform_stamped.transform.translation.z = transformed_pose.position.z
                transform_stamped.transform.rotation = transformed_pose.orientation

                self.br.sendTransform(transform_stamped)

            except tf2_ros.LookupException as e:
                self.get_logger().error(f'Transform lookup failed: {e}')
            except tf2_ros.ExtrapolationException as e:
                self.get_logger().error(f'Transform extrapolation failed: {e}')
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


    def send_local_request(self):
        # Load images from local directory
        depth_dir = "/home/jinkai/Downloads/Test_Images/Depth/image_50.png"
        rgb_dir = "/home/jinkai/Downloads/Test_Images/RGB/image_50.png"
        seg_dir = "/home/jinkai/Downloads/Test_Images/seg_mask/image_50.png"

        # Read images using OpenCV
        color_image_cv = cv2.imread(rgb_dir, cv2.IMREAD_COLOR)
        depth_image_cv = cv2.imread(depth_dir, cv2.IMREAD_GRAYSCALE).astype(np.float32) / 256 * 1000.0
        segmask_cv = cv2.imread(seg_dir, cv2.IMREAD_GRAYSCALE).astype(bool)
        segmask_cv = segmask_cv.astype(np.uint8) * 255

        # Convert BGR to RGB for color image
        color_image_cv = cv2.cvtColor(color_image_cv, cv2.COLOR_BGR2RGB)

        # Convert depth image to float32
        depth_image_cv = depth_image_cv.astype('float32')

        # Convert OpenCV images to ROS Image messages
        color_image_msg = self.bridge.cv2_to_imgmsg(color_image_cv, encoding="rgb8")
        depth_image_msg = self.bridge.cv2_to_imgmsg(depth_image_cv, encoding="32FC1")
        segmask_msg = self.bridge.cv2_to_imgmsg(segmask_cv, encoding="mono8")

        # Set the request fields
        self.req.color_image = color_image_msg
        self.req.depth_image = depth_image_msg
        self.req.segmask = segmask_msg

        # Prepare CameraInfo message
        camera_info = CameraInfo()
        self.req.camera_info = camera_info

        # Call the service
        self.future = self.cli.call_async(self.req)

        rclpy.spin_until_future_complete(self, self.future)
        if self.future.result() is not None:
            result = self.future.result().pose
            self.get_logger().info(f'Pose: {result}')
            self.process_pose(result)
            
        else:
            self.get_logger().error('Exception while calling service: %r' % self.future.exception())
    
    def process_pose(self, pose):
        # Create a TransformStamped message for camera to base transformation
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'base_frame'
        transform.child_frame_id = 'camera_frame'
        transform.transform.translation.x = 0.167
        transform.transform.translation.y = 0.053
        transform.transform.translation.z = 1.112
        transform.transform.rotation.x = 0.536
        transform.transform.rotation.y = -0.519
        transform.transform.rotation.z = 0.465
        transform.transform.rotation.w = -0.478

        # Transform the pose from camera frame to base frame
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer, self)
        transformed_pose = tf2_geometry_msgs.do_transform_pose(pose, transform)

        # Log the transformed pose
        self.get_logger().info(f'Transformed Pose in Base Frame: {transformed_pose}')

def main(args=None):
    rclpy.init(args=args)
    client = SucPoseClient()
    client.send_request()
    rclpy.spin(client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
