from suctionnet_ros2.norm_inference import NormStdInferencer
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from perception_interfaces.srv import Sucpose
import numpy as np
from cv_bridge import CvBridge
import rclpy
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
import tf2_ros
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3

class SuctionNetNode(Node):
    def __init__(self):
        super().__init__("succionnet_node")
        self.norm_inferencer = NormStdInferencer()
        self.bridge = CvBridge()
        self.create_service(Sucpose, "sucpose_service", self.suctionnet_callback)
        # self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.pose_stamp_pub = self.create_publisher(PoseStamped, "suction_pose", 10)
        self.est_size_pub = self.create_publisher(Vector3, "estimated_size", 10)
        self.est_size_inflation = 0.05
        self.get_logger().info("SuctionNet Node has been started")
    
    def suctionnet_callback(self, request, response):
        rgb = request.color_image
        depth = request.depth_image
        seg_mask = request.segmask
        rgb_img = self.bridge.imgmsg_to_cv2(rgb, "rgb8")
        seg_mask = self.bridge.imgmsg_to_cv2(seg_mask, "mono8")
        depth_img = self.bridge.imgmsg_to_cv2(depth,"passthrough")
        depth_img = depth_img.astype(np.float32)
        depth_img /= 1000
        try:
            quat_values, trans_values, XYdiff = self.norm_inferencer.infer(rgb_img, depth_img, seg_mask)
        except Exception as e:
            self.get_logger().error("Error in inferencing: {}".format(e))
            print("Depth Image Shape: ", depth_img.shape, "RGB Image Shape: ", rgb_img.shape, "Seg Mask Shape: ", seg_mask.shape)
            print("Depth Image Avg: ", np.mean(depth_img))

            return response
        print("SuctionNet Getting from infer", [trans_values[0], trans_values[1], trans_values[2]])
        print("XYdiff: ", XYdiff)
        quat = Quaternion()
        quat.x = quat_values[0]
        quat.y = quat_values[1]
        quat.z = quat_values[2]
        quat.w = quat_values[3]

        trans = Point()
        trans.x = trans_values[0]
        trans.y = trans_values[1]
        trans.z = trans_values[2]

        res = Pose()
        res.orientation = quat
        res.position = trans
 
        response.pose = res

        # Publish the suction pose
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "head_camera_rgb_optical_frame"
        pose.pose = res
        self.pose_stamp_pub.publish(pose)
        self.est_size_pub.publish(Vector3(x=XYdiff[0] + self.est_size_inflation, y=XYdiff[1]+self.est_size_inflation, z=0.0))

        # Publish the transform
        # t = TransformStamped()
        # t.header.stamp = self.get_clock().now().to_msg()
        # t.header.frame_id = "head_camera_rgb_optical_frame"
        # t.child_frame_id = "suction_pose"
        # t.transform.translation.x = trans.x
        # t.transform.translation.y = trans.y
        # t.transform.translation.z = trans.z
        # t.transform.rotation = quat

        # self.tf_broadcaster.sendTransform(t)

        return response

def main():
    rclpy.init()
    node = SuctionNetNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
