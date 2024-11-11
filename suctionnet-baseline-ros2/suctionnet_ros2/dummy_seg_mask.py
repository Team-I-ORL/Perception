import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import cv2
import numpy as np
from perception_interfaces.srv import Segmask

class DummySegment(Node):
    def __init__(self):
        super().__init__('dummy_segment')
        self.server = self.create_service(Segmask, '/dummy_segment_srv', self.segment_callback)
        self.bridge = CvBridge()
    
    def segment_callback(self, request, response):
        self._logger.info("Received request")
        seg_dir = "/home/jinkai/Downloads/Test_Images/seg_mask/image_50.png"
        segmask_cv = cv2.imread(seg_dir, cv2.IMREAD_GRAYSCALE).astype(bool)
        segmask_cv = segmask_cv.astype(np.uint8) * 255
        segmask_msg = self.bridge.cv2_to_imgmsg(segmask_cv, encoding="mono8")

        response.segmask = segmask_msg
        self._logger.info("Sent response")
        return response

def main(args=None):
    rclpy.init(args=args)
    dummy_segment = DummySegment()
    rclpy.spin(dummy_segment)
    rclpy.shutdown()