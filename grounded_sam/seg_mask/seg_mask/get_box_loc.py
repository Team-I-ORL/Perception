# Importing Libraries
# import os 
# import cv2
# import torch
# import numpy as np
# import supervision as sv 
# import pycocotools.mask as mask_util
# from torchvision.ops import box_convert
import torch
from PIL import Image
from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection

# Ensuring the paths are fine 
# import sys
# sys.path.append('/home/siddharth/fall_ws/src/Perception/grounded_sam/seg_mask/seg_mask/')
# # sys.path.append('/home/siddharth/fall_ws/src/Perception/grounded_sam/seg_mask/seg_mask/grounding_dino/')
# import grounding_dino.groundingdino.datasets.transforms as T
# from grounding_dino.groundingdino.util.inference import load_model, load_image, predict

# ROS2 stuff 
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from perception_interfaces.srv import FindObjInFrame, FindX

class FindBoxService(Node):
    
    def __init__(self):
        super().__init__('find_box_service')
        self.service = self.create_service(FindObjInFrame, 'find_box_in_frame', self.find_box_service)

        self.model_id = "IDEA-Research/grounding-dino-tiny"
        self.device = "cuda" if torch.cuda.is_available() else "cpu"

        self.processor = AutoProcessor.from_pretrained(self.model_id)
        self.model = AutoModelForZeroShotObjectDetection.from_pretrained(self.model_id).to(self.device)

        self.prompt = "cardboard box."
        self.BOX_THRESHOLD = 0.5
        self.TEXT_THRESHOLD = 0.5

        self.bridge = CvBridge()    

    def find_box_service(self, request, response):

        # Default response
        response.x = -1
        response.y = -1

        # Convert ROS image to OpenCV image
        try:
            self.image = self.bridge.imgmsg_to_cv2(request.image, "rgb8")
        except Exception as e:
            self.get_logger().info(f"Error bridging the image - {e}")
        # Ensure image is not None
        if self.image is None:
            return response
        
        self.model_inputs = self.processor(images = self.image, text = self.prompt, return_tensors="pt").to(self.device)

        with torch.no_grad():
            self.model_outputs = self.model(**self.model_inputs)

        results = self.processor.post_process_grounded_object_detection(
            self.model_outputs,
            self.model_inputs.input_ids,
            box_threshold = self.BOX_THRESHOLD,
            text_threshold = self.TEXT_THRESHOLD,
            target_sizes=[self.image.shape[:-1]]
        )
        
        print(len(results[0]['scores']))
        print(results)
        # If no boxes are found, return
        if len(results[0]['scores']) == 0:
            return response
        
        # Get the box with the highest confidence
        boxes = results[0]['boxes']
        scores = results[0]['scores']
        max_score_index = scores.argmax()
        box = boxes[max_score_index]

        # Return the bounding box
        # Calculate the center of the bounding box
        response.x = int((box[0] + box[2]) / 2)
        response.y = int((box[1] + box[3]) / 2)

        return response

def main(args=None):

    rclpy.init(args=args)
    node = FindBoxService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
