# Common stuff
import os
import gc 
import cv2
import torch
import rclpy
import numpy as np

# Models stuff
from torch.cuda.amp import autocast
from ultralytics import YOLO, SAM 

# ROS Stuff
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from perception_interfaces.srv import Segmask


class SegMaskService(Node):
    def __init__(self):
        super().__init__('yolo_sam_service')
        # Create service
        self.service = self.create_service(Segmask, 'seg_mask', self.seg_mask_service)

        # Load the models
        self.yolo_model = YOLO('/workspaces/isaac_ros-dev/src/Perception/grounded_sam/seg_mask/checkpoints/yolo_best.pt')
        self.sam_model = SAM("sam2.1_t.pt")

        # Define CV bride
        self.bridge = CvBridge()

        # Mapping Dict
        # TODO - Fix this mapping
        self.mapping_dict = {'obj1': 0, 'obj2': 1, 'obj3': 2}

    def seg_mask_service(self, request, response):
        try:
            # Convert ROS image to OpenCV image
            color_image = self.bridge.imgmsg_to_cv2(request.color_image, "rgb8")
            color_image = np.array(color_image)
            color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
            print(np.shape(color_image), type(color_image))
            # Step 1: Get YOLO bounding boxes
        #     yolo_results = self.yolo_model.predict(color_image, conf=0.2)
        #     boxes = yolo_results[0].boxes.xyxy.cpu().numpy()
            # Get the object of interest
            object_of_interest = request.object_of_interest.data
            print(f"Object of Interests - {object_of_interest}")

            # color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
            
            # print(np.shape(color_image), type(color_image))
            # Step 1: Get YOLO bounding boxes
            yolo_results = self.yolo_model.predict(color_image)
            # boxes = yolo_results[0].boxes.xyxy.cpu().numpy()

            # Filter boxes that belong to the object of interest
            filtered_boxes = []
            for box in yolo_results[0].boxes:
                if box.cls == self.mapping_dict[object_of_interest]:
                    filtered_boxes.append(box.xyxy.cpu().numpy())
            boxes = np.array(filtered_boxes)
            print(f"Num Boxes - {len(boxes)}")

            # Step 2: Use SAMv2 to get segmentation masks
            sam_results = self.sam_model(color_image, bboxes=boxes)

            masks = sam_results[0].masks
            print(f"Num Masks - {len(masks)}")
            areas = []
            masks_list = []
            for mask in masks:
                mask_data = mask.data.cpu().numpy().astype(np.uint8)
                mask_area = np.sum(mask_data)
                areas.append(mask_area)
                masks_list.append(mask_data)
            
            try:
                avg_area = np.mean(areas)
            except Exception as e:
                print("Not Masks found")


            # Find the mask with the area closest to the average
            closest_mask = min(masks_list, key=lambda mask: abs(np.sum(mask) - avg_area))
            closest_mask = closest_mask[0]

            # Step 3: Draw the segmentation masks on the image
            # segmented_image = sam_results[0].masks[0].data.cpu().numpy()[0].astype(np.uint8)*255
            print("Closet_Mask Size ", np.shape(closest_mask))

            print(np.unique(closest_mask))    
            # Display the segmented image
            import copy
            overlay_image = copy.deepcopy(color_image)
            print("image shape, ", np.shape(overlay_image))
            overlay_image[closest_mask > 0] = [0, 255, 0]  # Overlay mask with green color
            cv2.addWeighted(overlay_image, 0.5, color_image, 0.5, 0, overlay_image)
            cv2.imshow("Segmented Image with Overlay", overlay_image)
            cv2.imshow("Segmented Image", copy.deepcopy(closest_mask)*255)
            cv2.waitKey(5000)
            cv2.destroyAllWindows()
            print("*"*40)
            print("MASK FOUND")
            print("*"*40)
            # Convert OpenCV image back to ROS Image
            # gray_segmented_image = cv2.cvtColor(closest_mask, cv2.COLOR_RGB2GRAY)
            response.segmask = self.bridge.cv2_to_imgmsg(closest_mask, "mono8")
            # print(np.unique(response.segmask))
            return response

        except Exception as e:
            self.get_logger().error(f"Failed to process the request: {e}")
            response.segmask = Image() #= self.bridge.cv2_to_imgmsg(cv2.cvtColor(np.zeros_like(color_image), cv2.COLOR_BGR2GRAY).astype(np.uint8), "mono8")
            return response


def main(args=None):
    rclpy.init(args=args)
    node = SegMaskService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
