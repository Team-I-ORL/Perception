import cv2
import torch
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from torch.cuda.amp import autocast

# Import your YOLOv7 and SAMv2 utilities here
# Ensure you have your custom methods from your script imported as well
import sys
sys.path.append("/workspaces/isaac_ros-dev/src/Perception/grounded_sam/seg_mask/seg_mask/")
sys.path.append("/workspaces/isaac_ros-dev/src/Perception/grounded_sam/seg_mask/seg_mask/yolov7/")
from yolov7.models.experimental import attempt_load
from yolov7.utils.general import non_max_suppression, scale_coords
from yolov7.utils.datasets import letterbox

# Your SAMv2 imports
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor

# Service Import
# sys.path.append("/workspaces/isaac_ros-dev/install/perception_interfaces/include/perception_interfaces/")
# sys.path.append("/workspaces/isaac_ros-dev/src/Interfaces/")
# sys.path.append("/workspaces/isaac_ros-dev/install/perception_interfaces/include/perception_interfaces/perception_interfaces/")
from perception_interfaces.srv import Segmask

# print(type(Segmask))
import os

os.environ["TORCH_CUDNN_SDPA_ENABLED"] = "1"

# Device setup
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
bridge = CvBridge()

# Load YOLOv7 model
yolo_model = attempt_load('/workspaces/isaac_ros-dev/src/Perception/grounded_sam/seg_mask/checkpoints/yolo_best.pt', map_location=DEVICE)
yolo_model.eval()

# Load SAMv2 model
sam2_checkpoint = '/workspaces/isaac_ros-dev/src/Perception/grounded_sam/seg_mask/checkpoints/sam2_hiera_tiny.pt'
sam2_model_cfg = "sam2_hiera_t.yaml"
print("Building sam2")
sam2_model = build_sam2(sam2_model_cfg, sam2_checkpoint, apply_postprocessing=False, device=DEVICE)
print("Built samv2")
sam2_predictor = SAM2ImagePredictor(sam2_model)


class SegMaskService(Node):
    def __init__(self):
        super().__init__('yolo_sam_service')
        self.service = self.create_service(Segmask, 'seg_mask', self.seg_mask_service)

    def seg_mask_service(self, request, response):
        try:
            # Convert ROS image to OpenCV image
            color_image = bridge.imgmsg_to_cv2(request.color_image, "rgb8")

            # Step 1: Get YOLO bounding boxes
            bboxes = self.get_yolo_bboxes(color_image)

            # Step 2: Use SAMv2 to get segmentation masks
            masks = self.get_sam_masks(color_image, bboxes)

            # Step 3: Draw the segmentation masks on the image
            segmented_image = self.draw_masks(color_image, masks)

            print(np.unique(segmented_image))    
            # Display the segmented image
            import copy
            cv2.imshow("Segmented Image", copy.deepcopy(segmented_image)*255)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

            # Convert OpenCV image back to ROS Image
            gray_segmented_image = cv2.cvtColor(segmented_image, cv2.COLOR_RGB2GRAY)
            response.segmask = bridge.cv2_to_imgmsg(gray_segmented_image, "mono8")
            return response

        except Exception as e:
            self.get_logger().error(f"Failed to process the request: {e}")
            response.segmask = bridge.cv2_to_imgmsg(cv2.cvtColor(np.zeros_like(color_image), cv2.COLOR_BGR2GRAY), "mono8")
            return None

    def get_yolo_bboxes(self, img, img_size=640):
        # Prepare image for YOLOv7 model
        img_resized = letterbox(img, img_size)[0]
        img_resized = img_resized.transpose((2, 0, 1))  # HWC to CHW
        img_resized = np.ascontiguousarray(img_resized)

        # Convert to tensor and run YOLO inference
        img_tensor = torch.from_numpy(img_resized).to(DEVICE).float()
        img_tensor /= 255.0  # Normalize to [0,1]
        if img_tensor.ndimension() == 3:
            img_tensor = img_tensor.unsqueeze(0)

        # Get predictions
        with torch.no_grad():
            yolo_preds = yolo_model(img_tensor)[0]
        yolo_preds = non_max_suppression(yolo_preds, 0.25, 0.45, agnostic=False)  # Apply NMS

        # Extract bounding boxes and convert to original image scale
        bboxes = []
        for det in yolo_preds:
            if len(det):
                det[:, :4] = scale_coords(img_tensor.shape[2:], det[:, :4], img.shape).round()
                for *xyxy, conf, cls in reversed(det):
                    x1, y1, x2, y2 = map(int, xyxy)
                    bboxes.append((x1, y1, x2, y2))
        return bboxes

    def get_sam_masks(self, image, bboxes):
        masks = []
        sam2_predictor.set_image(image)
        with autocast():
            for bbox in bboxes:
                x1, y1, x2, y2 = bbox
                input_box = np.array([[x1, y1, x2, y2]])
                mask, _, _ = sam2_predictor.predict(point_coords=None,
                                                    point_labels=None,
                                                    box=input_box,
                                                    multimask_output=False)
                # masks.append(mask)
                if mask is not None and len(mask.shape) > 0:  # Ensure mask is not empty
                    masks.append(mask)

                else:
                    self.get_logger().warn(f"No mask generated for bbox: {bbox}")

        # Calculate the average area of the masks
        areas = [np.sum(mask) for mask in masks]
        avg_area = np.mean(areas)

        # Find the mask with the area closest to the average
        closest_mask = min(masks, key=lambda mask: abs(np.sum(mask) - avg_area))

        # Return only the closest mask
        masks = [closest_mask]
        return masks

    def draw_masks(self, image, masks):
        height, width = image.shape[:2]
        binary_mask = np.zeros_like(image)
        for mask in masks:
            if len(mask.shape) == 3 and mask.shape[0] == 1:
                mask = mask.squeeze(0)  # Remove extra dimension
            resized_mask = cv2.resize(mask, (width, height), interpolation=cv2.INTER_NEAREST)
            # color = np.random.random((1, 3)).tolist()[0]
            binary_mask[resized_mask > 0] = 1
        return binary_mask


def main(args=None):
    rclpy.init(args=args)
    node = SegMaskService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
