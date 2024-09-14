# #!/usr/bin/env python3

# import os
# import cv2
# import json
# import torch
# import numpy as np
# import matplotlib.pyplot as plt
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from PIL import Image as IMG
# from cv_bridge import CvBridge
# import message_filters
import grounding_dino.groundingdino.datasets.transforms as T
# from supervision import MaskAnnotator, Detections
# from pathlib import Path
# from torchvision.ops import box_convert
# from sam2.build_sam import build_sam2
# from sam2.sam2_image_predictor import SAM2ImagePredictor
# from sam2.automatic_mask_generator import SAM2AutomaticMaskGenerator
# from grounding_dino.groundingdino.util.inference import load_model, load_image, predict

# """
# Hyper parameters
# """
# TEXT_PROMPT = "box."
# SAM2_CHECKPOINT = "./checkpoints/sam2_hiera_large.pt"
# SAM2_MODEL_CONFIG = "sam2_hiera_l.yaml"
# GROUNDING_DINO_CONFIG = "grounding_dino/groundingdino/config/GroundingDINO_SwinT_OGC.py"
# GROUNDING_DINO_CHECKPOINT = "gdino_checkpoints/groundingdino_swint_ogc.pth"
# BOX_THRESHOLD = 0.35
# TEXT_THRESHOLD = 0.25
# DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
# OUTPUT_DIR = Path("outputs/grounded_sam2_local_demo")
# DUMP_JSON_RESULTS = True


# class ImageDepthSubscriber(Node):
#     def __init__(self):
#         super().__init__('image_depth_subscriber')

#         # Initialize subscribers for RGB and depth image topics
#         self.rgb_sub = message_filters.Subscriber(
#             self, Image, '/head_camera/rgb/image_raw')
#         self.depth_sub = message_filters.Subscriber(
#             self, Image, '/head_camera/depth_registered/image')

#         # Synchronize the RGB and depth images
#         self.ts = message_filters.ApproximateTimeSynchronizer(
#             [self.rgb_sub, self.depth_sub], 10, 0.1)
#         self.ts.registerCallback(self.callback)

#         self.bridge = CvBridge()

#         # Load models
#         self.sam2_model = build_sam2(
#             SAM2_MODEL_CONFIG, SAM2_CHECKPOINT, device=DEVICE)
#         self.sam2_predictor = SAM2AutomaticMaskGenerator(self.sam2_model)

#         self.grounding_model = load_model(
#             model_config_path=GROUNDING_DINO_CONFIG,
#             model_checkpoint_path=GROUNDING_DINO_CHECKPOINT,
#             device=DEVICE
#         )

#     def callback(self, rgb_msg, depth_msg):
#         try:
#             # Convert ROS Image messages to OpenCV format
#             rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
#             depth_image = self.bridge.imgmsg_to_cv2(
#                 depth_msg, desired_encoding="passthrough")

#             # Process the RGB image with Grounding DINO
#             image_source = IMG.fromarray(rgb_image).convert(
#                 "RGB")  # cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)

#             h, w, _ = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB).shape

#             # from torchvision import transforms as T

#             transform = T.Compose(
#                 [
#                     T.RandomResize([800], max_size=1333),
#                     T.ToTensor(),
#                     T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
#                 ]
#             )

#             # print("Working till here", transform(image_source, None))
#             image, _ = transform(image_source, None)
#             # print("Transform is the issue")

#             # Predict boxes using Grounding DINO
#             boxes, confidences, labels = predict(
#                 model=self.grounding_model,
#                 image=image,
#                 caption=TEXT_PROMPT,
#                 box_threshold=BOX_THRESHOLD,
#                 text_threshold=TEXT_THRESHOLD,
#             )

#             print("Here")

#             # Scale boxes to image dimensions
#             boxes = boxes * torch.Tensor([w, h, w, h])
#             input_boxes = box_convert(
#                 boxes=boxes, in_fmt="cxcywh", out_fmt="xyxy").numpy()

#             # Loop through each box, crop the image, and process it with SAM2
#             for i, box in enumerate(input_boxes):
#                 x_min, y_min, x_max, y_max = map(int, box)

#                 # Crop the region from the RGB image
#                 cropped_image = rgb_image[y_min:y_max, x_min:x_max]

#                 # Process the cropped image with SAM2 to generate masks
#                 # masks = self.sam2_predictor.generate(cropped_image)

#                 # # Display the cropped image and masks
#                 # plt.figure(figsize=(10, 10))
#                 # plt.imshow(cv2.cvtColor(cropped_image, cv2.COLOR_BGR2RGB))
#                 # self.show_anns(masks)
#                 # plt.axis('off')
#                 # plt.show()

#                 # Calculate the average depth for this bounding box
#                 cropped_depth = depth_image[y_min:y_max, x_min:x_max]
#                 valid_depth = cropped_depth[(
#                     cropped_depth > 0) & np.isfinite(cropped_depth)]
#                 if valid_depth.size > 0:
#                     avg_depth = np.mean(valid_depth) / \
#                         1000.0  # Convert mm to meters
#                     self.get_logger().info(
#                         f"Average depth in bounding box {i}: {avg_depth:.3f} meters")
#                 else:
#                     self.get_logger().warn(
#                         f"No valid depth values in bounding box {i}")

#         except Exception as e:
#             self.get_logger().error(f"Error processing images: {e}")

#     def show_anns(self, anns, borders=True):
#         if len(anns) == 0:
#             return
#         sorted_anns = sorted(anns, key=(lambda x: x['area']), reverse=True)
#         ax = plt.gca()
#         ax.set_autoscale_on(False)

#         img = np.ones((sorted_anns[0]['segmentation'].shape[0],
#                       sorted_anns[0]['segmentation'].shape[1], 4))
#         img[:, :, 3] = 0
#         for ann in sorted_anns:
#             m = ann['segmentation']
#             color_mask = np.concatenate([np.random.random(3), [0.5]])
#             img[m] = color_mask
#             if borders:
#                 contours, _ = cv2.findContours(
#                     m.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
#                 # Try to smooth contours
#                 contours = [cv2.approxPolyDP(
#                     contour, epsilon=0.01, closed=True) for contour in contours]
#                 cv2.drawContours(img, contours, -1,
#                                  (0, 0, 1, 0.4), thickness=1)

#         ax.imshow(img)


# def main(args=None):
#     rclpy.init(args=args)

#     # Create the ROS 2 node and spin it to subscribe to images
#     image_depth_subscriber = ImageDepthSubscriber()

#     try:
#         rclpy.spin(image_depth_subscriber)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         image_depth_subscriber.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3

import os
import cv2
import json
import torch
import numpy as np
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import message_filters
import tf2_ros
import tf_transformations
from PIL import Image as IMG
from grounding_dino.groundingdino.util.inference import load_model, predict
from supervision import MaskAnnotator, Detections
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor
from sam2.automatic_mask_generator import SAM2AutomaticMaskGenerator
from torchvision.ops import box_convert
from pathlib import Path
import tf2_geometry_msgs.tf2_geometry_msgs as tf2_geometry_msgs

"""
Hyper parameters
"""
TEXT_PROMPT = "box."
SAM2_CHECKPOINT = "./checkpoints/sam2_hiera_large.pt"
SAM2_MODEL_CONFIG = "sam2_hiera_l.yaml"
GROUNDING_DINO_CONFIG = "grounding_dino/groundingdino/config/GroundingDINO_SwinT_OGC.py"
GROUNDING_DINO_CHECKPOINT = "gdino_checkpoints/groundingdino_swint_ogc.pth"
BOX_THRESHOLD = 0.35
TEXT_THRESHOLD = 0.25
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
OUTPUT_DIR = Path("outputs/grounded_sam2_local_demo")
DUMP_JSON_RESULTS = True


class ImageDepthSubscriber(Node):
    def __init__(self):
        super().__init__('image_depth_subscriber')

        # Initialize subscribers for RGB and depth image topics
        self.rgb_sub = message_filters.Subscriber(
            self, Image, '/head_camera/rgb/image_raw')
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/head_camera/depth_registered/image')

        # Subscribe to camera info topics for both RGB and depth cameras
        self.rgb_camera_info_sub = self.create_subscription(
            CameraInfo, '/head_camera/rgb/camera_info', self.rgb_camera_info_callback, 10)
        self.depth_camera_info_sub = self.create_subscription(
            CameraInfo, '/head_camera/depth_registered/camera_info', self.depth_camera_info_callback, 10)

        # Synchronize the RGB and depth images
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], 10, 0.1)
        self.ts.registerCallback(self.callback)

        self.bridge = CvBridge()

        # Load models
        self.sam2_model = build_sam2(
            SAM2_MODEL_CONFIG, SAM2_CHECKPOINT, device=DEVICE)
        self.sam2_predictor = SAM2AutomaticMaskGenerator(self.sam2_model)

        self.grounding_model = load_model(
            model_config_path=GROUNDING_DINO_CONFIG,
            model_checkpoint_path=GROUNDING_DINO_CHECKPOINT,
            device=DEVICE
        )

        # Publisher for the 3D point (center of bounding box)
        self.point_pub = self.create_publisher(
            PointStamped, 'bbox_center_3d', 10)

        # Initialize TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Initialize variables for camera intrinsics
        self.rgb_camera_info = None
        self.depth_camera_info = None

    def rgb_camera_info_callback(self, msg):
        # Store the RGB camera info (intrinsic parameters)
        self.rgb_camera_info = msg
        self.get_logger().info("RGB Camera Info received")

    def depth_camera_info_callback(self, msg):
        # Store the Depth camera info (intrinsic parameters)
        self.depth_camera_info = msg
        self.get_logger().info("Depth Camera Info received")

    def callback(self, rgb_msg, depth_msg):
        if self.rgb_camera_info is None or self.depth_camera_info is None:
            self.get_logger().warn("Waiting for camera info...")
            return

        try:
            # Convert ROS Image messages to OpenCV format
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding="passthrough")

            # Convert RGB image to PIL format
            image_source = IMG.fromarray(rgb_image).convert("RGB")
            h, w = rgb_image.shape[:2]

            transform_pt = T.Compose(
                [
                    T.RandomResize([800], max_size=1333),
                    T.ToTensor(),
                    T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
                ]
            )

            # Apply transformations (assuming you already have them defined in the transform)
            image, _ = transform_pt(image_source, None)

            # Predict boxes using Grounding DINO
            boxes, confidences, labels = predict(
                model=self.grounding_model,
                image=image,
                caption=TEXT_PROMPT,
                box_threshold=BOX_THRESHOLD,
                text_threshold=TEXT_THRESHOLD,
            )

            # Scale boxes to image dimensions
            boxes = boxes * torch.Tensor([w, h, w, h])
            input_boxes = box_convert(
                boxes=boxes, in_fmt="cxcywh", out_fmt="xyxy").numpy()

            # Loop through each box, crop the image, and process it with SAM2
            for i, box in enumerate(input_boxes):
                x_min, y_min, x_max, y_max = map(int, box)

                # Get the center of the bounding box
                center_x = int((x_min + x_max) / 2)
                center_y = int((y_min + y_max) / 2)

                # Get the depth at the center of the bounding box
                depth = depth_image[center_y, center_x] / \
                    1000.0  # Convert mm to meters
                if depth == 0 or np.isnan(depth):
                    self.get_logger().warn(
                        f"Invalid depth value at the center of bounding box {i}")
                    continue

                # Extract the intrinsic parameters from CameraInfo
                fx = self.rgb_camera_info.k[0]  # Focal length x
                fy = self.rgb_camera_info.k[4]  # Focal length y
                cx = self.rgb_camera_info.k[2]  # Principal point x
                cy = self.rgb_camera_info.k[5]  # Principal point y

                # Compute 3D coordinates (x, y, z) in the camera frame using camera intrinsics
                point_x = (center_x - cx) * depth / fx
                point_y = (center_y - cy) * depth / fy
                point_z = depth

                # Create a PointStamped message to publish the point
                point_msg = PointStamped()
                point_msg.header.frame_id = 'head_camera'  # Camera frame
                point_msg.header.stamp = self.get_clock().now().to_msg()

                point_msg.point.x = point_x
                point_msg.point.y = point_y
                point_msg.point.z = point_z

                # Transform the point from the camera frame to the 'base_link' frame
                try:
                    transform = self.tf_buffer.lookup_transform(
                        'base_link', 'head_camera_depth_optical_frame', rclpy.time.Time())
                    point_in_base = tf2_geometry_msgs.do_transform_point(
                        point_msg, transform)

                    # Publish the 3D point
                    self.point_pub.publish(point_in_base)

                    self.get_logger().info(f"Published 3D center of bounding box {i} in base_link frame: "
                                           f"x={point_in_base.point.x}, y={point_in_base.point.y}, z={point_in_base.point.z}")

                except Exception as e:
                    self.get_logger().error(f"Error transforming point: {e}")

        except Exception as e:
            self.get_logger().error(f"Error processing images: {e}")


def main(args=None):
    rclpy.init(args=args)

    # Create the ROS 2 node and spin it to subscribe to images
    image_depth_subscriber = ImageDepthSubscriber()

    try:
        rclpy.spin(image_depth_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        image_depth_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
