#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class DepthImageSubscriber(Node):
    def __init__(self, pixel_x, pixel_y):
        super().__init__('depth_image_subscriber')

        # Initialize the subscription to the depth image topic
        self.subscription = self.create_subscription(
            Image,
            # Change this to match your depth image topic
            '/head_camera/depth_registered/image',
            self.depth_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        self.bridge = CvBridge()

        # Set the pixel location
        self.pixel_x = pixel_x
        self.pixel_y = pixel_y
        self.get_logger().info(
            f"Subscribed to depth image topic. Checking depth at pixel ({self.pixel_x}, {self.pixel_y})")

    def depth_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            depth_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="passthrough")

            # Check if the pixel coordinates are valid
            if (0 <= self.pixel_x < depth_image.shape[1]) and (0 <= self.pixel_y < depth_image.shape[0]):
                depth_value = depth_image[self.pixel_y, self.pixel_x]

                # Convert depth from millimeters (common in 16-bit depth images) to meters
                if msg.encoding == "16UC1":  # 16-bit depth image in millimeters
                    depth_value = depth_value / 1000.0  # Convert to meters

                # Print the depth value at the specified pixel
                self.get_logger().info(
                    f"Depth at pixel ({self.pixel_x}, {self.pixel_y}): {depth_value} meters")
            else:
                self.get_logger().warn(
                    f"Pixel coordinates ({self.pixel_x}, {self.pixel_y}) are out of bounds.")
        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {e}")


def main(args=None):
    rclpy.init(args=args)

    # Set the pixel location where you want to check the depth
    pixel_x = 320  # Example x-coordinate
    pixel_y = 240  # Example y-coordinate

    # Create the node and spin
    depth_image_subscriber = DepthImageSubscriber(pixel_x, pixel_y)

    try:
        rclpy.spin(depth_image_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        depth_image_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
