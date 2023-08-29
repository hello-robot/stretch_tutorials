#!/usr/bin/env python3

import rclpy
import sys
import os
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class CaptureImage(Node):
    """
    A class that converts a subscribed ROS image to a OpenCV image and saves
    the captured image to a predefined directory.
    """
    def __init__(self):
        """
        A function that initializes a CvBridge class, subscriber, and save path.
        :param self: The self reference.
        """
        super().__init__('stretch_capture_image')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.save_path = '/home/hello-robot/ament_ws/src/stretch_tutorials/stored_data'
        self.br = CvBridge()

    def image_callback(self, msg, data):
        """
        A callback function that converts the ROS image to a CV2 image and stores the
        image.
        :param self: The self reference.
        :param msg: The ROS image message type.
        """
        
        try:
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().warn('CV Bridge error: {0}'.format(e))

        file_name = 'camera_image.jpeg'
        completeName = os.path.join(self.save_path, file_name)
        cv2.imwrite(completeName, image)
        rclpy.shutdown()
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    capture_image = CaptureImage()
    rclpy.spin(capture_image)

if __name__ == '__main__':
    main()
