#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class EdgeDetection(Node):
    """
    A class that converts a subscribed ROS image to a OpenCV image and saves
    the captured image to a predefined directory.
    """
    def __init__(self):
        """
        A function that initializes a CvBridge class, subscriber, and other
        parameter values.
        :param self: The self reference.
        """
        super().__init__('stretch_edge_detection')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/camera/color/image_raw', self.callback, 1)
        self.pub = self.create_publisher(Image, '/image_edge_detection', 1)
        self.save_path = '/home/hello-robot/catkin_ws/src/stretch_tutorials/stored_data'
        self.lower_thres = 100
        self.upper_thres = 200
        self.get_logger().info("Publishing the CV2 Image. Use RViz to visualize.")

    def callback(self, msg):
        """
        A callback function that converts the ROS image to a CV2 image and goes
        through the Canny Edge filter in OpenCV for edge detection. Then publishes
        that filtered image to be visualized in RViz.
        :param self: The self reference.
        :param msg: The ROS image message type.
        """
        try:
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().warn('CV Bridge error: {0}'.format(e))

        image = cv2.Canny(image, self.lower_thres, self.upper_thres)
        image_msg = self.bridge.cv2_to_imgmsg(image, 'passthrough')
        image_msg.header = msg.header
        self.pub.publish(image_msg)

def main(args=None):
    rclpy.init(args=args)
    edge_detection = EdgeDetection()
    rclpy.spin(edge_detection)

if __name__ == '__main__':
    main()
