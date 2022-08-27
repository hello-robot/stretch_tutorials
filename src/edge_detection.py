#!/usr/bin/env python

# Import Modules
import rospy
import sys
import os
import cv2

# We're going to subscribe to an uncompressed image, so we need to import the
# definition for them
from sensor_msgs.msg import Image

# Import CvBridge function to convert between ROS image messages and OpenCV images.
from cv_bridge import CvBridge, CvBridgeError


class EdgeDetection:
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
        # Instantiate a CvBridge() object
        self.bridge = CvBridge()

        # Initialize subscriber
        self.sub = rospy.Subscriber('/camera/color/image_raw_upright_view', Image, self.callback, queue_size=1)

        # Initialize publisher
        self.pub = rospy.Publisher('/image_edge_detection', Image, queue_size=1)

        # Create path to save effort and position values
        self.save_path = '/home/hello-robot/catkin_ws/src/stretch_tutorials/stored_data'

        # Setting edge detection parameters
        self.lower_thres = 100
        self.upper_thres = 200

        # Create log message
        rospy.loginfo("Publishing the CV2 Image. Use RViz to visualize.")

    def callback(self, msg):
        """
        A callback function that converts the ROS image to a CV2 image and goes
        through the Canny Edge filter in OpenCV for edge detection. Then publishes
        that filtered image to be visualized in RViz.
        :param self: The self reference.
        :param msg: The ROS image message type.
        """
        # Try to convert the ROS image message to a CV2 Image
        try:
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError, e:
            rospy.logwarn('CV Bridge error: {0}'.format(e))

        # Apply the Canny Edge filter function to the transformed CV2 Image
        image = cv2.Canny(image, self.lower_thres, self.upper_thres)

        # Convert CV2 Image to a ROS image
        image_msg = self.bridge.cv2_to_imgmsg(image, 'passthrough')
        image_msg.header = msg.header

        # Publish CV2 image
        self.pub.publish(image_msg)

if __name__ == '__main__':
    # Initialize the node and name it edge detection
    rospy.init_node('edge_detection', argv=sys.argv)

    # Instantiate the EdgeDetection Class
    EdgeDetection()

    # Give control to ROS.  This will allow the callback to be called whenever new
    # messages come in.  If we don't put this line in, then the node will not work,
    # and ROS will not process any messages
    rospy.spin()
