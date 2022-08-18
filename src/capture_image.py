#!/usr/bin/env python3

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

class CaptureImage:
    """
    A class that converts a subscribed ROS image to a OpenCV image and saves
    the captured image to a predefined directory.
    """
    def __init__(self):
        """
        A function that initializes a CvBridge class, subscriber, and save path.
        :param self: The self reference.
        """
        # Initialize the CvBridge class
        self.bridge = CvBridge()

        # Initialize subscriber
        self.sub = rospy.Subscriber('/camera/color/image_raw_upright_view', Image, self.callback, queue_size=1)

        # Create path to save captured images to the stored data folder
        self.save_path = '/home/hello-robot/catkin_ws/src/stretch_tutorials/stored_data'

    def callback(self, msg):
        """
        A callback function that converts the ROS image to a CV2 image and stores the
        image.
        :param self: The self reference.
        :param msg: The ROS image message type.
        """
        # Try to convert the ROS image message to a cv2 Image
        try:
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            rospy.logwarn('CV Bridge error: {0}'.format(e))

        # Define the file name
        file_name = 'camera_image.jpeg'
        completeName = os.path.join(self.save_path, file_name)

        # Save image to previously defined directory
        cv2.imwrite(completeName, image)

        # Sends a signal to rospy to shutdown the ROS interfaces
        rospy.signal_shutdown("done")

        # Exit the Python interpreter
        sys.exit(0)

if __name__ == '__main__':
    # Initialize the node and name it capture_image
    rospy.init_node('capture_image', argv=sys.argv)

    #  Instantiate the CaptureImage class
    CaptureImage()

    # Give control to ROS.  This will allow the callback to be called whenever new
    # messages come in.  If we don't put this line in, then the node will not work,
    # and ROS will not process any messages
    rospy.spin()
