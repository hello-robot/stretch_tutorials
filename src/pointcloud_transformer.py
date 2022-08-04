#!/usr/bin/env python

# Import modules
import rospy
import tf
import sensor_msgs.point_cloud2 as pc2

# Import message types and other python libraries
from sensor_msgs.msg import PointCloud2, PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Header


class PointCloudTransformer:
    """
    A class that takes in a PointCloud2 message and transfers it into a
    pointcloud message. Then that pointcloud is
    """
    def __init__(self):
        """
        Function that initializes the subsriber, publisher, and other variables.
        :param self: The self reference.
        """
        # Initialize Subscriber
        self.pointcloud2_sub = rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.callback_pcl2, queue_size=1)

        # Initialize PointCloud Publisher
        self.pointcloud_pub = rospy.Publisher("/camera_cloud", PointCloud, queue_size=1)

        # Initialize self.cloud for data storage in pointcloud_data callback function
        self.pcl2_cloud = None

        # Intialize tf.Transformlister
        self.listener = tf.TransformListener(True, rospy.Duration(10.0))

    def callback_pcl2(self,msg):
        """
        Callback function that stores the pointcloud2 message.
        :param self: The self reference.
        :param msg: The Pointcloud2 message type.
        """
        self.pcl2_cloud = msg


    def pcl_transformer(self):
        """
        A function that extracts the points from the stored PointCloud2 message
        and appends those points to a PointCloud message. Then transforms the
        PointCloud from its original header frame id, 'camera_color_optical_frame'
        to the 'base_link' frame.
        :param self: The self reference.
        """
        # Create a PointCloud for temporary use. Set the temporary PointCloud's
        # header to the stored PointCloud2 header
        temp_cloud = PointCloud()
        temp_cloud.header = self.pcl2_cloud.header


        # For loop to extract pointcloud2 data into a list of x,y,z, and
        # append those values to the PointCloud message, temp_cloud
        for data in pc2.read_points(self.pcl2_cloud, skip_nans=True):
            temp_cloud.points.append(Point32(data[0],data[1],data[2]))

        # Transform the points in the PointCloud message to reference the base_link
        transformed_cloud = self.transform_pointcloud(temp_cloud)

        # Publish transformed PointCloud
        self.pointcloud_pub.publish(transformed_cloud)

    def transform_pointcloud(self,msg):
        """
        Function that stores the pointcloud2 message.
        :param self: The self reference.
        :param msg: The PointCloud message.

        :returns new_cloud: PointCloud message.
        """
        while not rospy.is_shutdown():
            try:
                # run the transformPointCloud() function to change the referene frame
                # to the base_link
                new_cloud = self.listener.transformPointCloud("/base_link" ,msg)
                return new_cloud
                if new_cloud:
                    break
            except (tf.LookupException, tf.ConnectivityException,tf.ExtrapolationException):
                pass

if __name__=="__main__":
    # Initialize the node, and call it "pointcloud_transformer"
    rospy.init_node('pointcloud_transformer',anonymous=True)

    # Instantiate the PointCloudTransformer class
    obj = PointCloudTransformer()

    # We're going to publish information at 1 Hz
    rate = rospy.Rate(1)

    # Give the listener some time to accumulate transforms
    rospy.sleep(1)

    # Run while loop until the node is shutdown
    while not rospy.is_shutdown():

        # Run the pcl_transformer method
        obj.pcl_transformer()
        rate.sleep()
