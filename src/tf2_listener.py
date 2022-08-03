#!/usr/bin/env python

# Import modules
import rospy

# The TransformStamped message is imported to create a child frame
from geometry_msgs.msg import TransformStamped

# Import StaticTransformBroadcaster to publish static transforms
import tf2_ros


class FrameListener():
    """
    This Class prints the transformation between the fk_link_mast frame and the
    target frame, link_grasp_center.
    """
    def __init__(self):
        """
        A function that initializes the variables and looks up a tranformation
        between a target and source frame.
        :param self: The self reference.
        """
        # Start a Tf buffer that will store the tf information for a few seconds.
        # Then set up a tf listener, which will subscribe to all of the relevant
        # tf topics, and keep track of the information.
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        # Store frame names in variables that will be used to compute transformations
        from_frame_rel = 'link_grasp_center'
        to_frame_rel = 'fk_link_lift'

        # Give the listener some time to accumulate transforms
        rospy.sleep(1.0)

        # We're going to publish information at 1 Hz
        rate = rospy.Rate(1)

        # Run while loop until the node is shutdown
        while not rospy.is_shutdown():
            # Try to look up the transform we want. Use a try-except block, since
            # it may fail on any single call, due to internal timing issues in the
            # transform publishers
            try:
                # Look up transform between from_frame_rel and to_frame_rel frames
                trans = tf_buffer.lookup_transform(to_frame_rel,
                                                   from_frame_rel,
                                                   rospy.Time())
                # Print the transform
                rospy.loginfo('The pose of target frame %s with reference to %s is: \n %s', from_frame_rel, to_frame_rel, trans.transform)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn(' Could not transform %s from %s ', to_frame_rel, from_frame_rel)

            # Manage the rate the node prints out a message
            rate.sleep()


if __name__ == '__main__':
    # Initialize the node, and call it "tf2_liistener"
    rospy.init_node('tf2_listener')

    # Instantiate the FrameListener class
    FrameListener()

    # Give control to ROS.  This will allow the callback to be called whenever new
    # messages come in.  If we don't put this line in, then the node will not work,
    # and ROS will not process any messages
    rospy.spin()
