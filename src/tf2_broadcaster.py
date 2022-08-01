#!/usr/bin/env python

# Import modules
import rospy

# Because of transformations
import tf.transformations

# The TransformStamped message is imported to create a child frame
from geometry_msgs.msg import TransformStamped

# Import StaticTransformBroadcaster to publish static transforms
from tf2_ros import StaticTransformBroadcaster

class FixedFrameBroadcaster():
    """
    This node publishes three child static frames in reference to their parent frames as below:
    parent -> link_mast            child -> fk_link_mast
    parent -> link_lift            child -> fk_link_lift
    parent -> link_wrist_yaw       child -> fk_link_wrist_yaw
    """
    def __init__(self):
        """
        A function that creates a broadcast node and publishes three new transform
        frames.
        :param self: The self reference.
        """
        # Create a broadcast node
        self.br = StaticTransformBroadcaster()

        # Create a stamped transform to broadcast
        self.mast = TransformStamped()

        # Define parent and child frames
        self.mast.header.stamp = rospy.Time.now()
        self.mast.header.frame_id = 'link_mast'
        self.mast.child_frame_id = 'fk_link_mast'

        # Set pose values to transform
        self.mast.transform.translation.x = 0.0
        self.mast.transform.translation.y = 2.0
        self.mast.transform.translation.z = 0.0
        q = tf.transformations.quaternion_from_euler(1.5707, 0, -1.5707)
        self.mast.transform.rotation.x = q[0]
        self.mast.transform.rotation.y = q[1]
        self.mast.transform.rotation.z = q[2]
        self.mast.transform.rotation.w = q[3]

        # Repeat broadcast transform for fk_link_lift
        self.lift = TransformStamped()
        self.lift.header.stamp = rospy.Time.now()
        self.lift.header.frame_id = 'link_lift'
        self.lift.child_frame_id = 'fk_link_lift'
        self.lift.transform.translation.x = 0.0
        self.lift.transform.translation.y = 1.0
        self.lift.transform.translation.z = 0.0
        q = tf.transformations.quaternion_from_euler(1.5707, 0, -1.5707)
        self.lift.transform.rotation.x = q[0]
        self.lift.transform.rotation.y = q[1]
        self.lift.transform.rotation.z = q[2]
        self.lift.transform.rotation.w = q[3]

        # Repeat broadcast transform for fk_link_wrist_yaw
        self.wrist = TransformStamped()
        self.wrist.header.stamp = rospy.Time.now()
        self.wrist.header.frame_id = 'link_wrist_yaw'
        self.wrist.child_frame_id = 'fk_link_wrist_yaw'
        self.wrist.transform.translation.x = 0.0
        self.wrist.transform.translation.y = 1.0
        self.wrist.transform.translation.z = 0.0
        q = tf.transformations.quaternion_from_euler(1.5707, 0, -1.5707)
        self.wrist.transform.rotation.x = q[0]
        self.wrist.transform.rotation.y = q[1]
        self.wrist.transform.rotation.z = q[2]
        self.wrist.transform.rotation.w = q[3]

        # Publish transforms in a list
        self.br.sendTransform([self.mast, self.lift, self.wrist])

        # Create rospy log message
        rospy.loginfo('Publishing TF frames. Use RViz to visualize')

if __name__ == '__main__':
    # Initialize the node, and call it "tf2_broadcaster"
    rospy.init_node('tf2_broadcaster')

    # Instantiate the FixedFrameBroadcaster class
    FixedFrameBroadcaster()

    # Give control to ROS.  This will allow the callback to be called whenever new
	# messages come in.  If we don't put this line in, then the node will not work,
	# and ROS will not process any messages
    rospy.spin()
