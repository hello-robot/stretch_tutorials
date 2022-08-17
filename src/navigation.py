#!/usr/bin/env python

# Import modules
import rospy
import actionlib
import sys

# We need the MoveBaseAction and MoveBaseGoal from the move_base_msgs package.
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# We're going to use the quaternion type here.
from geometry_msgs.msg import Quaternion

# tf includes a handy set of transformations to move between Euler angles and quaternions (and back).
from tf import transformations

class StretchNavigation:
    """
    A simple encapsulation of the navigation stack for a Stretch robot.
    """
    def __init__(self):
        """
        Create an instance of the simple navigation interface.
        :param self: The self reference.
        """
        # Make an action client, and wait for the server.
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        rospy.loginfo('{0}: Made contact with move_base server'.format(self.__class__.__name__))

        # Initialize the callback functions
        self.active_callback = None
        self.feedback_callback = None
        self.done_callback = None

        # Create a MoveBaseGoal message type, and fill in all of the relevant fields.
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time()

        # Set a position in the coordinate frame
        self.goal.target_pose.pose.position.x = 0.0
        self.goal.target_pose.pose.position.y = 0.0
        self.goal.target_pose.pose.position.z = 0.0

        # Set a orientation in the coordinate frame
        self.goal.target_pose.pose.orientation.x = 0.0
        self.goal.target_pose.pose.orientation.y = 0.0
        self.goal.target_pose.pose.orientation.z = 0.0
        self.goal.target_pose.pose.orientation.w = 1.0

    def get_quaternion(self,theta):
        """
        A function to build Quaternians from Euler angles. Since the Stretch only
        rotates around z, we can zero out the other angles.

        :param theta: The angle the robot makes with the x-axis.
        """
        return Quaternion(*transformations.quaternion_from_euler(0.0, 0.0, theta))

    def go_to(self, x, y, theta, wait=False):
        """
        Drive the robot to a particlar pose on the map. The Stretch only needs
        (x, y) coordinates and a heading.

        :param x: x coordinate in the map frame.
        :param y: y coordinate in the map frame.
        :param theta: heading (angle with the x-axis in the map frame)
        """
        rospy.loginfo('{0}: Heading for ({1}, {2}) at {3} radians'.format(self.__class__.__name__,
        x, y, theta))

        # Set the x and y positions
        self.goal.target_pose.pose.position.x = x
        self.goal.target_pose.pose.position.y = y

        # Set the orientation.  This is a quaternion, so we use the helper function.
        self.goal.target_pose.pose.orientation = self.get_quaternion(0.0)

        # Make the action call.  Include the callbacks.  Unless these have been set somewhere else, they are passed
        # as None, which means no callback.
        self.client.send_goal(self.goal,
                              active_cb=self.active_callback,
                              feedback_cb=self.feedback_callback,
                              done_cb=self.done_callback)

        self.client.wait_for_result()


if __name__ == '__main__':
    # Initialize the node, and call it "navigation"
    rospy.init_node('navigation', argv=sys.argv)

    # Declare a `StretchNavigation` object
    nav = StretchNavigation()

    # Send a nav goal to the `go_to()` method
    nav.go_to(0.25, 0.0, 0.0, wait=True)
