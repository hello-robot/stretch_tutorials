#! /usr/bin/env python

# Import modules
import rospy
import time
import tf2_ros
import numpy as np
from math import pi

# Import hello_misc script for handling trajectory goals with an action client
import hello_helpers.hello_misc as hm

# We're going to subscribe to a JointState message type, so we need to import
# the definition for it
from sensor_msgs.msg import JointState

# Import the FollowJointTrajectoryGoal from the control_msgs.msg package to
# control the Stretch robot.
from control_msgs.msg import FollowJointTrajectoryGoal

# Import JointTrajectoryPoint from the trajectory_msgs package to define
# robot trajectories.
from trajectory_msgs.msg import JointTrajectoryPoint

# Import TransformStamped from the geometry_msgs package for the publisher
from geometry_msgs.msg import TransformStamped

class LocateDockingTag(hm.HelloNode):
    """
    A class that actuates the RealSense camera to find the docking station using
    an ArUco tag.
    """
    def __init__(self):
        """
        A function that initializes the subscriber and other needed variables.
        :param self: The self reference.
        """
        # Initialize the inhereted hm.Hellonode class
        hm.HelloNode.__init__(self)

        # Initialize Subscriber
        self.joint_states_sub = rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)

        # Initialize publisher
        self.transform_pub = rospy.Publisher('ArUco_transform', TransformStamped, queue_size=10)

        # Initialize the variable that will store the joint state positions
        self.joint_state = None

        # Provide the min and max joint positions for the head pan. These values
        # are needed for sweeping the head to search for the ArUco tag.
        self.min_pan_position = -4.10
        self.max_pan_position =  1.50

        # Define the number of steps for the sweep, then create the step size for
        # the head pan joint.
        self.pan_num_steps = 10
        self.pan_step_size = abs(self.min_pan_position - self.max_pan_position)/self.pan_num_steps

        # Define the min tilt position, number of steps, and step size.
        self.min_tilt_position = -0.75
        self.tilt_num_steps = 3
        self.tilt_step_size = pi/16


    def joint_states_callback(self, msg):
        """
        A callback function that stores Stretch's joint states.
        :param self: The self reference.
        :param msg: The JointState message type.
        """
        self.joint_state = msg


    def send_command(self, command):
        '''
        Handles single joint control commands by constructing a FollowJointTrajectoryGoal message and sending it to the trajectory_client
        created in hello_misc.
        :param self: The self reference.
        :param command: A dictionary message type.
        '''
        if (self.joint_state is not None) and (command is not None):

            # Extract the string value from the `joint` key
            joint_name = command['joint']

            # Set trajectory_goal as a FollowJointTrajectoryGoal and define
            # the joint name
            trajectory_goal = FollowJointTrajectoryGoal()
            trajectory_goal.trajectory.joint_names = [joint_name]

            # Set positions for the following 5 trajectory points
            point = JointTrajectoryPoint()

            # Check to see if `delta` is a key in the command dictionary
            if 'delta' in command:
                # Get the current position of the joint and add the delta as a
                # new position value
                joint_index = self.joint_state.name.index(joint_name)
                joint_value = self.joint_state.position[joint_index]
                delta = command['delta']
                new_value = joint_value + delta
                point.positions = [new_value]

            # Check to see if `position` is a key in the command dictionary
            elif 'position' in command:
                # extract the head position value from the `position` key
                point.positions = [command['position']]

            # Assign goal position with updated point variable
            trajectory_goal.trajectory.points = [point]

            # Specify the coordinate frame that we want (base_link) and set the time to be now.
            trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
            trajectory_goal.trajectory.header.frame_id = 'base_link'

            # Make the action call and send the goal. The last line of code waits
            # for the result
            self.trajectory_client.send_goal(trajectory_goal)
            self.trajectory_client.wait_for_result()


    def find_tag(self, tag_name='docking_station'):
        """
        A function that actuates the camera to search for a defined ArUco tag
        marker. Then the function returns the pose
        :param self: The self reference.
        :param tag_name: A string value of the ArUco marker name.

        :returns transform: A TransformStamped message type.
        """
        # Create a dictionaries to get the head in its initial position for its search
        pan_command = {'joint': 'joint_head_pan', 'position': self.min_pan_position}
        self.send_command(pan_command)
        tilt_command = {'joint': 'joint_head_tilt', 'position': self.min_tilt_position}
        self.send_command(tilt_command)

        # Nested for loop to sweep the pan and tilt in increments
        for i in range(self.tilt_num_steps):
            for j in range(self.pan_num_steps):
                # Update the joint_head_pan position by the pan_step_size
                pan_command = {'joint': 'joint_head_pan', 'delta': self.pan_step_size}
                self.send_command(pan_command)

                # Give time for system to do a Transform lookup before next step
                rospy.sleep(0.5)

                # Use a try-except block
                try:
                    # Look up transform between the base_link and located ArUco tag
                    transform = self.tf_buffer.lookup_transform('base_link',
                                                            tag_name,
                                                            rospy.Time())
                    rospy.loginfo("Found Requested Tag: \n%s", transform)

                    # Publish the transform
                    self.transform_pub.publish(transform)

                    # Return the transform
                    return transform
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    continue

            # Begin sweep with new tilt angle
            pan_command = {'joint': 'joint_head_pan', 'position': self.min_pan_position}
            self.send_command(pan_command)
            tilt_command = {'joint': 'joint_head_tilt', 'delta': self.tilt_step_size}
            self.send_command(tilt_command)
            rospy.sleep(.25)

        # Notify that the requested tag was not found 
        rospy.loginfo("The requested tag '%s' was not found", tag_name)

    def main(self):
        """
        Function that initiates the issue_command function.
        :param self: The self reference.
        """
        # The arguments of the main function of the hm.HelloNode class are the
        # node_name, node topic namespace, and boolean (default value is true)
        hm.HelloNode.main(self, 'docking_station_locator', 'docking_station_locator', wait_for_first_pointcloud=False)

        # Create a StaticTranformBoradcaster Node. Also, start a Tf buffer that
        # will store the tf information for a few seconds.Then set up a tf listener, which
        # will subscribe to all of the relevant tf topics, and keep track of the information.
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # Give the listener some time to accumulate transforms
        rospy.sleep(1.0)

        # Notify Stretch is searchring for the ArUco tag with a rospy loginfo fucntion
        rospy.loginfo('Searching for docking ArUco tag.')

        # Search for the ArUco marker for the docking station
        pose = self.find_tag("docking_station")


if __name__ == '__main__':
    # Use a try-except block
    try:
        # Instantiate the `LocateDockingTag()` object
        node = LocateDockingTag()
        # Run the `main()` method.
        node.main()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')
