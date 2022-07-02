#!/usr/bin/env python

# Every python controller needs these lines
import rospy
import time
import actionlib
import os
import csv

# Import the FollowJointTrajectoryGoal from the control_msgs.msg package to
# control the Stretch robot.
from control_msgs.msg import FollowJointTrajectoryGoal

# Import JointTrajectoryPoint from the trajectory_msgs package to define
# robot trajectories.
from trajectory_msgs.msg import JointTrajectoryPoint

# We're going to subscribe to 64-bit integers, so we need to import the definition
# for them.
from sensor_msgs.msg import JointState

# Import hello_misc script for handling trajecotry goals with an action client.
import hello_helpers.hello_misc as hm

# Import datetime for naming the exported data files
from datetime import datetime

class JointActuatorEffortSensor(hm.HelloNode):
    """
    A class that sends multiple joint trajectory goals to a single joint.
    """
    # Initialize the inhereted hm.Hellonode class.
    def __init__(self):
        """
        Function that initializes the subscriber,and other features.
        :param self: The self reference.
        """
        hm.HelloNode.__init__(self)

        # Set up a subscriber. We're going to subscribe to the topic "joint_states"
        self.sub = rospy.Subscriber('joint_states', JointState, self.callback)

        # Create a list of joints
        self.joints = ['joint_lift']

        # Create an empty list for later storage.
        self.joint_effort = []

        # Create path to save effort and position values
        self.save_path = '/home/hello-robot/catkin_ws/src/stretch_ros_tutorials/stored_data'

        # Create boolean data type for conditional statements if you want to export effort data.
        self.export_data = False


    def callback(self, msg):
        """
        Callback function to update and store JointState messages.
        :param self: The self reference.
        :param msg: The JointState message.
        """
        # Store the joint messages for later use
        self.joint_states = msg


    def issue_command(self):
        """
        Function that makes an action call and sends joint trajectory goals
        to a single joint
        :param self: The self reference.
        """

        # Set trajectory_goal as a FollowJointTrajectoryGoal and define
        # the joint name.
        trajectory_goal = FollowJointTrajectoryGoal()
        trajectory_goal.trajectory.joint_names = self.joints

        # Provide desired positions for joint name.
        point0 = JointTrajectoryPoint()
        point0.positions = [0.9]

        # Set a list to the trajectory_goal.trajectory.points
        trajectory_goal.trajectory.points = [point0]

        # Specify the coordinate frame that we want (base_link) and set the time to be now.
        trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
        trajectory_goal.trajectory.header.frame_id = 'base_link'

        # Make the action call and send the goal. Also define the feedback and
        # done callback function.
        self.trajectory_client.send_goal(trajectory_goal, feedback_cb=self.feedback_callback, done_cb=self.done_callback)
        rospy.loginfo('Sent stow goal = {0}'.format(trajectory_goal))
        self.trajectory_client.wait_for_result()


    def feedback_callback(self,feedback):
        """
        The feedback_callback function deals with the incoming feedback messages
        from the trajectory_client. Although, in this function, we do not use the
        feedback information.
        :param self: The self reference.
        :param feedback: FollowJointTrajectoryActilnFeedback message.
        """
        # Conditional statement for replacement of joint names if wrist_extension
        # is in the self.joint_names list.
        if 'wrist_extension' in self.joints:
            self.joints.remove('wrist_extension')
            self.joints.append('joint_arm_l0')

        # create an empty list of current effort values that will be appended
        # to the overall joint_effort list.
        current_effort = []

        # Use of forloop to parse the names of the requested joints list.
        # The index() function returns the index at the first occurrence of
        # the name of the requested joint in the self.joint_states.name list.
        for joint in self.joints:
            index = self.joint_states.name.index(joint)
            current_effort.append(self.joint_states.effort[index])

        # If the self.export_data boolean is false, then print the names and efforts
        # of the joints in the terminal.
        if not self.export_data:
            # Print the joint position values to the terminal
            print("name: " + str(self.joints))
            print("effort: " + str(current_effort))

        # Else, append the current effort to the joint_effort list.
        else:
            self.joint_effort.append(current_effort)


    def done_callback(self, status, result):
        """
        The done_callback function will be called when the joint action is complete.
        Within this function we export the data to a .txt file in  the /stored_data directory.
        :param self: The self reference.
        :param feedback: FollowJointTrajectoryActionFeedback message.
        """
        # Conditional statemets to notify whether the action succeeded or failed.
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo('Suceeded')
        else:
            rospy.loginfo('Failed')

        # Conditional statement for exporting data.
        if self.export_data:
            # File name is the date and time the action was complete
            file_name = datetime.now().strftime("%Y-%m-%d_%I:%M:%S-%p")
            completeName = os.path.join(self.save_path, file_name)

            # Write the joint names and efforts to a .txt file.
            with open(completeName, "w") as f:
                writer = csv.writer(f)
                writer.writerow(self.joints)
                writer.writerows(self.joint_effort)


    def main(self):
        """
        Function that initiates the issue_command function.
        :param self: The self reference.
        """
        # The arguments of the main function of the hm.HelloNode class are the
        # node_name, node topic namespace, and boolean (default value is true).
        hm.HelloNode.main(self, 'issue_command', 'issue_command', wait_for_first_pointcloud=False)
        rospy.loginfo('issuing command...')
        self.issue_command()
        time.sleep(2)


if __name__ == '__main__':
	try:
    	# Initialize the SingleJointActuator() class and set it to node and run the
    	# main() function.
		node = JointActuatorEffortSensor()
		node.main()
	except KeyboardInterrupt:
		rospy.loginfo('interrupt received, so shutting down')
