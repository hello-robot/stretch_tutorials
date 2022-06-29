#!/usr/bin/env python

# Every python controller needs these lines
import rospy
import time
import actionlib
import os
import csv


from datetime import datetime
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

class SingleJointActuator(hm.HelloNode):
    """
    A class that sends multiple joint trajectory goals to a single joint.
    """
    # Initialize the inhereted hm.Hellonode class.
    def __init__(self):
        hm.HelloNode.__init__(self)

        # Set up a subscriber. We're going to subscribe to the topic "joint_states"
        self.sub = rospy.Subscriber('joint_states', JointState, self.callback)


        self.joints = ['joint_lift']

        self.joint_positions = []

        self.joint_effort = []

        # Create path to save effort and position values
        self.save_path = '/home/hello-robot/catkin_ws/src/stretch_ros_tutorials/stored_data'

        self.export = True

    def callback(self, msg):
        """
        Callback function to deal with the incoming JointState messages.
        :param self: The self reference.
        :param msg: The JointState message.
        """
        # Store the joint messages for later use
        self.joint_states = msg



    def issue_command(self):
        """
        Function that makes an action call and sends joint trajectory goals
        to a single joint
        :param self: The self reference.hello2020
        """

        # Set trajectory_goal as a FollowJointTrajectoryGoal and define
        # the joint name.
        trajectory_goal = FollowJointTrajectoryGoal()
        trajectory_goal.trajectory.joint_names = self.joints

        # Provide desired positions for joint name.
        # Set positions for the following 5 trajectory points.
        point0 = JointTrajectoryPoint()
        point0.positions = [0.4]

        # point1 = JointTrajectoryPoint()
        # point1.positions = [0.30]

        # Then trajectory_goal.trajectory.points is set as a list of the joint
        # trajectory points
        trajectory_goal.trajectory.points = [point0]#, point1]

        # Specify the coordinate frame that we want (base_link) and set the time to be now.
        trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
        trajectory_goal.trajectory.header.frame_id = 'base_link'

        # Make the action call and send the goal. The last line of code waits
        # for the result before it exits the python script.
        self.trajectory_client.send_goal(trajectory_goal, feedback_cb=self.feedback_callback, done_cb=self.done_callback)
        rospy.loginfo('Sent stow goal = {0}'.format(trajectory_goal))
        self.trajectory_client.wait_for_result()

    def feedback_callback(self,feedback):
        """
        print_states function to deal with the incoming JointState messages.
        :param self: The self reference.
        :param joints: A list of joint names.
        """
        # # Create an empty list that will store the positions of the requested joints
        # joint_effort = []

        # Use of forloop to parse the names of the requested joints list.
        # The index() function returns the index at the first occurrence of
        # the name of the requested joint in the self.joint_states.name list.
        for i in range(len(self.joints)):
            index = self.joint_states.name.index(self.joints[i])
            self.joint_effort.append(self.joint_states.effort[index])
            self.joint_positions.append(self.joint_states.position[index])

            # # Print the joint position values to the terminal
            # print("name: " + str(self.joints))
            # print("effort: " + str(joint_effort))


    def done_callback(self,status, result):
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo('Suceeded with result {0}'.format(result.SUCCESSFUL))
        else:
            rospy.loginfo('Failed with result {0}'.format(result.INVALID_GOAL))

        if self.export:
            file_name = datetime.now().strftime("%Y-%m-%d_%I:%M:%S-%p")
            completeName = os.path.join(self.save_path, file_name)

            rows = zip(self.joint_positions, self.joint_effort)

            with open(completeName, "w") as f:
                writer = csv.writer(f)
                for row in rows:
                    writer.writerow(row)


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
		node = SingleJointActuator()
		node.main()
	except KeyboardInterrupt:
		rospy.loginfo('interrupt received, so shutting down')
