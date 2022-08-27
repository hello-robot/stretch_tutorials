#!/usr/bin/env python3

# Import modules
import rospy
import time

# Import the FollowJointTrajectoryGoal from the control_msgs.msg package to
# control the Stretch robot
from control_msgs.msg import FollowJointTrajectoryGoal

# Import JointTrajectoryPoint from the trajectory_msgs package to define
# robot trajectories
from trajectory_msgs.msg import JointTrajectoryPoint

# Import hello_misc script for handling trajectory goals with an action client
import hello_helpers.hello_misc as hm

class StowCommand(hm.HelloNode):
    '''
    A class that sends a joint trajectory goal to stow the Stretch's arm.
    '''
    def __init__(self):
        """
        Function that initializes the inhereted hm.HelloNode class.
        :param self: The self reference.
        """
        hm.HelloNode.__init__(self)

    def issue_stow_command(self):
        '''
        Function that makes an action call and sends stow postion goal.
        :param self: The self reference.
        '''
        # Set stow_point as a JointTrajectoryPoint()
        stow_point = JointTrajectoryPoint()
        stow_point.time_from_start = rospy.Duration(0.000)

        # Provide desired positions of lift (meters), wrist extension (meters), and yaw of
        # the wrist (radians)
        stow_point.positions = [0.2, 0.0, 3.4]

        # Set trajectory_goal as a FollowJointTrajectoryGoal and define
        # the joint names as a list
        trajectory_goal = FollowJointTrajectoryGoal()
        trajectory_goal.trajectory.joint_names = ['joint_lift', 'wrist_extension', 'joint_wrist_yaw']

        # Then trajectory_goal.trajectory.points is defined by the positions
        # set in stow_point
        trajectory_goal.trajectory.points = [stow_point]

        # Specify the coordinate frame that we want (base_link) and set the time to be now
        trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
        trajectory_goal.trajectory.header.frame_id = 'base_link'

        # Make the action call and send the goal. The last line of code waits
        # for the result before it exits the python script
        self.trajectory_client.send_goal(trajectory_goal)
        rospy.loginfo('Sent stow goal = {0}'.format(trajectory_goal))
        self.trajectory_client.wait_for_result()

    def main(self):
        '''
        Function that initiates stow_command function.
        :param self: The self reference.
        '''
        # The arguments of the main function of the hm.HelloNode class are the
        # node_name, node topic namespace, and boolean (default value is true)
        hm.HelloNode.main(self, 'stow_command', 'stow_command', wait_for_first_pointcloud=False)
        rospy.loginfo('stowing...')
        self.issue_stow_command()
        time.sleep(2)

if __name__ == '__main__':
    try:
        # Instanstiate a `StowCommand()` object and execute the main() method
        node = StowCommand()
        node.main()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')
