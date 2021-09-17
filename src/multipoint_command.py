#!/usr/bin/env python3

import rospy
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import hello_helpers.hello_misc as hm
import time

class MultiPointCommand(hm.HelloNode):

    def __init__(self):
        hm.HelloNode.__init__(self)

    def issue_multipoint_command(self):
      point0 = JointTrajectoryPoint()
      point0.time_from_start = rospy.Duration(0.000)
      point0.positions = [0.2, 0.0, 3.4]
      point0.velocities = [0.2]
      point0.accelerations = [1.0]

      point1 = JointTrajectoryPoint()
      point1.time_from_start = rospy.Duration(0.5)
      point1.positions = [0.3, 0.1, 2.0]
      point1.velocities = [0.1]
      point1.accelerations = [-1.0]

      point2 = JointTrajectoryPoint()
      point2.time_from_start = rospy.Duration(1.0)
      point2.positions = [0.5, 0.2, 1.0]
      point2.velocities = [0.02]
      point2.accelerations = [-1.0]

      point3 = JointTrajectoryPoint()
      point3.time_from_start = rospy.Duration(1.5)
      point3.positions = [0.6, 0.3, 0.0]
      point3.velocities = [0.1]
      point3.accelerations = [1.0]

      point4 = JointTrajectoryPoint()
      point4.time_from_start = rospy.Duration(2.0)
      point4.positions = [0.8, 0.2, -1.0]
      point4.velocities = [0.2]
      point4.accelerations = [1]

      point5 = JointTrajectoryPoint()
      point5.time_from_start = rospy.Duration(2.5)
      point5.positions = [0.5, 0.1, 0.0]


      trajectory_goal = FollowJointTrajectoryGoal()
      trajectory_goal.trajectory.joint_names = ['joint_lift', 'wrist_extension', 'joint_wrist_yaw']
      trajectory_goal.trajectory.points = [point0, point1, point2, point3, point4, point5]
      trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
      trajectory_goal.trajectory.header.frame_id = 'base_link'

    def main(self):
        hm.HelloNode.main(self, 'multipoint_command', 'multipoint_command', wait_for_first_pointcloud=False)
        rospy.loginfo('issuing multipoint command...')
        self.issue_multipoint_multijoint_command()
        time.sleep(2)


if __name__ == '__main__':
    try:
        node = StowCommand()
        node.main()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')
