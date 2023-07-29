#!/usr/bin/env python3

import rclpy
from rclpy.duration import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from hello_helpers.hello_misc import HelloNode
import time


class MultiPointCommand(HelloNode):
    def __init__(self):
        HelloNode.__init__(self)
        HelloNode.main(self, 'multipoint_command', 'multipoint_command', wait_for_first_pointcloud=False)

    def issue_multipoint_command(self):
        while not self.joint_state.position:
            self.get_logger().info("Waiting for joint states message to arrive")
            time.sleep(0.1)
            continue
        
        self.get_logger().info('Issuing multipoint command...')
        joint_state = self.joint_state
        duration0 = Duration(seconds=0.0)
        duration1 = Duration(seconds=6.0)
        duration2 = Duration(seconds=9.0)
        duration3 = Duration(seconds=12.0)
        duration4 = Duration(seconds=16.0)
        duration5 = Duration(seconds=20.0)

        lift_index = joint_state.name.index('joint_lift')
        arm_index = joint_state.name.index('wrist_extension')
        wrist_yaw_index = joint_state.name.index('joint_wrist_yaw')
        gripper_index = joint_state.name.index('joint_gripper_finger_left')
        joint_value1 = joint_state.position[lift_index]
        joint_value2 = joint_state.position[arm_index]
        joint_value3 = joint_state.position[wrist_yaw_index]
        joint_value4 = joint_state.position[gripper_index]
        point0 = JointTrajectoryPoint()
        point0.positions = [joint_value1, joint_value2, joint_value3, joint_value4]
        point0.velocities = [0.0, 0.0, 0.0, 0.0]
        point0.time_from_start = duration0.to_msg()
        point1 = JointTrajectoryPoint()
        point1.positions = [0.9, 0.0, 0.0, 0.0] 
        point1.time_from_start = duration1.to_msg()
        point2 = JointTrajectoryPoint()
        point2.positions = [0.9, 0.2, 0.0, -0.3]
        point2.time_from_start = duration2.to_msg()
        point3 = JointTrajectoryPoint()
        point3.positions = [0.9, 0.4, 0.0, -0.3]
        point3.time_from_start = duration3.to_msg()
        point4 = JointTrajectoryPoint()
        point4.positions = [0.9, 0.4, 0.0, 0.0]
        point4.time_from_start = duration4.to_msg()
        point5 = JointTrajectoryPoint()
        point5.positions = [0.4, 0.0, 1.54, 0.0]
        point5.time_from_start = duration5.to_msg()
        trajectory_goal = FollowJointTrajectory.Goal()
        trajectory_goal.trajectory.joint_names = ['joint_lift', 'wrist_extension', 'joint_wrist_yaw', 'joint_gripper_finger_left']
        trajectory_goal.trajectory.points = [point0, point1, point2, point3, point4, point5]
        trajectory_goal.trajectory.header.frame_id = 'base_link'
        self.trajectory_client.send_goal_async(trajectory_goal)
        self.get_logger().info("Goal sent")
        
    def main(self):
        self.issue_multipoint_command()


def main():
    try:
        node = MultiPointCommand()
        node.main()
        node.new_thread.join()
    except KeyboardInterrupt:
        node.get_logger().info("Exiting")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
