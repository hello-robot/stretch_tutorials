#!/usr/bin/env python3

import rclpy
from rclpy.duration import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from hello_helpers.hello_misc import HelloNode
import time


class StowCommand(HelloNode):
    def __init__(self):
        HelloNode.__init__(self)
        HelloNode.main(self, 'stow_command', 'stow_command', wait_for_first_pointcloud=False)

    def issue_stow_command(self):
        while not self.joint_state.position:
            self.get_logger().info("Waiting for joint states message to arrive")
            time.sleep(0.1)
            continue

        self.get_logger().info('Stowing...')
        joint_state = self.joint_state

        # Set two points as JointTrajectoryPoint(): 
        # stow_point1 is the current state, while stow_point2 is the goal state
        stow_point1 = JointTrajectoryPoint()
        stow_point2 = JointTrajectoryPoint()
        duration1 = Duration(seconds=0.0)
        duration2 = Duration(seconds=4.0)
        stow_point1.time_from_start = duration1.to_msg()
        stow_point2.time_from_start = duration2.to_msg()

        # Provide desired positions of lift, wrist extension, and wrist yaw
        lift_index = joint_state.name.index('joint_lift')
        arm_index = joint_state.name.index('wrist_extension')
        wrist_yaw_index = joint_state.name.index('joint_wrist_yaw')
        joint_value1 = joint_state.position[lift_index]
        joint_value2 = joint_state.position[arm_index]
        joint_value3 = joint_state.position[wrist_yaw_index]
        
        stow_point1.positions = [joint_value1, joint_value2, joint_value3]
        stow_point2.positions = [0.2, 0.0, 3.14]

        # Set trajectory_goal as a FollowJointTrajectoryGoal and define
        # the joint names as a list.
        trajectory_goal = FollowJointTrajectory.Goal()
        trajectory_goal.trajectory.joint_names = ['joint_lift', 'wrist_extension', 'joint_wrist_yaw']

        # Then trajectory_goal.trajectory.points is defined by the positions
        # set in stow_point1 and stow_point2
        trajectory_goal.trajectory.points = [stow_point1, stow_point2]

        # Make the action call and send the goal
        self.trajectory_client.send_goal_async(trajectory_goal)
        self.get_logger().info("Goal sent")

    def main(self):
        self.issue_stow_command()


# Create a function, main(), to do all the setup
def main():
    try:
        # Create object of the StowCommand class
        node = StowCommand()
        # Call the main() method of the StowCommand class
        node.main()
        node.new_thread.join()
    except:
        node.get_logger().info("Exiting")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
