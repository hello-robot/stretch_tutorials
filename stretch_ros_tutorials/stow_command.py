#!/usr/bin/env python3

# Every python controller needs these lines
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient

import sys

# Import the FollowJointTrajectoryGoal from the control_msgs.action package to
# control the Stretch robot.
from control_msgs.action import FollowJointTrajectory

# Import JointTrajectoryPoint from the trajectory_msgs package to define
# robot trajectories and JointState from sensor_msgs to store joint states
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState


class StowCommand(Node):
    def __init__(self):
        # Initialize a ROS node named stow_command
        super().__init__('stretch_stow_command')
        self.joint_state = JointState()

        # Create an action client that can communicate with the /stretch_controller/follow_joint_trajectory action server
        self.trajectory_client = ActionClient(self, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory')
        server_reached = self.trajectory_client.wait_for_server(timeout_sec=60.0)
        if not server_reached:
            self.get_logger().error('Unable to connect to arm action server. Timeout exceeded.')
            sys.exit()

        # Create a subscriber for the /stretch/joint_states topic
        self.subscription = self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 1)
        self.subscription

    # Callback function for the /stretch/joint_states topic
    def joint_states_callback(self, joint_state):
        self.joint_state = joint_state

    def issue_stow_command(self):
        joint_state = self.joint_state
        if (joint_state is not None):
            self.get_logger().info('stowing...')

            # Set two points as JointTrajectoryPoint(): stow_point1 is the current state, 
            # stow_point2 is the goal state
            stow_point1 = JointTrajectoryPoint()
            stow_point2 = JointTrajectoryPoint()
            duration1 = Duration(seconds=0.0)
            duration2 = Duration(seconds=4.0)
            stow_point1.time_from_start = duration1.to_msg()
            stow_point2.time_from_start = duration2.to_msg()

            # Provide desired positions of lift, wrist extension, and yaw of
            # the wrist
            joint_value1 = joint_state.position[1] # joint_lift is at index 1
            joint_value2 = joint_state.position[0] # wrist_extension is at index 0
            joint_value3 = joint_state.position[8] # joint_wrist_yaw is at index 8
            stow_point1.positions = [joint_value1, joint_value2, joint_value3]
            stow_point2.positions = [0.2, 0.0, 3.14]

            # Set trajectory_goal as a FollowJointTrajectoryGoal and define
            # the joint names as a list.
            trajectory_goal = FollowJointTrajectory.Goal()
            trajectory_goal.trajectory.joint_names = ['joint_lift', 'wrist_extension', 'joint_wrist_yaw']

            # Then trajectory_goal.trajectory.points is defined by the positions
            # set in stow_point1 and stow_point2
            trajectory_goal.trajectory.points = [stow_point1, stow_point2]

            # Specify the coordinate frame that we want (base_link) and set the time to be now
            trajectory_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
            trajectory_goal.trajectory.header.frame_id = 'base_link'

            # Make the action call and send the goal
            self.trajectory_client.send_goal_async(trajectory_goal)
            self.get_logger().info('Sent stow goal = {0}'.format(trajectory_goal))


# Create a funcion, main(), to do all of the setup the hm.HelloNode class
# and issue the stow command.
def main(args=None):
    rclpy.init(args=args)
     
    # Create object of the StowCommand class
    stow_command = StowCommand()
    
    # spin_once allows the joint_states_callback functions to be executed once 
    # for joint states to be received to this node
    rclpy.spin_once(stow_command)
    stow_command.issue_stow_command()

    rclpy.spin(stow_command)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    stow_command.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
