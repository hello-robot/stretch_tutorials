#!/usr/bin/env python3

# Every python controller needs these lines
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

# Import the FollowJointTrajectoryGoal from the control_msgs.action package to
# control the Stretch robot.
from control_msgs.action import FollowJointTrajectory

# Import JointTrajectoryPoint from the trajectory_msgs package to define
# robot trajectories.
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState


class MultiPointCommand(Node):
    # Initialize the inhereted hm.Hellonode class.
    def __init__(self):
        super().__init__('stretch_multipoint_command')

        # Create an action client that can communicate with the /stretch_controller/follow_joint_trajectory action server
        self.trajectory_client = ActionClient(self, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory')
        server_reached = self.trajectory_client.wait_for_server(timeout_sec=60.0)
        if not server_reached:
            self.get_logger().error('Unable to connect to arm action server. Timeout exceeded.')
            sys.exit()

        # Create a subscriber for the /stretch/joint_states topic
        self.subscription = self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 1)
        self.subscription

        self.get_logger().info('issuing multipoint command...')

    # Callback function for the /stretch/joint_states topic
    def joint_states_callback(self, joint_state):
        self.joint_state = joint_state

    def issue_multipoint_command(self):
        joint_state = self.joint_state
        duration0 = Duration(seconds=0.0)
        duration1 = Duration(seconds=2.0)
        duration2 = Duration(seconds=4.0)
        duration3 = Duration(seconds=6.0)
        duration4 = Duration(seconds=8.0)
        duration5 = Duration(seconds=10.0)
        
        # Provide desired positions of lift, wrist extension, and yaw of
        # the wrist
        joint_value1 = joint_state.position[1] # joint_lift is at index 1
        joint_value2 = joint_state.position[0] # wrist_extension is at index 0
        joint_value3 = joint_state.position[8] # joint_wrist_yaw is at index 8

        # Set point0 as a JointTrajectoryPoint().
        point0 = JointTrajectoryPoint()

        # Provide desired positions of lift, wrist extension, and yaw of
        # the wrist (in meters).
        point0.positions = [joint_value1, joint_value2, joint_value3]

        # Provide desired velocity of the lift (m/s), wrist extension (m/s),
        # and wrist yaw (rad/s).
        # IMPORTANT NOTE: The lift and wrist extension can only go up to 0.2 m/s!
        point0.velocities = [0.2, 0.2, 2.5]

        # Provide desired velocity of the lift (m/s^2), wrist extension (m/s^2),
        # and wrist yaw (rad/s^2).
        point0.accelerations = [1.0, 1.0, 3.5]

        point0.time_from_start = duration0.to_msg()

        # Set positions for the following 5 trajectory points.
        # IMPORTANT NOTE: If you do not provide any velocities or accelerations for the lift
        # or wrist extension, then they go to their default values. However, the
        # Velocity and Acceleration of the wrist yaw will stay the same from the
        # previous value unless updated.
        point1 = JointTrajectoryPoint()
        point1.positions = [0.3, 0.1, 2.0]
        point1.time_from_start = duration1.to_msg()

        point2 = JointTrajectoryPoint()
        point2.positions = [0.5, 0.2, -1.0]
        point2.time_from_start = duration2.to_msg()

        point3 = JointTrajectoryPoint()
        point3.positions = [0.6, 0.3, 0.0]
        point3.time_from_start = duration3.to_msg()

        point4 = JointTrajectoryPoint()
        point4.positions = [0.8, 0.2, 1.0]
        point4.time_from_start = duration4.to_msg()

        point5 = JointTrajectoryPoint()
        point5.positions = [0.5, 0.1, 0.0]
        point5.time_from_start = duration5.to_msg()

        # Set trajectory_goal as a FollowJointTrajectoryGoal and define
        # the joint names as a list.
        trajectory_goal = FollowJointTrajectory.Goal()
        trajectory_goal.trajectory.joint_names = ['joint_lift', 'wrist_extension', 'joint_wrist_yaw']

        # Then trajectory_goal.trajectory.points is defined by a list of the joint
        # trajectory points
        trajectory_goal.trajectory.points = [point0, point1, point2, point3, point4, point5]

        # Specify the coordinate frame that we want (base_link) and set the time to be now.
        trajectory_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory_goal.trajectory.header.frame_id = 'base_link'

        # Make the action call and send the goal. The last line of code waits
        # for the result before it exits the python script.
        self.trajectory_client.send_goal_async(trajectory_goal)
        self.get_logger().info('Sent stow goal = {0}'.format(trajectory_goal))

def main(args=None):
    rclpy.init(args=args)

    multipoint_command = MultiPointCommand()

    rclpy.spin_once(multipoint_command)
    multipoint_command.issue_multipoint_command()

    rclpy.spin(multipoint_command)

    multipoint_command.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
