# Stretch Modes
Stretch Driver allows the robot to be commanded in several modes of operation. Understanding what these modes are and what behaviors they enable is central to using the robot effectively.

## Position mode
Position mode enables position control of the arm, head and mobile base with sequential incremental positions achieved using the move_by() method in the underlying Python interface to the robot (Stretch Body). It disables velocity control of the mobile base through the /cmd_vel topic. The position commands to various joints are honored by the Joint Trajectory Server through the /stretch_controller/follow_joint_trajectory action interface.

To understand how to command joints using the Joint Trajectory Server, refer to the [Follow Joint Trajectory Commands](https://docs.hello-robot.com/0.2/stretch-tutorials/ros2/follow_joint_trajectory/) tutorial.

## Trajectory mode
Trajectory mode is able to execute plans from high level planners like MoveIt2. These planners are able to generate waypoint trajectories for each joint for smooth motion profiles in velocity and acceleration space. The joint trajectory action server, and the underlying Python interface to the robot (Stretch Body) execut the trajectory respecting each waypoints' time_from_start attribute of the trajectory_msgs/JointTrajectoryPoint message. This allows coordinated motion of the base + arm. To understand better how this is achieved, it might be instructive to look at the [Trajectory API](https://docs.hello-robot.com/0.2/stretch-tutorials/stretch_body/tutorial_splined_trajectories/) tutorial in Stretch Body.

To understand how to command joints using the Joint Trajectory Server, refer to the [Follow Joint Trajectory Commands](https://docs.hello-robot.com/0.2/stretch-tutorials/ros2/follow_joint_trajectory/) tutorial.

## Navigation mode
Navigation mode enables mobile base velocity control via the /cmd_vel topic, and disables position-based control of the mobile base as in position mode. The arm and head joints remain commandable in position mode.

To understand how to command the mobile base using navigation mode, refer to the [Mobile Base Velocity Control](https://docs.hello-robot.com/0.2/stretch-tutorials/ros2/example_1/) tutorial.