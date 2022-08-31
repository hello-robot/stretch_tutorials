# Introduction

This repo provides instructions on installing and using code on the Stretch RE1 robot. The goal is to provide a user familiar with ROS with the tools to operate a Stretch robot.

## Stretch ROS Tutorials
1. [Getting Started](getting_started.md)
2. [Gazebo Basics](gazebo_basics.md)
3. [Teleoperating Stretch](teleoperating_stretch.md)
4. [Internal State of Stretch](internal_state_of_stretch.md)
5. [RViz Basics](rviz_basics.md)
6. [Navigation Stack](navigation_stack.md)
7. [MoveIt! Basics](moveit_basics.md)
8. [Follow Joint Trajectory Commands](follow_joint_trajectory.md)
9. [Perception](perception.md)
10. [ArUco Marker Detection](aruco_marker_detection.md)
11. [ReSpeaker Microphone Array](respeaker_microphone_array.md)
12. [FUNMAP](https://github.com/hello-robot/stretch_ros/tree/master/stretch_funmap)
<!-- 13. ROS testing
14. Other Nav Stack Features -->


## Other ROS Examples
To help get you get started on your software development, here are examples of nodes to have the stretch perform simple tasks.

1. [Teleoperate Stretch with a Node](example_1.md) - Use a python script that sends velocity commands.  
2. [Filter Laser Scans](example_2.md) - Publish new scan ranges that are directly in front of Stretch.
3. [Mobile Base Collision Avoidance](example_3.md) - Stop Stretch from running into a wall.
4. [Give Stretch a Balloon](example_4.md) - Create a "balloon" marker that goes where ever Stretch goes.
5. [Print Joint States](example_5.md) - Print the joint states of Stretch.
6. [Store Effort Values](example_6.md) - Print, store, and plot the effort values of the Stretch robot.
7. [Capture Image](example_7.md) - Capture images from the RealSense camera data.
8. [Voice to Text](example_8.md) - Interpret speech and save transcript to a text file.
9. [Voice Teleoperation of Base](example_9.md) - Use speech to teleoperate the mobile base.
10. [Tf2 Broadcaster and Listener](example_10.md) - Create a tf2 broadcaster and listener.
11. [PointCloud Transformation](example_11.md) - Convert PointCloud2 data to a PointCloud and transform to a different frame.
12. [ArUco Tag Locator](example_12.md) - Actuate the head to locate a requested ArUco marker tag and return a transform.
13. [2D Navigation Goals](example_13.md) - Send 2D navigation goals to the move_base ROS node.  
