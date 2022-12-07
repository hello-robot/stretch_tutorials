![](../images/banner.png)
# Tutorial Track: Stretch ROS

## Robot Operating System (ROS)
Despite the name, ROS is not an operating system. ROS is a middleware framework that is a collection of transport protocol, development and debugging tools, and open-source packages, all wrapped in a single blanket. As a transport protocol, it allows distributed communication via messages through nodes. It provides development tools such as the CMake build system that allows building applications written in languages like Python and C++, among many, and the launch system that allows running mutiple nodes simultaneously. Due to its popularity, it's also the open-source framework of choice for collaboration across research and industry.

At Hello Robot, we strongly support the open-source initiative and believe that the best way to grow is to grow with the community.

This tutorial track is for users looking to become familiar with programming the Stretch RE1 and RE2 via ROS Noetic. We recommend going through the tutorials in the following order:

## Basics

|   | Tutorial                                                                        | Description                                        |
|---|---------------------------------------------------------------------------------|----------------------------------------------------|
| 1 | [Getting Started](getting_started.md)                                           | Setup instructions for ROS on Stretch. |
| 2 | [Gazebo Basics](gazebo_basics.md)                                               | Use Stretch in a simulated environment with Gazebo. |
| 3 | [Teleoperating Stretch](teleoperating_stretch.md)                               | Control Stretch with a keyboard or xbox controller. |
| 4 | [Internal State of Stretch](internal_state_of_stretch.md)                       | Monitor the joint states of Stretch. |
| 5 | [RViz Basics](rviz_basics.md)                                                   | Visualize topics in Stretch. |
| 6 | [Navigation Stack](navigation_stack.md)                                         | Motion planning and control for the mobile base using Nav stack. |
| 7 | [MoveIt! Basics](moveit_basics.md)                                              | Motion planning and control for the arm using MoveIt. |
| 8 | [Follow Joint Trajectory Commands](follow_joint_trajectory.md)                  | Control joints using joint trajectory server. |
| 9 | [Perception](perception.md)                                                     | Use the Realsense D435i camera to visualize the environment. |
| 10 | [ArUco Marker Detection](aruco_marker_detection.md)                             | Localize objects using ArUco markers. |
| 11 | [ReSpeaker Microphone Array](respeaker_microphone_array.md)                     | Learn to use the ReSpeaker Microphone Array. |
| 12 | [FUNMAP](https://github.com/hello-robot/stretch_ros/tree/master/stretch_funmap) | Fast Unified Navigation, Manipulation and Planning. |


## Other Examples
To help get you get started on your software development, here are examples of nodes to have the stretch perform simple tasks.


|   | Tutorial                                        | Description                                        |
|---|-------------------------------------------------|----------------------------------------------------|
| 1 | [Teleoperate Stretch with a Node](example_1.md) |  Use a python script that sends velocity commands.  |
| 2 | [Filter Laser Scans](example_2.md)              |  Publish new scan ranges that are directly in front of Stretch.|
| 3 | [Mobile Base Collision Avoidance](example_3.md) |  Stop Stretch from running into a wall.|
| 4 | [Give Stretch a Balloon](example_4.md)          |  Create a "balloon" marker that goes where ever Stretch goes.|
| 5 | [Print Joint States](example_5.md)              |  Print the joint states of Stretch.|
| 6 | [Store Effort Values](example_6.md)             |  Print, store, and plot the effort values of the Stretch robot.|
| 7 | [Capture Image](example_7.md)                   |  Capture images from the RealSense camera data.|
| 8 | [Voice to Text](example_8.md)                   |  Interpret speech and save transcript to a text file.|
| 9 | [Voice Teleoperation of Base](example_9.md)     |  Use speech to teleoperate the mobile base.|
| 10 | [Tf2 Broadcaster and Listener](example_10.md)   |  Create a tf2 broadcaster and listener.|
| 11 | [PointCloud Transformation](example_11.md)      |  Convert PointCloud2 data to a PointCloud and transform to a different frame.|
| 12 | [ArUco Tag Locator](example_12.md)              |  Actuate the head to locate a requested ArUco marker tag and return a transform.|
| 13 | [2D Navigation Goals](example_13.md)            |  Send 2D navigation goals to the move_base ROS node.|
