![](../images/banner.png)
# Tutorial Track: Stretch ROS 2 (Beta)

!!! note
    Stretch's ROS2 packages and this ROS2 tutorial track are both under active development. They are considered 'beta', and we welcome any feedback. If you find any issues or bugs in the [stretch_ros2](https://github.com/hello-robot/stretch_ros2) repository, please see the [Stretch ROS2](https://github.com/hello-robot/stretch_ros2/issues) and [Stretch Tutorials](https://github.com/hello-robot/stretch_tutorials/issues) issue trackers.

## Robot Operating System 2 (ROS 2)

Despite the name, ROS is not an operating system. ROS is a middleware framework that is a collection of transport protocols, development and debugging tools, and open-source packages. As a transport protocol, ROS enables distributed communication via messages between nodes. As a development and debugging toolkit, ROS provides build systems that allow for writing applications in a wide variety of languages (Python and C++ are used in this tutorial track), a launch system to manage the execution of multiple nodes simultaneously, and command line tools to interact with the running system. Finally, as a popular ecosystem, there are many open-source ROS packages that allow users to quickly prototype with new sensors, actuators, planners, perception stacks, and more.

This tutorial track is for users looking to get familiar with programming Stretch robots via ROS 2. We recommend going through the tutorials in the following order:

## Basics

|  | Tutorial                                                                        | Description                                        |
|--|---------------------------------------------------------------------------------|----------------------------------------------------|
| 1 | [Getting Started](getting_started.md)                                              | Setup instructions for ROS 2 on Stretch|
| 2 | [Introduction to ROS 2](intro_to_ros2.md.md)                                       | Explore the client library used in ROS2 |
| 3 | [Introduction to HelloNode](intro_to_hellonode.md)                                 | Explore the Hello Node class to create a ROS2 node for Stretch |
| 4 | [Teleoperating Stretch](teleoperating_stretch.md)                                  | Control Stretch with a Keyboard or a Gamepad controller. |
| 5 | [Internal State of Stretch](internal_state_of_stretch.md)                          | Monitor the joint states of Stretch. |
| 6 | [RViz Basics](rviz_basics.md)                                                      | Visualize topics in Stretch. |
| 7 | [Nav2 Stack](navigation_overview.md)                                               | Motion planning and control for mobile base. |
| 8 | [Follow Joint Trajectory Commands](follow_joint_trajectory.md)                     | Control joints using joint trajectory server. |
| 9 | [Perception](perception.md)                                                        | Use the Realsense D435i camera to visualize the environment. |
| 10 | [ArUco Marker Detection](aruco_marker_detection.md)                               | Localize objects using ArUco markers. |
| 11 | [ReSpeaker Microphone Array](respeaker_mic_array.md)                              | Learn to use the ReSpeaker Microphone Array. |
| 12 | [FUNMAP](https://github.com/hello-robot/stretch_ros2/tree/humble/stretch_funmap)  | Fast Unified Navigation, Manipulation and Planning. |
<!--| 5 | [MoveIt2 Basics](moveit_basics.md)                                              | Motion planning and control for the arm using MoveIt. |
| 6 | [MoveIt2 with Rviz](moveit_rviz_demo.md)                                              | Motion planning and control for the arm using MoveIt. |
| 7 | [MoveIt2 MoveGroup C++ API](moveit_movegroup_demo.md)                                 | Motion planning and control for the arm using MoveIt. |


| 13 | [ROS testing](coming_soon.md)                                                   | Write ROS system tests for introspection. |
| 14 | [Other Nav Stack Features](coming_soon.md)                               | Advanced features for Nav 2. | -->

## Other Examples
To help get you started on your software development, here are examples of nodes to have Stretch perform simple tasks.


|   | Tutorial                                        | Description                                        |
|---|-------------------------------------------------|----------------------------------------------------|
| 1 | [Mobile Base Velocity Control](example_1.md)                                   |  Use a python script that sends velocity commands.  | 
| 2 | [Filter Laser Scans](example_2.md)                                                |  Publish new scan ranges that are directly in front of Stretch.| 
| 3 | [Mobile Base Collision Avoidance](example_3.md)                                   |  Stop Stretch from running into a wall.| 
| 4 | [Give Stretch a Balloon](example_4.md)                                            |  Create a "balloon" marker that goes where ever Stretch goes.|
| 5 | [Print Joint States](example_5.md)                                                |  Print the joint states of Stretch.| 
| 6 | [Store Effort Values](example_6.md)                                               |  Print, store, and plot the effort values of the Stretch robot.| 
| 7 | [Capture Image](example_7.md)                                                             |  Capture images from the RealSense camera data.| 
| 8 | [Voice to Text](example_8.md)                                                             |  Interpret speech and save transcript to a text file.| 
| 9 | [Voice Teleoperation of Base](example_9.md)                                               |  Use speech to teleoperate the mobile base.|
| 10 | [Tf2 Broadcaster and Listener](example_10.md)                                            |  Create a tf2 broadcaster and listener.|
| 11 | [ArUco Tag Locator](example_12.md)                               |  Actuate the head to locate a requested ArUco marker tag and return a transform.|
| 12 | [Obstacle Avoider](obstacle_avoider.md)                                                  |  Avoid obstacles using the planar lidar. |
| 13 | [Align to ArUco](align_to_aruco.md)                                                      |  Detect ArUco fiducials using OpenCV and align to them.|
| 14 | [Deep Perception](deep_perception.md)                                                    |  Use YOLOv5 to detect 3D objects in a point cloud.|
| 15 | [Avoiding Race Conditions and Deadlocks](avoiding_deadlocks_race_conditions.md)          | Learn how to avoid Race Conditions and Deadlocks |
<!--| 11 | [PointCloud Transformation](example_11.md)      |  Convert PointCloud2 data to a PointCloud and transform to a different frame.| 

