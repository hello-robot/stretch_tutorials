![](../images/banner.png)
# Tutorial Track: Stretch ROS 2

This tutorial series covers writing ROS 2 software for Stretch. ROS 2 programs can be written in a variety of programming languages, but this series uses Python. We'll write programs that enable Stretch to navigate autonomously in its environment, manipulate objects with Stretch's gripper, perceive its environment, and much more.

## Prerequisites

Ensure that:

 1. Your Stretch has the latest robot distribution installed
    - These tutorials were written for the latest robot distribution. Take a look at the [Distributions & Roadmap](../../software/distributions/) guide to identify your current distribution and upgrade if necessary.
 2. You have gone through the [Getting Started Tutorials](../../getting_started/hello_robot/)
    - If you've never developed with Stretch before or are new to programming, check out the [Developing with Stretch](../../developing/basics/) tutorial series. In particular, the [Using ROS 2 with Stretch](#TODO) tutorial from that series is a good resource for those new to ROS 2.

## Robot Operating System 2 (ROS 2)

ROS 2 is the successor to ROS,  The ROS in ROS 2 stands for "robot operating system", but despite the name, ROS is not an operating system. It's a middleware framework that is a collection of transport protocols, development and debugging tools, and open-source packages.

As a transport protocol, ROS enables distributed communication via messages between nodes. As a development and debugging toolkit, ROS provides build systems that allow for writing applications in a wide variety of languages (Python and C++ are used in this tutorial track), a launch system to manage the execution of multiple nodes simultaneously, and command line tools to interact with the running system. Finally, as a popular ecosystem, there are many open-source ROS packages that allow users to quickly prototype with new sensors, actuators, planners, perception stacks, and more.


Despite the name, ROS is not an operating system. ROS is a middleware framework that is a collection of transport protocols, development and debugging tools, and open-source packages. As a transport protocol, ROS enables distributed communication via messages between nodes. As a development and debugging toolkit, ROS provides build systems that allow for writing applications in a wide variety of languages (Python and C++ are used in this tutorial track), a launch system to manage the execution of multiple nodes simultaneously, and command line tools to interact with the running system. Finally, as a popular ecosystem, there are many open-source ROS packages that allow users to quickly prototype with new sensors, actuators, planners, perception stacks, and more.

This tutorial track is for users looking to get familiar with programming Stretch robots via ROS 2. We recommend going through the tutorials in the following order:

## Basics

|  | Tutorial                                                                        | Description                                        |
|--|---------------------------------------------------------------------------------|----------------------------------------------------|
| 1 | [Creating your own package, launch files, nodes](writing_nodes.md)                                              | Setup instructions for ROS 2 on Stretch|
| 2 | [Simulation Tutorial](stretch_simulation.md)                                       | Explore the client library used in ROS2 |
| 3 | [Teleoperating Stretch](teleoperating_stretch.md)                                  | Control Stretch with a Keyboard or a Gamepad controller. |
| 4 | [RViz Basics](rviz_basics.md)                                                      | Visualize topics in Stretch. |
| 5 | [Follow Joint Trajectory Commands](follow_joint_trajectory.md)                     | Control joints using joint trajectory server. |
| 6 | [Introduction to HelloNode](intro_to_hellonode.md)                                 | Explore the Hello Node class to create a ROS2 node for Stretch |
| 7 | [Robot Driver](robot_drivers.md)                                               | ROS2 Wrapper for the python API. |
| 8 | [Twist Control](twist_control.md)                                               | Using Twist messages to control the mobile base. |
| 9 | [Sensors](sensors_tutorial.md)                                               | Stretch sensors including the ReSpeaker microphone array, IMU, bump sensors, and cliff sensors. |
| 10 | [Nav2 Stack](navigation_overview.md)                                               | Motion planning and control for mobile base. |
| 11 | [Perception](perception.md)                                                        | Use the Realsense D435i camera to visualize the environment. |
| 12 | [Deep Perception](deep_perception.md)                                                        | Perception using Deep Learning. |
| 13 | [ArUco Marker Detection](aruco_marker_detection.md)                               | Localize objects using ArUco markers. |
| 14 | [Offloading Computation Tutorial](remote_compute.md)                               | Offloading computationally intensive processes. |
| 15 | [Avoiding Race Conditions and Deadlocks](avoiding_deadlocks_race_conditions.md)          | Learn how to avoid Race Conditions and Deadlocks |
| 16 | [Autonomy Demos](demo_hello_world.md)                               | A few demos showcasing Stretch's autonomous capabilities. |
| 17 | [FUNMAP](https://github.com/hello-robot/stretch_ros2/tree/humble/stretch_funmap)  | Fast Unified Navigation, Manipulation and Planning. |



## Other Examples
To help get you started on your software development, here are examples of nodes to have Stretch perform simple tasks.


|   | Tutorial                                        | Description                                        |
|---|-------------------------------------------------|----------------------------------------------------|
| 1 | [Voice to Text](speech_to_text.md)                                                             |  Interpret speech and save transcript to a text file.| 
| 2 | [Voice Teleoperation of Base](voice_teleop.md)                                               |  Use speech to teleoperate the mobile base.|
| 3 | [Filter Laser Scans](lidar_filtering.md)                                                |  Publish new scan ranges that are directly in front of Stretch.| 
| 4 | [Give Stretch a Balloon](rviz_markers.md)                                            |  Create a "balloon" marker that goes where ever Stretch goes.|
| 5 | [Align to ArUco](align_to_aruco.md)                                                      |  Detect ArUco fiducials using OpenCV and align to them.|
| 6 | [ArUco Tag Locator](aruco_locator.md)                               |  Actuate the head to locate a requested ArUco marker tag and return a transform.|
| 7 | [Print Joint States](joint_states.md)                                                |  Print the joint states of Stretch.| 
| 8 | [Store Effort Values](joint_effort_plotting.md)                                               |  Print, store, and plot the effort values of the Stretch robot.| 
| 9 | [Tf2 Broadcaster and Listener](tf2_transforms.md)                                            |  Create a tf2 broadcaster and listener.|
| 10 | [Capture Image](realsense_camera.md)                                                             |  Capture images from the RealSense camera data.| 
| 11 | [Mobile Base Collision Avoidance](collision_avoidance.md)                                   |  Stop Stretch from running into a wall.| 
| 12 | [Obstacle Avoider](obstacle_avoider.md)                                                  |  Avoid obstacles using the planar lidar. |
<!--| 11 | [PointCloud Transformation](example_11.md)      |  Convert PointCloud2 data to a PointCloud and transform to a different frame.| -->
