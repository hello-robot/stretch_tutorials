# Getting Started

## Prerequisites

1. A Stretch robot (see below for simulation instructions if you don’t have a robot)
2. Follow the [Getting Started]() guide (hello_robot_xbox_teleop must not be running in the background)
3. Interacting with Linux through the [command line](https://ubuntu.com/tutorials/command-line-for-beginners#1-overview)
4. Basic understanding of [ROS](http://wiki.ros.org/ROS/Tutorials)
5. Setup [untethered operation](https://docs.hello-robot.com/0.2/stretch-tutorials/getting_started/untethered_operation/) (optional)

### Connecting a Monitor
If you cannot access the robot through ssh due to your network settings, you will need to connect an HDMI monitor, USB keyboard, and mouse to the USB ports in the robot's trunk.

## Setting Up Stretch in Simulation
Users who don’t have a Stretch, but want to try the tutorials can set up their computer with Stretch Gazebo.

### Requirements
Although lower specifications might be sufficient, for the best experience we recommend the following for running the simulation:

* **Processor**: Intel i7 or comparable
* **Memory**: 16 GB
* **Storage**: 50 GB
* **OS**: Ubuntu 20.04
* **Graphics Card**: NVIDIA GTX2060 (optional)

### Setup
Hello Robot is currently running Stretch on Ubuntu 20.04 and ROS Noetic. To begin the setup, follow the [Run the new robot installation script](https://github.com/hello-robot/stretch_install/blob/master/docs/robot_install.md#run-the-new-robot-installation-script) on your system.

Finally, follow the [Creating a new ROS workspace](https://github.com/hello-robot/stretch_install/blob/master/docs/ros_workspace.md) guide to create a fresh catkin workspace complete with all the dependencies.

To begin working with a simulated Stretch, follow the [Gazebo basics](https://docs.hello-robot.com/0.2/stretch-tutorials/ros1_melodic/gazebo_basics/) tutorial.