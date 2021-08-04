# Getting Started

## Ubuntu

Hello Robot utilizes Ubuntu, an open source Linux operating system, for the Stretch RE1 platform. If you are unfamiliar with the operating system, we encourage you to review a [tutorial](https://ubuntu.com/tutorials/command-line-for-beginners#1-overview) provided by Ubuntu. Additionally, the Linux command line, BASH, is used to execute commands and is needed to run ROS on the Stretch robot. Here is a [tutorial](https://ryanstutorials.net/linuxtutorial/) on getting started with BASH.

## ROS Setup

Hello Robot is currently running Stretch on Ubuntu 20.04 and on ROS Noetic. Instructions on installing Noetic can be found in our open source [installation guide](https://github.com/hello-robot/stretch_ros/blob/dev/noetic/install_noetic.md). To begin the setup, clone the [stretch_ros](https://github.com/hello-robot/stretch_ros.git) package to your preferred workspace, then build the workspace.
```
cd catkin_ws/src/
git clone https://github.com/hello-robot/stretch_ros.git
cd ..
catkin build
```

## RoboMaker

If a user is unable to dual boot and install ubuntu their local machine, an alternative is to use [AWS RoboMaker](https://aws.amazon.com/robomaker/) for simulation and testing the Stretch RE1 platform.
