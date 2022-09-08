# Getting Started

**NOTE**: ROS 2 tutorials are still under active development. 

## Installing Ubuntu 20.04 with ROS 2 Galactic on Stretch
Hello Robot utilizes Ubuntu, an open source Linux operating system, for the Stretch RE1 platform. If you are unfamiliar with the operating system, we encourage you to review a [tutorial](https://ubuntu.com/tutorials/command-line-for-beginners#1-overview) provided by Ubuntu. Additionally, the Linux command line, BASH, is used to execute commands and is needed to run ROS on the Stretch robot. Here is a [tutorial](https://ryanstutorials.net/linuxtutorial/) on getting started with BASH.

Instructions on installing Ubuntu 20.04 with ROS Noetic and ROS 2 Galactic can be found in our open source [installation guide](https://github.com/hello-robot/stretch_ros/blob/dev/noetic/install_noetic.md). Following these steps should create a separate Ubuntu 20.04 partition with an ament worskspace created in the home directory.

## ROS 2 Tutorials Setup on Local Computer
Once your system is setup, clone the [stretch_ros_tutorials](https://github.com/hello-sanchez/stretch_ros_tutorials.git) repo to the src directory of the ament workspace, then build the packages.

```
cd ~/ament_ws/src
git clone https://github.com/hello-robot/stretch_tutorials.git
git checkout ROS2
cd ~/ament_ws
colcon build
```

Then source your workspace with the following command
```
source ~/ament_ws/install/setup.bash"
```

