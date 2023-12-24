# Updating Stretch Software

Stretch's software is improved with new features and bug fixes with each update. In this guide, we cover when and how to update the various software components on your Stretch.

## When to Update

We develop our software publicly on GitHub, allowing anyone to follow and propose the development of a code feature or bug fix. While we wholeheartedly welcome collaboration on GitHub, it is not necessary to be active on GitHub to follow our software releases. We announce every major release of software on our [forum](https://forum.hello-robot.com/c/announcements). These are stable releases with code that has been extensively tested on many Stretch robots. To be notified of new releases, create an account on the forum and click the bell icon in the top left of the [announcements section](https://forum.hello-robot.com/c/announcements/6). The forum is also available to report issues and ask questions about any of our software packages.

## How to Update

Each Stretch is shipped with firmware, a Python SDK, and ROS packages developed specifically for Stretch. At the moment, there are three separate processes for updating each of these components.

### Stretch ROS

Stretch ROS is the [Robot Operating System](https://www.ros.org/about-ros/) (ROS) interface to the robot. Many robotics developers find ROS useful to bootstrap their robotics software developments. Depending on whether you want to set up a ROS or ROS 2 workspace, the easiest way to download the most recent updates in the stretch_ros and stretch_ros2 code repositories, while resolving all source-built dependencies at the same time, is by following the instructions in the [Creating a New ROS Workspace](https://github.com/hello-robot/stretch_install/blob/master/docs/ros_workspace.md) section in the stretch_install repo. 

!!! warning
    Before you proceed, please ensure that all your personal files in the catkin or ament workspace have been backed up safely. This is important because executing the following set of commands deletes your existing workspace and replaces it with a fresh one.

To download the stretch_install repo, execute:
```{.bash .shell-prompt}
cd ~/
git clone https://github.com/hello-robot/stretch_install.git
cd stretch_install
git pull
```

To replace the ROS Melodic catkin_ws in Ubuntu 18.04, execute:
```{.bash .shell-prompt}
./factory/18.04/stretch_create_catkin_workspace.sh -w <optional-path-to-ws>
```

To replace the ROS Noetic catkin_ws in Ubuntu 20.04, execute:
```{.bash .shell-prompt}
./factory/20.04/stretch_create_catkin_workspace.sh -w <optional-path-to-ws>
```

To replace the ROS 2 Galactic ament_ws in Ubuntu 20.04, execute:
```{.bash .shell-prompt}
./factory/20.04/stretch_create_ament_workspace.sh -w <optional-path-to-ws>
```

### Stretch Body Python SDK

Stretch Body is the Python SDK for the robot. It abstracts away the low-level details of communication with the embedded devices and provides an intuitive API for working with the robot. You may update it using the following commands depending on the Python version.

If you are using Python2, execute:
```{.bash .shell-prompt}
pip install -U hello-robot-stretch-body
pip install -U hello-robot-stretch-body-tools
pip install -U hello-robot-stretch-factory
pip3 install -U hello_robot_stretch_body_tools_py3
```

For Python3, execute:
```{.bash .shell-prompt}
python3 -m pip -q install --no-warn-script-location hello-robot-stretch-body
python3 -m pip -q install --no-warn-script-location hello-robot-stretch-body-tools
python3 -m pip -q install --no-warn-script-location hello-robot-stretch-factory
python3 -m pip -q install --no-warn-script-location hello-robot-stretch-tool-share
```

### Stretch Firmware

The firmware and the Python SDK (called Stretch Body) communicate on an established protocol. Therefore, it is important to maintain a protocol match between the different firmware and Stretch Body versions. Fortunately, there is a script that handles this automatically. In the command line, run the following command:

```{.bash .shell-prompt}
REx_firmware_updater.py --status
```

This script will automatically determine what version is currently running on the robot and provide a recommendation for the next step. Follow the next steps provided by the firmware updater script.

### Ubuntu

The operating system upon which Stretch is built is called Ubuntu. This operating system provides the underlying packages that power Stretch's software packages. Furthermore, users of Stretch depend on this operating system and the underlying packages to develop software on Stretch. Therefore, it is important to keep the OS and these underlying packages up to date. In the command line, run the following command:

```{.bash .shell-prompt}
sudo apt update
sudo apt upgrade
```

[Apt](https://en.wikipedia.org/wiki/APT_(software)) is the package manager that handles updates for all Ubuntu packages.

## Troubleshooting

### Firmware Mismatch Error

When working with Stretch Body, if you see the following error:

```{.bash .no-copy}
----------------
Firmware protocol mismatch on /dev/XXXX.
Protocol on board is pX.
Valid protocol is: pX.
Disabling device.
Please upgrade the firmware and/or version of Stretch Body.
----------------
```

This error appears because the low-level Python SDK and the firmware cannot communicate with each other. There is a protocol mismatch preventing communication between the two. Simply run the following script and follow its recommendations to upgrade/downgrade the firmware as necessary to match the protocol level of Stretch Body.

```{.bash .shell-prompt}
REx_firmware_updater.py --status
```

------
<div align="center"> All materials are Copyright 2022 by Hello Robot Inc. Hello Robot and Stretch are registered trademarks.</div>
