# Using a Remote Workstation to Offload Heavy Compute
In this tutorial, we will explore the method for offloading computationally intensive processes, such as running computer vision models, to a remote workstation computer. This approach offers several advantages such as:
- Save processing power of the Robot.
- Utilizing the available GPU hardware on powerful workstations to run large deep learning models.
- Ability to strategically offload less critical high-computation process to enhance Robot's efficiency.

In this tutorial, we will delve into the intricate process of **offloading [Stretch Deep Perception](https://github.com/hello-robot/stretch_ros2/tree/humble/stretch_deep_perception) ROS2 nodes**. These nodes are known for their demanding computational requirements and are frequently used in [Stretch Demos](https://github.com/hello-robot/stretch_ros2/tree/humble/stretch_demos). 

*NOTE: All Stretch ROS2 packages are developed with Humble distro.*

### Setting a ROS_DOMAIN_ID

ROS2 utilizes [DDS](https://design.ros2.org/articles/ros_on_dds.html) as the default middleware for communication. **DDS enables nodes within the same physical network to seamlessly discover one another and establish communication, provided they share the same `ROS_DOMAIN_ID`**. This powerful mechanism ensures secure message passing between remote nodes as intended.

By default, all ROS 2 nodes are configured with domain ID 0. To avoid conflicts, select a domain ID from the range of 0 to 101, and then set this chosen domain ID as the value for the `ROS_DOMAIN_ID` environment variable in both the Workstation and the Robot.
```{.bash .shell-prompt}
export ROS_DOMAIN_ID=<ID>
```

## Setup the Workstation to work with Stretch
The workstation needs to be installed with the appropriate stretch related ros2 packages to have access Stretch robot meshes for accurate Visualization in Rviz,interfaces dependencies and essential perception packages.

This section assumes the workstation already has an active ROS2 distro and colcon dependencies pre-installed.
You can find [ROS2 Installation step for Ubuntu here](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Install-Binary.html#).


##### Setup Essential stretch_ros2 Packages 

Make sure the ROS2 distro is sourced.
```{.bash .shell-prompt}
source /opt/ros/humble/setup.bash
```

Create workspace directory and clone stretch_ros2 packages along with it's dependency packages to `src` folder.
```{.bash .shell-prompt}
mkdir -p ~/ament_ws/src
cd ~/ament_ws/src/
git clone https://github.com/hello-robot/stretch_ros2
git clone https://github.com/hello-binit/ros2_numpy -b humble
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development
git clone https://github.com/Slamtec/sllidar_ros2.git -b main
git clone https://github.com/hello-binit/respeaker_ros2.git -b humble
git clone https://github.com/hello-binit/audio_common.git -b humble
```

Build and install all the packages present in source folder. 
```{.bash .shell-prompt}
cd ~/ament_ws
rosdep install --rosdistro=humble -iyr --skip-keys="librealsense2" --from-paths src
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Make sure to source the workspace to discover the packages in it.
```{.bash .shell-prompt}
source ~/ament_ws/install/setup.bash
```

##### Setup Robot URDF and Meshes
All the robots are shipped with an calibrated URDF files configured with robot version specific meshes that would more accurately match the actual robot you are using. So we recommend you to **copy the `stretch_description` directory that exists in your robot and replace it with the one existing in the workstation**. The Stretch Description directory exists in the path `~/ament_ws/src/stretch_ros2/stretch_description`.

If you dont want use the URDFs from the robot, you can manually generate the right URDF w.r.t your robot configuration using the following commands:
```{.bash .shell-prompt}
cd ~/ament_ws/src/stretch_ros2/stretch_description/urdf/

#if Dex-Wrist Installed
cp stretch_description_dex.xacro stretch_description.xacro

#if Standard Gripper
cp stretch_description_standard.xacro stretch_description.xacro

ros2 run stretch_calibration update_uncalibrated_urdf
cp stretch_uncalibrated.urdf stretch.urdf
```

After setting up the 
```
cd ~/ament_ws
colcon build
```

##### Download Stretch Deep Perception Models
```{.bash .shell-prompt}
cd ~/ 
git clone https://github.com/hello-robot/stretch_deep_perception_models
```
TODO: [Parameterize models_directory](https://github.com/hello-robot/stretch_ros2/blob/humble/stretch_deep_perception/stretch_deep_perception/detect_nearest_mouth.py#L60)



## Robot Side Nodes

##### Start Stretch Driver Node
Starts bla bla
```{.bash .shell-prompt}
ros2 launch stretch_core stretch_driver.launch.py
```

##### Start Realsense Camera Stream Node
Starts bla bla
```{.bash .shell-prompt}
ros2 launch stretch_core d435i_high_resolution.launch.py
```

##### Start RP Lidar Node
Starts bla bla
```
ros2 launch stretch_core rplidar.launch.py
```


## Workstation Side Node

##### Start 'Detect Nearest Mouth' Computer Vision Node
Starts Bla Bla
```{.bash .shell-prompt}
ros2 launch stretch_deep_perception stretch_detect_nearest_mouth.launch.py
```

##### Start Funmap Planner
Starts Bla Bla
```{.bash .shell-prompt}
ros2 launch stretch_funmap funmap.launch.py
```

##### Visualization from Rviz
```
```







