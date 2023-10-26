# Offload Compute to Remote Workstation
Bla Bla Bla  

#### Setting a ROS_DOMAIN_ID

Bla Bla Bla

## Robot Side Nodes

##### Start Stretch Driver Node
Starts bla bla
```
ros2 launch stretch_core stretch_driver.launch.py
```

##### Start Realsense Camera Stream Node
Starts bla bla
```
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
```
ros2 launch stretch_deep_perception stretch_detect_nearest_mouth.launch.py
```

##### Start Funmap Planner
Starts Bla Bla
```
ros2 launch stretch_funmap funmap.launch.py
```

##### Visualization from Rviz
```
```

### Workstation Setup to work with Stretch

This assumes the workstation side already has an active ROS2 distro (preferably Humble) colcon dependencies installed.
You can find [ROS2 Installation step for Ubuntu here](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Install-Binary.html#).

###### Setup Stretch ROS2 Packages on Workstation

```
source /opt/ros/humble/setup.bash
source /usr/share/colcon_cd/function/colcon_cd.sh
mkdir -p ament_ws/src
cd ament_ws/src/
git clone https://github.com/hello-robot/stretch_ros2
cd ..
rosdep install --rosdistro=humble -iyr --skip-keys="librealsense2" --from-paths src
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source ~/ament_ws/install/setup.bash
sudo apt install ros-humble-realsense2-description
```

```
cd ~/ament_ws/src/stretch_ros2/stretch_description/urdf/

#if Dex-Wrist Installed
cp stretch_description_dex.xacro stretch_description.xacro

#if Standard Gripper
cp stretch_description_standard.xacro stretch_description.xacro

ros2 run stretch_calibration update_uncalibrated_urdf
cp stretch_uncalibrated.urdf stretch.urdf

cd ~/ament_ws
colcon build
```

```
cd ~/ 
git clone https://github.com/hello-robot/stretch_deep_perception_models
```
TODO: [Parameterize models_directory](https://github.com/hello-robot/stretch_ros2/blob/humble/stretch_deep_perception/stretch_deep_perception/detect_nearest_mouth.py#L60)





