# Perception Introduction

The Stretch robot is equipped with the Intel RealSense D435i camera, an essential component that allows the robot to measure and analyze the world around it. In this tutorial, we are going to showcase how to visualize the various topics published from the camera.

Begin by switching your stretch_ros repository to the [feature/upright_camera_view](https://github.com/hello-robot/stretch_ros/tree/feature/upright_camera_view). The physical configuration of the camera results in the images being displayed sideways. Thus, this branch publishes a new topic that rotates the raw image upright.

```bash
cd ~/catkin_ws/src/stretch_ros/stretch_core
git checkout feature/upright_camera_view
```
Then run the stretch driver launch file.

```bash
# Terminal 1
roslaunch stretch_core stretch_driver.launch
```

To activate the RealSense camera and publish topics to be visualized, run the following launch file in a new terminal.

```bash
# Terminal 2
roslaunch stretch_core d435i_low_resolution.launch
```

Within this tutorial package, there is an RViz config file with the topics for perception already in the Display tree. You can visualize these topics and the robot model by running the command below in a new terminal.

```bash
# Terminal 3
rosrun rviz rviz -d /catkin_ws/src/stretch_ros_tutorials/rviz/perception_example.rviz
```


<p align="center">
  <img src="images/perception_rviz.gif"/>
</p>


<p align="center">
  <img src="images/perception_image.gif"/>
</p>


<p align="center">
  <img src="images/perception_camera.gif"/>
</p>


<p align="center">
  <img src="images/perception_depth.gif"/>
</p>

```bash
# Terminal 4
rosrun stretch_core keyboard_teleop
```
