# Perception Introduction

The Stretch robot is equipped with the Intel RealSense D435i camera, an essential component that allows the robot to measure and analyze the world around it. In this tutorial, we are going to showcase how to visualize the various topics published from the camera.

Begin by checking out the [feature/upright_camera_view](https://github.com/hello-robot/stretch_ros/tree/feature/upright_camera_view) branch in the stretch_ros repository. The configuration of the camera results in the images being displayed sideways. Thus, this branch publishes a new topic that rotates the raw image upright.

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
rosrun rviz rviz -d /home/hello-robot/catkin_ws/src/stretch_ros_tutorials/rviz/perception_example.rviz
```

## PointCloud2 Display

A list of displays on the left side of the interface can visualize the camera data. Each display has its properties and status that notify a user if topic messages are received.

For the `PointCloud2` display, a [sensor_msgs/pointCloud2](http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/PointCloud2.html) message named */camera/depth/color/points*, is received and the gif below demonstrates the various display properties when visualizing the data.


<p align="center">
  <img src="images/perception_rviz.gif"/>
</p>

## Image Display
The `Image` display visualizes a [sensor_msgs/Image](http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/Image.html) messaged, */camera/color/image/raw*, in a new window that pops up when you click on the check box next to the display name. This feature shows the image data from the camera; however, the image comes out sideways. Thus, you can select the */camera/color/image/raw_upright_view* from the **Image Topic** options to get an upright view of the image.
<p align="center">
  <img src="images/perception_image.gif"/>
</p>

## Camera Display
The `Camera` display is similar to that of the `Image` display
<p align="center">
  <img src="images/perception_camera.gif"/>
</p>

## DepthCloud Display
<p align="center">
  <img src="images/perception_depth.gif"/>
</p>
