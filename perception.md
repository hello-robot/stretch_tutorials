## Perception Introduction

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
rosrun rviz rviz -d /home/hello-robot/catkin_ws/src/stretch_tutorials/rviz/perception_example.rviz
```

### PointCloud2 Display

A list of displays on the left side of the interface can visualize the camera data. Each display has its properties and status that notify a user if topic messages are received.

For the `PointCloud2` display, a [sensor_msgs/pointCloud2](http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/PointCloud2.html) message named */camera/depth/color/points*, is received and the gif below demonstrates the various display properties when visualizing the data.


<p align="center">
  <img src="images/perception_rviz.gif"/>
</p>

### Image Display
The `Image` display when toggled creates a new rendering window that visualizes a [sensor_msgs/Image](http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/Image.html) messaged, */camera/color/image_raw*. This feature shows the image data from the camera; however, the image comes out sideways. Thus, you can select the */camera/color/image_raw_upright_view* from the **Image Topic** options to get an upright view of the image.
<p align="center">
  <img src="images/perception_image.gif"/>
</p>

### Camera Display
The `Camera` display is similar to that of the `Image` display. In this setting, the rendering window also visualizes other displays, such as the PointCloud2, the RobotModel, and Grid Displays. The **visibility** property can toggle what displays your are interested in visualizing.
<p align="center">
  <img src="images/perception_camera.gif"/>
</p>

### DepthCloud Display
The `DepthCloud` display is visualized in the main RViz window. This display takes in the depth image and RGB image, provided by the RealSense, to visualize and register a point cloud.
<p align="center">
  <img src="images/perception_depth.gif"/>
</p>


## Deep Perception
Hello Robot also has a ROS package that uses deep learning models for various detection demos. A link to the package is provided: [stretch_deep_perception](https://github.com/hello-robot/stretch_ros/tree/master/stretch_deep_perception).


**Next Tutorial:** [ReSpeaker Microphone Array](respeaker_mircophone_array.md)
