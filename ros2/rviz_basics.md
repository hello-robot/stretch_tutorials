## Visualizing with RViz

You can utilize RViz to visualize Stretch's sensor information. To begin, run the stretch driver launch file.

```{.bash .shell-prompt}
ros2 launch stretch_core stretch_driver.launch.py
```

Then run the following command to bring up a simple RViz configuration of the Stretch robot.

```{.bash .shell-prompt}
ros2 run rviz2 rviz2 -d `ros2 pkg prefix --share stretch_calibration`/rviz/stretch_simple_test.rviz
```

An RViz window should open, allowing you to see the various DisplayTypes in the display tree on the left side of the window.

![image](https://raw.githubusercontent.com/hello-robot/stretch_tutorials/ROS2/images/simple_rviz.png)

If you want to visualize Stretch's [tf transform tree](http://wiki.ros.org/rviz/DisplayTypes/TF), you need to add the display type to the RViz window. First, click on the *Add* button and include the *TF*  type to the display. You will then see all of the transform frames of the Stretch robot and the visualization can be toggled off and on by clicking the checkbox next to the tree. Below is a gif for reference.

![image](https://raw.githubusercontent.com/hello-robot/stretch_tutorials/ROS2/images/rviz_adding_tf.gif)

There are further tutorials for RViz that can be found [here](http://wiki.ros.org/rviz/Tutorials).

## Visualizing camera data

The Stretch robot is equipped with the [Intel RealSense D435i camera](https://www.intelrealsense.com/depth-camera-d435i/), an essential component that allows the robot to measure and analyze the world around it. In this tutorial, we are going to showcase how to visualize the various topics published by the camera.

Begin by running the stretch `driver.launch.py` file.

```{.bash .shell-prompt}
ros2 launch stretch_core stretch_driver.launch.py
```

To activate the [RealSense camera](https://www.intelrealsense.com/depth-camera-d435i/) and publish topics to be visualized, run the following launch file in a new terminal.

```{.bash .shell-prompt}
ros2 launch stretch_core d435i_low_resolution.launch.py
```

Within this tutorial package, there is an [RViz config file](https://github.com/hello-robot/stretch_tutorials/blob/noetic/rviz/perception_example.rviz) with the topics for perception already in the Display tree. You can visualize these topics and the robot model by running the command below in a new terminal.

```{.bash .shell-prompt}
ros2 run rviz2 rviz2 -d /home/hello-robot/ament_ws/src/stretch_tutorials/rviz/perception_example.rviz
```

## PointCloud2 Display

A list of displays on the left side of the interface can visualize the camera data. Each display has its properties and status that notify a user if topic messages are received.

For the `PointCloud2` display, a [sensor_msgs/pointCloud2](http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/PointCloud2.html) message named `/camera/depth/color/points` is received and the GIF below demonstrates the various display properties when visualizing the data.

<p align="center">
  <img src="https://raw.githubusercontent.com/hello-robot/stretch_tutorials/noetic/images/perception_rviz.gif"/>
</p>

## Image Display
The `Image` display when toggled creates a new rendering window that visualizes a [sensor_msgs/Image](http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/Image.html) messaged, */camera/color/image_raw*. This feature shows the image data from the camera; however, the image comes out sideways.

<p align="center">
  <img src="https://raw.githubusercontent.com/hello-robot/stretch_tutorials/noetic/images/perception_image.gif"/>
</p>

## DepthCloud Display
The `DepthCloud` display is visualized in the main RViz window. This display takes in the depth image and RGB image provided by RealSense to visualize and register a point cloud.

<p align="center">
  <img src="https://raw.githubusercontent.com/hello-robot/stretch_tutorials/noetic/images/perception_depth.gif"/>
</p>

## Deep Perception
Hello Robot also has a ROS package that uses deep learning models for various detection demos. A link to the tutorials is provided: [stretch_deep_perception](https://docs.hello-robot.com/latest/ros2/deep_perception/).
