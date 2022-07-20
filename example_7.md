## Example 7

In this example, we will review the [image_view](http://wiki.ros.org/image_view?distro=melodic) ROS package and a Python script that captures an image from the RealSense camera.


<p align="center">
  <img src="images/camera_image.jpeg"/>
</p>



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

### Capture Image with image_view

There are a couple of methods to save an image using the image_view package.

**OPTION 1:** Use the `image_view` node to open a simple image viewer for ROS sensor_msgs/image topics.

```bash
# Terminal 4
rosrun image_view image_view image:=/camera/color/image_raw_upright_view
```
Then you can save the current image by right-clicking on the display window. By deafult, images will be saved as frame000.jpg, frame000.jpg, etc. Note, that the image will be saved to the terminal's current work directory.

**OPTION 2:** Use the `image_saver` node to save an image to the terminals current work directory.

```bash
# Terminal 4
rosrun image_view image_saver image:=/camera/color/image_raw_upright_view
```

### Capture Image with Python Script




### Edge Detection


<p align="center">
  <img src="images/camera_image_edge_detection.jpeg"/>
</p>
