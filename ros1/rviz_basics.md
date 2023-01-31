## Visualizing with RViz

You can utilize RViz to visualize Stretch's sensor information. To begin, in a terminal, run the stretch driver launch file.

```bash
roslaunch stretch_core stretch_driver.launch
```

Then run the following command in a separate terminal to bring up a simple RViz configuration of the Stretch robot.
```bash
rosrun rviz rviz -d `rospack find stretch_core`/rviz/stretch_simple_test.rviz
```
An RViz window should open, allowing you to see the various DisplayTypes in the display tree on the left side of the window.

<p align="center">
  <img src="https://raw.githubusercontent.com/hello-robot/stretch_tutorials/noetic/images/simple_rviz.png"/>
</p>

If you want to visualize Stretch's [tf transform tree](http://wiki.ros.org/rviz/DisplayTypes/TF), you need to add the display type to the RViz window. First, click the *Add* button and include the *TF*  type in the display. You will then see all of the transform frames of the Stretch robot, and the visualization can be toggled off and on by clicking the checkbox next to the tree. Below is a gif for reference.

<p align="center">
  <img src="https://raw.githubusercontent.com/hello-robot/stretch_tutorials/noetic/images/rviz_adding_tf.gif"/>
</p>

There are further tutorials for RViz which can be found [here](http://wiki.ros.org/rviz/Tutorials).


## Running RViz and Gazebo (Simulation)
Let's bring up Stretch in the willow garage world from the [gazebo basics tutorial](gazebo_basics.md) and RViz by using the following command.

```bash
roslaunch stretch_gazebo gazebo.launch world:=worlds/willowgarage.world rviz:=true
```
the `rviz` flag will open an RViz window to visualize a variety of ROS topics.

<p align="center">
  <img src="https://raw.githubusercontent.com/hello-robot/stretch_tutorials/noetic/images/willowgarage_with_rviz.png"/>
</p>

Bring up the [keyboard teleop](teleoperating_stretch.md) node to drive Stretch and observe its sensor input.