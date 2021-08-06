## Visualizing with RViz

Let's bringup stretch in the willowgarage world from the [gazebo basics tutorial](gazebo_basics.md) and RViz by using the following command.

```
roslaunch stretch_gazebo stretch_willowgarage_world.launch rviz:=true
```
the `rviz` flag will open an RViz window  to visualize a variety of ROS topics.
![image](images/willowgarage_with_rviz.png)

If you want the visualize the [tf transform tree](http://wiki.ros.org/rviz/DisplayTypes/TF), you can add this to the your display. First click on the *Add* button and include the TF to the display. You will then see all of the transform frames of the Stretch robot and the visualization can be toggled off and on by clicking the checkbox next the tree. Below if a gif for reference.

![image](images/rviz_add_tf.gif)

<!-- Bringup the keyboard teleop to drive Stretch around. You will be able to visualize -->

Further tutorials for RViz can be found [here](http://wiki.ros.org/rviz/Tutorials).
