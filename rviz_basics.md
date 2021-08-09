## Visualizing with RViz

Let's bringup stretch in the willowgarage world from the [gazebo basics tutorial](gazebo_basics.md) and RViz by using the following command.

```bash
roslaunch stretch_gazebo stretch_willowgarage_world.launch rviz:=true
```
the `rviz` flag will open an RViz window  to visualize a variety of ROS topics.
![image](images/willowgarage_with_rviz.png)

If you want the visualize the [tf transform tree](http://wiki.ros.org/rviz/DisplayTypes/TF), you can add this to the your display. First click on the *Add* button and include the *TF*  type to the display. You will then see all of the transform frames of the Stretch robot and the visualization can be toggled off and on by clicking the checkbox next the tree. Below is a gif for reference.

![image](images/rviz_adding_tf.gif)

Bringup the [keyboard teleop](teleoperating_stretch.md) to drive Stretch and observe its sensor input. Further tutorials for RViz can be found [here](http://wiki.ros.org/rviz/Tutorials).
