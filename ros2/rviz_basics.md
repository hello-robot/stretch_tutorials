## Visualizing with RViz

!!! note
    ROS 2 tutorials are still under active development. 

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
