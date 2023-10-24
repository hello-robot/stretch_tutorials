# Intro to Navigation

## Display

Visualize the robot in Rviz:

```
roslaunch stretch_core stretch.launch lidar_odom:=false respeaker:=false rviz:=true
```

For more details on the arguments, see the [API docs](https://github.com/hello-robot/stretch_ros/tree/noetic/stretch_core#launch-files).

## Simulation

Visualize the simulated robot in Rviz:

```
roslaunch stretch_gazebo gazebo.launch rviz:=true
```

## Teleoperation

Switch to 'navigation' [mode](https://github.com/hello-robot/stretch_ros/tree/noetic/stretch_core#mode-std_msgsstring):

```
rosservice call /switch_to_navigation_mode
```

Run base keyboard teleop:

```
roslaunch stretch_core teleop_twist.launch
```

or if you have the Xbox controller:

```
roslaunch stretch_core teleop_twist.launch teleop_type:=joystick
```

The deadman button is the 'A' button (the green one).

![](https://docs.hello-robot.com/0.2/stretch-tutorials/getting_started/images/xbox.png)
