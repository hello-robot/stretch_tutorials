Help for the upright view, checkout out the /feature/upright_camera_view in the stretch_core package.

```bash
cd ~/catkin_ws/src/stretch_ros/stretch_core
git checkout feature/upright_camera_view
```

```bash
roslaunch stretch_core stretch_driver.launch
```

```bash
roslaunch stretch_core d435i_low_resolution.launch
```

```bash
rosrun rviz rviz -d /catkin_ws/src/stretch_ros_tutorials/rviz/perception_example.rviz
```

```bash
rosrun stretch_core keyboard_teleop
```
