# MoveIt! Basics
<!--
## MoveIt! on Stretch
To run MoveIt with the actual hardware, (assuming `stretch_driver` is already running) simply run
```bash
roslaunch stretch_moveit_config move_group.launch
```
This will runs all of the planning capabilities, but without the setup, simulation and interface that the above demo provides. In order to create plans for the robot with the same interface as the offline demo, you can run
```bash
roslaunch stretch_moveit_config moveit_rviz.launch
``` -->

## MoveIt! Without Hardware
To begin running MoveIt! on stretch, checkout to the feature/hybrid_planning branch and run the demo launch file.

```bash
cd ~/ament_ws/src/stretch_ros2
git checkout feature/hybrid_planning
cd ~/ament_ws
colcon build
ros2 launch stretch_moveit_config movegroup_moveit2.launch.py
```
This will brining up an RViz instance where you can move the robot around using [interactive markers](http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Getting%20Started) and create plans between poses. You can reference the bottom gif as a guide to plan and execute motion.

![image](https://raw.githubusercontent.com/hello-robot/stretch_tutorials/ROS2/images/moveit.gif)


Additionally, the demo allows a user to select from the five groups, *stretch_arm*, *stretch_gripper*, *stretch_head*, *mobile_base_arm* and *position* to move. The option to change groups in the in *Planning Request* section in the *Displays* tree. A few notes to be kept in mind:

* Pre-defined start and goal states can be specified in Start State and Goal State drop downs in Planning tab of Motion Planning RViz plugin.

* *stretch_gripper* group does not show markers, and is intended to be controlled via the joints tab that is located in the very right of Motion Planning Rviz plugin.

* When planning with *stretch_head* group make sure you select *Approx IK Solutions* in Planning tab of Motion Planning RViz plugin.


![image](https://raw.githubusercontent.com/hello-robot/stretch_tutorials/ROS2/images/moveit_groups.gif)


## Running Gazebo with MoveIt! and Stretch


<!-- ```bash
# Terminal 1:
roslaunch stretch_gazebo gazebo.launch
# Terminal 2:
roslaunch stretch_gazebo teleop_keyboard.launch # or use teleop_joy.launch if you have a controller
# Terminal 3
roslaunch stretch_moveit_config demo_gazebo.launch
```
This will launch an Rviz instance that visualizes the joints with markers and an empty world in Gazebo with Stretch and load all the controllers. There are pre-defined positions for each joint group for demonstration purposes. There are three joint groups, namely stretch_arm, stretch_gripper and stretch_head that can be controlled individually via Motion Planning Rviz plugin. Start and goal positions for joints can be selected similar to [this moveit tutorial](https://ros-planning.github.io/moveit_tutorials/doc/quickstart_in_rviz/quickstart_in_rviz_tutorial.html#choosing-specific-start-goal-states).
![image](images/gazebo_moveit.gif) -->

**Next Tutorial:** [Follow Joint Trajectory Commands](follow_joint_trajectory.md)
