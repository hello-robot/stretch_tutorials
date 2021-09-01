## Navigation Stack with Actual robot

stretch_navigation provides the standard ROS navigation stack as two launch files. This package utilizes gmapping, move_base, and AMCL to drive the stretch RE1 around a mapped space. Running this code will require the robot to be untethered.


Then run the following commands to map the space that the robot will navigate in.
```bash
roslaunch stretch_navigation mapping.launch
```
Rviz will show the robot and the map that is being constructed. With the terminal open, use the instructions printed by the teleop package to teleoperate the robot around the room. Avoid sharp turns and revisit previously visited spots to form loop closures.

<p align="center">
  <img height=600 src="images/mapping.gif"/>
</p>

In Rviz, once you see a map that has reconstructed the space well enough, you can run the following commands to save the map to `stretch_user/`.

```bash
mkdir -p ~/stretch_user/maps
rosrun map_server map_saver -f ${HELLO_FLEET_PATH}/maps/<map_name>
```

The `<map_name>` does not include an extension. Map_saver will save two files as `<map_name>.pgm` and `<map_name>.yaml`.

Next, with `<map_name>.yaml`, we can navigate the robot around the mapped space. Run:

```bash
roslaunch stretch_navigation navigation.launch map_yaml:=${HELLO_FLEET_PATH}/maps/<map_name>.yaml
```

Rviz will show the robot in the previously mapped space, however, it's likely that the robot's location in the map does not match the robot's location in the real space. In the top bar of Rviz, use 2D Pose Estimate to lay an arrow down roughly where the robot is located in the real space. AMCL, the localization package, will better localize our pose once we give the robot a 2D Nav Goal. In the top bar of Rviz, use 2D Nav Goal to lay down an arrow where you'd like the robot to go. In the terminal, you'll see move_base go through the planning phases and then navigate the robot to the goal. If planning fails, the robot will begin a recovery behavior: spinning around 360 degrees in place.

It is also possible to send 2D Pose Estimates and Nav Goals programatically. In your own launch file, you may include `navigation.launch` to bring up the navigation stack. Then, you can send `move_base_msgs::MoveBaseGoal` messages in order to navigate the robot programatically.


<!-- ## Navigation Stack in Gazebo

To test Stretch Navigation in simulation there is a `mapping_gazebo.launch` and `navigation_gazebo.launch` files on the [feature/navigation_updates](https://github.com/hello-robot/stretch_ros/tree/feature/navigation_updates/stretch_navigation/launch) branch. Note that this branch works on ROS Melodic. Navigate to the branch by running the following

```
roscd stretch_navigation
git checkout feature/navigation_updates
```
Then bringup [Stretch in the willowgarage world](gazebo_basics.md) and in a new terminal run the following command to build a map of the Willow Garage world

```
roslaunch stretch_navigation mapping_gazebo.launch gazebo_visualize_lidar:=true gazebo_world:=worlds/willowgarage.world
```

```
roslaunch stretch_navigation teleop_keyboard.launch
```
 -->

**Next Tutorial:** [MoveIt! Basics](moveit_basics.md)
