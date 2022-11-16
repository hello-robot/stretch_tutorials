# Nav2 Stack Using RViz
In this tutorial, we will explore the ROS 2 navigation stack using slam_toolbox for mapping an environment and the core Nav2 packages to navigate in the mapped environment. If you want to know more about teleoperating the mobile base or working with the RPlidar 2D scanner on Stretch and how to get it up and running, we recommend visiting the previous tutorials on teleoperating stretch and laser scanner. These topics are a vital part of how Stretch's mobile base can be velocity controlled using Twist messages, and how the RPlidar's LaserScan messages enable obstacle avoidance for autonomous navigation.

Navigation is a key aspect of an autonomous agent because, often, to do anything meaningful, the agent needs to traverse an environment to reach a specific spot to perform a specific task. With an assistive robot like Stretch, the task could be anything from delivering water or medicines for the elderly to performing a routine patrol of an establishment for security.

Stretch's mobile base enables this capability and this tutorial will explore how we can autonomously plan and execute mobile base trajectories. Running this tutorial will require the robot to be untethered, so please ensure that the robot is adequetly charged.

## Mapping
The first step is to map the space that the robot will navigate in. The `mapping.launch.py` file will enable you to do this. First run:

```bash
ros2 launch stretch_navigation mapping.launch.py
```

Rviz will show the robot and the map that is being constructed. First, click on the "Startup" button in the RViz window to activate the lifecycle nodes. Now, use the xbox controller (or keyboard) to teleoperate the robot around. To teleoperate the robot using the xbox controller, keep the front left (LB) button pressed while using the right joystick for translation and rotation.

Avoid sharp turns and revisit previously visited spots to form loop closures.

**Insert Image for joystick**

**Insert GIF for mapping**
<p align="center">
  <img height=600 src="https://raw.githubusercontent.com/hello-robot/stretch_tutorials/ROS2/images/mapping.gif"/>
</p>

In Rviz, once you see a map that has reconstructed the space well enough, you can run the following commands to save the map to `stretch_navigation/` and `stretch_user/` directories.

```bash
mkdir ~/ament_ws/src/stretch_ros2/stretch_navigation/maps
ros2 run nav2_map_server map_saver_cli -f ~/ament_ws/src/strech_ros2/stretch_navigation/maps/<map_name>
```

The `<map_name>` does not include an extension. The map_saver node will save two files as `<map_name>.pgm` and `<map_name>.yaml`.

If you want to preserve the generated map or use it across different ROS distributions, copy the files in the `stretch_user` directory.

```bash
mkdir -p ~/stretch_user/maps
cp ~/ament_ws/src/stretch_ros2/stretch_navigation/maps/* ~/stretch_user/maps
```

## Navigation
Next, with `<map_name>.yaml`, we can navigate the robot around the mapped space. Run:

```bash
ros2 launch stretch_navigation navigation.launch map:=~/ament_ws/src/stretch_ros2/stretch_navigation/maps/<map_name>.yaml
```

Rviz will show the robot in the previously mapped space, however, it's likely that the robot's location in the map does not match the robot's location in the real space. In the top bar of Rviz, use 2D Pose Estimate to lay an arrow down roughly where the robot is located in the real space. AMCL, the localization package, will better localize our pose once we give the robot a 2D Nav Goal. 

In the top bar of Rviz, use 2D Nav Goal to lay down an arrow where you'd like the robot to go. In the terminal, you'll see Nav2 go through the planning phases and then navigate the robot to the goal. If planning fails, the robot will begin a recovery behavior - spinning around 180 degrees in place or backing up.

**Insert GIF for navigation**
<p align="center">
  <img height=600 src="https://raw.githubusercontent.com/hello-robot/stretch_tutorials/ROS2/images/mapping.gif"/>
</p>

## Note
The launch files expose the launch argument "teleop_type". By default, this argument is set to "joystick", which launches joystick teleop in the terminal with the xbox controller that ships with Stretch RE1. The xbox controller utilizes a dead man's switch safety feature to avoid unintended movement of the robot. This is the switch located on the front left side of the controller marked "LB". Keep this switch pressed and translate or rotate the base using the joystick located on the right side of the xbox controller.

If the xbox controller is not available, the following commands will launch mapping or navigation, respectively, with keyboard teleop:

```bash
ros2 launch stretch_navigation mapping.launch teleop_type:=keyboard
```
or
```bash
ros2 launch stretch_navigation navigation.launch teleop_type:=keyboard
```

## Simple Commander API
It is also possible to send 2D Pose Estimates and Nav Goals programatically. In your own launch file, you may include `navigation.launch` to bring up the navigation stack. Then, you can send pose goals using the Nav2 simple commander API in order to navigate the robot programatically. We will explore this in the next tutorial.
