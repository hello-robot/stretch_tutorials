# Spawning Stretch in Simulation (Gazebo)

## Empty World Simulation
To spawn Stretch in Gazebo's default empty world run the following command in your terminal.

```bash
roslaunch stretch_gazebo gazebo.launch
```

This will bring up the robot in the gazebo simulation similar to the image shown below.

<p align="center">
  <img src="https://raw.githubusercontent.com/hello-robot/stretch_tutorials/noetic/images/stretch_gazebo_empty_world.png"/>
</p>

## Custom World Simulation
In Gazebo, you can spawn Stretch in various worlds. First, source the Gazebo world files by running the following command in a terminal:

```bash
echo "source /usr/share/gazebo/setup.sh"
```

Then using the world argument, you can spawn Stretch in the Willow Garage world by running the following:

```bash
roslaunch stretch_gazebo gazebo.launch world:=worlds/willowgarage.world
```

<p align="center">
  <img src="https://raw.githubusercontent.com/hello-robot/stretch_tutorials/noetic/images/stretch_willowgarage_world.png"/>
</p>
