# Spawning Stretch in Simulation (Gazebo)

### Empty World Simulation
To spawn the Stretch in gazebo's default empty world run the following command in your terminal.
```
roslaunch stretch_gazebo gazebo.launch
```
This will bringup the robot in the gazebo simulation similar to the image shown below.

<!-- <img src="images/stretch_gazebo_empty_world.png" width="500" align="center"> -->
![image](images/stretch_gazebo_empty_world.png)

### Custom World Simulation
In gazebo, you are able to spawn Stretch in various worlds. First, source the gazebo world files by running the following command in a terminal
```
echo "source /usr/share/gazebo/setup.sh"
```


Then using the world argument, you can spawn the stretch in the willowgarage world by running the following

```
roslaunch stretch_gazebo gazebo.launch world:=worlds/willowgarage.world
```

![image](images/stretch_willowgarage_world.png)

**Next Tutorial:** [Teleoperating Stretch](teleoperating_stretch.md)
