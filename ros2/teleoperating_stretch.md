## Teleoperating Stretch

### NOTE
Teleoperation support for Stretch in ROS 2 is under active development. Please reach out to us if you want to teleoperate Stretch in ROS 2.

Refer to the instructions below if you want to test this functionality in ROS 1.

### Xbox Controller Teleoperating
![image](https://raw.githubusercontent.com/hello-robot/stretch_tutorials/ROS2/images/xbox_controller_commands.png)

Stretch comes ready to run out of the box. The Xbox Teleoperation demo will let you quickly test out the robot capabilities by teleoperating it with an Xbox Controller.

Note: Make sure the USB Dongle is plugged into the the USB port of the base trunk.

To start the demo:

* Remove the 'trunk' cover and power on the robot
Wait for about 45 seconds. You will hear the Ubuntu startup sound, followed by two beeps (indicating the demo is running).
* Hit the Connect button on the controller. The upper two LEDs of the ring will illuminate.
* Hit the Home Robot button. Stretch will go through its homing calibration routine.
* **Note**: make sure the space around the robot is clear before running the Home function

You're ready to go! A few things to try:

* Hit the Stow Robot button. The robot will assume the stow pose.
* Practice driving the robot around.
* Pull the Fast Base trigger while driving. When stowed, it will make Stretch drive faster
* Manually stop the arm or lift from moving to make it stop upon contact.
* Try picking up your cellphone from the floor
* Try grasping cup from a counter top
* Try delivering an object to a person

If you're done, hold down the Shutdown PC button for 2 seconds. This will cause the PC to turn off. You can then power down the robot.

### Keyboard Teleoperating

To teleoperate a Stretch's mobile base with the keyboard, you first need to set the ros parameter to *navigation* mode for the robot to receive *Twist* messages. Begin by running `roscore` in a terminal. Then in a new terminal, type the following commands

```
rosparam set /stretch_driver/mode "navigation"
roslaunch stretch_core stretch_driver.launch
```
The first line will set the stretch driver mode to *navigation* before running the `stretch_driver.launch` file.

Then in a new terminal launch the teleop_twist_keyboard node with the argument remapping the *cmd_vel* topic name to *stretch/cmd_vel*.
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=stretch/cmd_vel
```

Below are the keyboard commands that allow a user to move Stretch.  
```
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .
For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >
t : up (+z)
b : down (-z)
anything else : stop
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
CTRL-C to quit
currently:	speed 0.5	turn 1.0
```
To stop the node from sending twist messages, type **Ctrl** + **c**.

### Create a node for Teleoperating
To move Stretch's mobile base using a python script, please look at [example 1](example_1.md) for reference.


## Teleoperating in Gazebo


### Keyboard Teleoperating
For keyboard teleoperation, first [startup Stretch in simulation](gazebo_basics.md). Then run the following command in a new terminal.

```bash
roslaunch stretch_gazebo gazebo.launch
```

In a new terminal, type the following

```
roslaunch stretch_gazebo teleop_keyboard.launch
```
The same keyboard commands will be presented to a user to move the robot.

### Xbox Controller Teleoperating
An alternative for robot base teleoperation is to use an Xbox controller. Stop the keyboard teleoperation node by typing **Ctrl** + **c** in the terminal where the command was executed. Then connect the Xbox controller device to your local machine and run the following command.

```
roslaunch stretch_gazebo teleop_joy.launch
```
Note that the teleop_twist_joy package has a deadman switch by default which disables the drive commands to be published unless pressed. For a Logitech F310 joystick, this button is A.

