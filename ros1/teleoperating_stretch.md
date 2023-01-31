## Teleoperating Stretch

### Xbox Controller Teleoperating
If you have not already had a look at the [Xbox Controller Teleoperation](https://docs.hello-robot.com/0.2/stretch-tutorials/getting_started/quick_start_guide_re2/#hello-world-demo) section in the Quick Start guide, now might be a good time to try it.

### Keyboard Teleoperating: Full Body

For full-body teleoperation with the keyboard, you first need to run the `stretch_driver.launch` in a terminal.

```{.bash .shell-prompt}
roslaunch stretch_core stretch_driver.launch
```

Then in a new terminal, type the following command

```{.bash .shell-prompt}
rosrun stretch_core keyboard_teleop
```

Below are the keyboard commands that allow a user to control all of Stretch's joints.

```{.bash .no-copy}
---------- KEYBOARD TELEOP MENU -----------

              i HEAD UP                    
 j HEAD LEFT            l HEAD RIGHT       
              , HEAD DOWN                  


 7 BASE ROTATE LEFT     9 BASE ROTATE RIGHT
 home                   page-up            


              8 LIFT UP                    
              up-arrow                     
 4 BASE FORWARD         6 BASE BACK        
 left-arrow             right-arrow        
              2 LIFT DOWN                  
              down-arrow                   


              w ARM OUT                    
 a WRIST FORWARD        d WRIST BACK       
              x ARM IN                     


              5 GRIPPER CLOSE              
              0 GRIPPER OPEN               

  step size:  b BIG, m MEDIUM, s SMALL     

              q QUIT                       

-------------------------------------------
```

To stop the node from sending twist messages, press `Ctrl` + `c` in the terminal.

### Keyboard Teleoperating: Mobile Base

Begin by running the following command in your terminal:

```{.bash .shell-prompt}
roslaunch stretch_core stretch_driver.launch
```

To teleoperate a Stretch's mobile base with the keyboard, you first need to switch the mode to `navigation` for the robot to receive `Twist` messages. This is done using a rosservice call in a new terminal. In the same terminal run the teleop_twist_keyboard node with the argument remapping the `cmd_vel` topic name to `stretch/cmd_vel`.

```{.bash .shell-prompt}
rosservice call /switch_to_navigation_mode
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=stretch/cmd_vel
```

Below are the keyboard commands that allow a user to move Stretch's base.  

```{.bash .no-copy}
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

To stop the node from sending twist messages, type `Ctrl` + `c`.

### Create a node for Mobile Base Teleoperating
To move Stretch's mobile base using a python script, please look at [Teleoperate Stretch with a node](example_1.md) for reference.

## Teleoperating in Gazebo

### Keyboard Teleoperating: Mobile Base
For keyboard teleoperation of the Stretch's mobile base, first, [startup Stretch in simulation](gazebo_basics.md). Then run the following command in a new terminal.

```{.bash .shell-prompt}
roslaunch stretch_gazebo gazebo.launch
```

In a new terminal, type the following

```{.bash .shell-prompt}
roslaunch stretch_core teleop_twist.launch twist_topic:=/stretch_diff_drive_controller/cmd_vel linear:=1.0 angular:=2.0 teleop_type:=keyboard
```

The same keyboard commands will be presented to a user to move the robot.

### Xbox Controller Teleoperating
An alternative for robot base teleoperation is to use an Xbox controller. Stop the keyboard teleoperation node by typing `Ctrl` + `c` in the terminal where the command was executed. Then connect the Xbox controller device to your local machine and run the following command.

```{.bash .shell-prompt}
roslaunch stretch_core teleop_twist.launch twist_topic:=/stretch_diff_drive_controller/cmd_vel linear:=1.0 angular:=2.0 teleop_type:=joystick
```

Note that the teleop_twist_joy package has a deadman switch by default which disables the drive commands to be published unless pressed. For a Logitech F310 joystick, this button is A.