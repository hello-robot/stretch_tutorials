## Teleoperating Stretch

### Xbox Controller Teleoperating

Make sure your Xbox controller is connected, then run `ros2 launch stretch_core stretch_driver.launch.py mode:=gamepad` to start the robot driver in gamepad mode.

### Keyboard Teleoperating: Full Body

For full-body teleoperation with the keyboard, you first need to run the `stretch_driver.launch.py` in a terminal.

```{.bash .shell-prompt}
ros2 launch stretch_core stretch_driver.launch.py
```

Then in a new terminal, type the following command

```{.bash .shell-prompt}
ros2 run stretch_core keyboard_teleop
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

To stop the node from sending twist messages, type **Ctrl** + **c**.

### Keyboard Teleoperating: Mobile Base

Begin by running the following command in your terminal:

```{.bash .shell-prompt}
ros2 launch stretch_core stretch_driver.launch.py mode:=navigation
```

To teleoperate a Stretch's mobile base with the keyboard, you first need to switch the mode to *nagivation* for the robot to receive *Twist* messages. In comparison with ROS1 that we needed to use the rosservice command, we can do it in the same driver launch as you can see in the command you just input! Now in other terminal run the teleop_twist_keyboard node with the argument remapping the *cmd_vel* topic name to *stretch/cmd_vel*.

```{.bash .shell-prompt}
ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=stretch/cmd_vel
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

To stop the node from sending twist messages, type **Ctrl** + **c**.

### Create a node for Mobile Base Teleoperating
To move Stretch's mobile base using a python script, please look at [Teleoperate Stretch with a Node](twist_control.md) for reference.

