## Teleoperating in Gazebo

### Keyboard Teloperation
First, [Startup Stretch in simulation](gazebo_basics.md). Then type the following the command for teleoperating a Stretch robot's mobile base with the keyboard.

```
roslaunch stretch_gazebo teleop_keyboard.launch
```
Below is the keyboard commands that allow a user to move Stretch in gazebo.  
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

## Teleoperating the Actual Robot
