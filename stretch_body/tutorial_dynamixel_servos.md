
# Working with Dynamixel Servos

In this tutorial we will go into the details with Dynamixel servos and Stretch.


## Overview

Stretch comes with two Dynamixel buses - one for the head and one for the end-of-arm:

```bash
>>$ ls  /dev/hello-dynamixel-*
/dev/hello-dynamixel-head  /dev/hello-dynamixel-wrist
```

Typically, users will interact with these devices through either the [Head](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/head.py) or [EndOfArm](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/end_of_arm.py) interfaces.  This tutorial is for users looking to work directly with the servos from the provided servo tools or through Stretch Body's low level Dynamixel API. 

## Servo Tools

**NOTE**: The servo tools here are part of the [Stretch Factory package](https://github.com/hello-robot/stretch_factory) which is installed as a part of Stretch Body.

### Jogging the Servos

You can directly command each servo using the command line tool `REx_dynamixel_servo_jog.py`. This can be useful for debugging new servos added to the end-of-arm tool during system bring-up. For example, to command the head pan servo:

```bash
$ REx_dynamixel_jog.py /dev/hello-dynamixel-head 11
[Dynamixel ID:011] ping Succeeded. Dynamixel model number : 1080
------ MENU -------
m: menu
a: increment position 50 tick
b: decrement position 50 tick
A: increment position 500 ticks
B: decrement position 500 ticks
v: set profile velocity
u: set profile acceleration
z: zero position
h: show homing offset
o: zero homing offset
q: got to position
p: ping
r: reboot
w: set max pwm
t: set max temp
i: set id
d: disable torque
e: enable torque
-------------------
```

### Rebooting the Servos

Under high-load conditions the servos may enter an error state to protect themselves from thermal overload. In this case, the red LED on the servo will flash (if visible). In addition, the servo will be unresponsive to motion commands. In this case, allow the overheating servo to cool down and reboot the servos using the `stretch_robot_dynamixel_reboot.py` tool: 

```bash
$ stretch_robot_dynamixel_reboot.py
For use with S T R E T C H (TM) RESEARCH EDITION from Hello Robot Inc.

---- Rebooting Head ---- 
[Dynamixel ID:011] Reboot Succeeded.
[Dynamixel ID:012] Reboot Succeeded.
---- Rebooting Wrist ---- 
[Dynamixel ID:013] Reboot Succeeded.
[Dynamixel ID:014] Reboot Succeeded.
```

### Identify Servos on the Bus

If it is unclear which servos are on the bus, and at what baud rate, you can use the `REx_dynamixel_id_scan.py` tool. Here we see that the two head servos are at ID 11 and 12 at baud 57600.

```bash
$ RE1_dynamixel_id_scan.py /dev/hello-dynamixel-head --baud 57600
Scanning bus /dev/hello-dynamixel-head at baud rate 57600
----------------------------------------------------------
[Dynamixel ID:000] ping Failed.
[Dynamixel ID:001] ping Failed.
[Dynamixel ID:002] ping Failed.
[Dynamixel ID:003] ping Failed.
[Dynamixel ID:004] ping Failed.
[Dynamixel ID:005] ping Failed.
[Dynamixel ID:006] ping Failed.
[Dynamixel ID:007] ping Failed.
[Dynamixel ID:008] ping Failed.
[Dynamixel ID:009] ping Failed.
[Dynamixel ID:010] ping Failed.
[Dynamixel ID:011] ping Succeeded. Dynamixel model number : 1080
[Dynamixel ID:012] ping Succeeded. Dynamixel model number : 1060
[Dynamixel ID:013] ping Failed.
[Dynamixel ID:014] ping Failed.
[Dynamixel ID:015] ping Failed.
[Dynamixel ID:016] ping Failed.
[Dynamixel ID:017] ping Failed.
[Dynamixel ID:018] ping Failed.
[Dynamixel ID:019] ping Failed.
[Dynamixel ID:020] ping Failed.
[Dynamixel ID:021] ping Failed.
[Dynamixel ID:022] ping Failed.
[Dynamixel ID:023] ping Failed.
[Dynamixel ID:024] ping Failed.
```

### Setting the Servo Baud Rate

Stretch ships with its Dynamixels servos configured to baudrate=115200.  When adding your own servos to the end-of-arm tool, you may want to set the servo baud using the `RE1_dynamixel_set_baud.py` tool. For example:

```bash
$ REx_dynamixel_set_baud.py /dev/hello-dynamixel-wrist 13 115200
---------------------
Checking servo current baud for 57600
----
Identified current baud of 57600. Changing baud to 115200
Success at changing baud
```
**Note**: Earlier units of Stretch RE1 may be running Dynamixel servos at baud 57600
### Setting the Servo ID

Dynamixel servos come with ID=1 from the factory. When adding your own servos to the end-of-arm tool, you may want to set the servo ID using the `REx_dynamixel_id_change.py` tool. For example:

```bash
$ RE1x_dynamixel_id_change.py /dev/hello-dynamixel-wrist 1 13 --baud 115200
[Dynamixel ID:001] ping Succeeded. Dynamixel model number : 1080
Ready to change ID 1 to 13. Hit enter to continue:

[Dynamixel ID:013] ping Succeeded. Dynamixel model number : 1080
Success at setting ID to 13
```

## Stretch Body Dynamixel API

Stretch Body's low level Dynamixel API includes a hierarchy of three classes

| Class                                                        |
| ------------------------------------------------------------ |
| [DynamixelXChain](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/dynamixel_X_chain.py) |
| [DynamixelHelloXL430](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/dynamixel_hello_XL430.py) |
| [DynamixelXL430](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/dynamixel_XL430.py) |

**NOTE**: The naming of XL430 is for legacy reasons. These classes will work with all X Series servos. 

### DynamixelXChain

DynamixelXChain manages a set of daisy-chained servos on a single bus (for example the head_pan and head_tilt servos). It allows for greater communication bandwidth by doing group read/write over USB. 

The [EndOfArm](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/end_of_arm.py) class derives from DynamixelXChain in order to provide an extensible interface that supports a user integrating additional DOF to the robot. The tutorial [Adding Custom Wrist DOF](./tutorial_extending_wrist_dof.md) explains how to do this.

### DynamixelHelloXL430

DynamixelHelloXL430 provides an interface to servo motion that is consistent with the Stretch Body lift, arm, and base joints. It also manages the servo parameters and calibration. Let's explore this interface further. From iPython let's look at the status message for DynamixelHelloXL430

```bash
import stretch_body.dynamixel_hello_XL430 

m = stretch_body.dynamixel_hello_XL430.DynamixelHelloXL430('head_pan')
m.startup()

m.pretty_print()
----- HelloXL430 ------ 
Name head_pan
Position (rad) -0.0
Position (deg) -0.0
Position (ticks) 1250
Velocity (rad/s) -0.0
Velocity (ticks/s) 0
Effort (%) 0.0
Effort (ticks) 0
Temp 34.0
Comm Errors 0
Hardware Error 0
Hardware Error: Input Voltage Error:  False
Hardware Error: Overheating Error:  False
Hardware Error: Motor Encoder Error:  False
Hardware Error: Electrical Shock Error:  False
Hardware Error: Overload Error:  False
Watchdog Errors:  0
Timestamp PC 1661552966.7202659
Range (ticks) [0, 3827]
Range (rad) [ 1.9174759848570513  ,  -3.953068490381297 ]
Stalled True
Stall Overload False
Is Calibrated 0

```

We see that it reports the position in both radians (with respect to the joint frame) and ticks (with respect to the servo encoder). DynamixelHelloXL430 handles the calibration between the two using its method `ticks_to_world_rad` through the following params:

```bash
>>$ stretch_params.py | grep head_pan | grep '_t '
...              
stretch_configuration_params.yaml         param.head_pan.range_t           [0, 3827]                     
stretch_configuration_params.yaml         param.head_pan.zero_t            1250 
```

In addition to `move_to` and `move_by`, the class also implements a splined trajectory interface as discussed in the [Splined Trajectory Tutorial](./tutorial_splined_trajectory.py). 

### DynamixelXL430

 DynamixelXL430 provides a thin wrapper to the [Robotis Dynamixel SDK](http://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#control-table). You may chose to interact with the servo at this level as well. For example to jog the head_pan 200 ticks:
```python
import stretch_body.dynamixel_XL430
import time

m = stretch_body.dynamixel_XL430.DynamixelXL430(11, '/dev/hello-dynamixel-head',baud=115200)
m.startup()

x=m.get_pos() #In encoder ticks
m.go_to_pos(x+200) #Move 200 ticks incremental

time.sleep(2.0)
m.stop()
```



------
<div align="center"> All materials are Copyright 2022 by Hello Robot Inc. The Stretch RE1 robot has patents pending</div>
