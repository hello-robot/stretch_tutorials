![](./images/banner.png)
# Working with Stretch Body
The Stretch_Body package provides a low level Python API to the Stretch RE1 hardware.  

The package is available on [Git and installable via Pip](https://github.com/hello-robot/stretch_body).

It encapsulates

* Mobile base 
* Arm 
* Lift 
* Head actuators
* Wrist and tool actuators
* Wrist accelerometer and Arduino
* Base power and IMU board

The robot's 3rd party hardware devices are intended to be accessed through ROS and not Stretch_Body. However, it is possible to directly access this hardware through open-source Python packages:

* Laser range finder:  [rplidar](https://github.com/SkoltechRobotics/rplidar)
* Respeaker: [respeaker_python_library](https://github.com/respeaker/respeaker_python_library)
* D435i: [pyrealsense2](https://pypi.org/project/pyrealsense2/)

The Stretch_Body package is intended for advanced users who prefer to not use ROS to control the robot. It assumes a moderate level of experience programming robot sensors and actuators.


## Robot Interface

The primary developer interface to  Stretch_Body is the [Robot class](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/robot.py).  

As an example, the Python script below prints all Robot sensor and state data to the console every 250ms. 

```python linenums="1"
import time
import stretch_body.robot

robot=stretch_body.robot.Robot()
robot.startup()

for i in range(10):
	robot.pretty_print()
	time.sleep(0.25)
	
robot.stop()

```

Looking at this in detail:

```python linenums="2"
import stretch_body.robot
```

The package stretch_body includes the Python module for Robot as well as other Devices such as Lift and Arm.

```python linenums="4"
robot=stretch_body.robot.Robot()
robot.startup()
```

Here we instantiate an instance of our Robot. The call to `startup()` opens the serial ports to the various devices, loads the Robot YAML parameters, and launches a few helper threads.

```python linenums="7"
for i in range(10):
	robot.pretty_print()
	time.sleep(0.25)
```

The call to `pretty_print()` prints to console all of the robot's sensor and state data.

```python linenums="11"
robot.stop()
```

Finally, the `stop()` method shuts down the Robot threads and cleanly closes the open serial ports.

### Units

The Robot API uses SI units of:

* meters
* radians
* seconds
* Newtons
* Amps
* Volts

Parameters may be named with a suffix to help describe the unit type. For example:

* pos_m : meters
* pos_r: radians

### The Robot Status

The Robot derives from the [Device class](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/device.py). It also encapsulates a number of other Devices:

* [robot.head](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/head.py)
* [robot.arm](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/arm.py)
* [robot.lift](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/lift.py)
* [robot.base](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/base.py)
* [robot.wacc](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/wacc.py)
* [robot.pimu](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/pimu.py)
* [robot.end_of_arm](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/end_of_arm.py)

All devices contain a Status dictionary. The Status contains the most recent sensor and state data of that device. For example, looking at the Arm class we see:

```python
class Arm(Device):
    def __init__(self):
        ...
		self.status = {'pos': 0.0, 'vel': 0.0, 'force':0.0, \
                       'motor':self.motor.status,'timestamp_pc':0}
```

The Status dictionaries are  automatically updated by a background thread of the Robot at 25Hz. The Status data can be accessed via the Robot. For example:

```python
if robot.arm.status['pos']>0.25:
    print('Arm extension greater than 0.25m')
```

If an instantaneous snapshot of the entire Robot Status is needed, the `get_status()` method can be used instead:

```python
status=robot.get_status()
if status['arm']['pos']>0.25:
    print('Arm extension greater than 0.25m')
```

### The Robot Command

In contrast to the Robot Status which pulls data from the Devices, the Robot Command pushes data to the Devices.

Consider the following example which extends and then retracts the arm by 0.1 meters:

```python linenums="1"
import time
import stretch_body.robot

robot=stretch_body.robot.Robot()
robot.startup()

robot.arm.move_by(0.1)
robot.push_command()
time.sleep(2.0) 

robot.arm.move_by(-0.1)
robot.push_command()
time.sleep(2.0)
	
robot.stop()
```

A few important things are going on:

```python linenums="7"
robot.arm.move_by(0.1)
```

The `move_by()` method queues up an RPC command to the stepper motor controller. However, the command does not yet execute.

```python linenums="8"
robot.push_command()
```

The `push_command()` causes all queued up RPC commands to be executed at once.  In this example we call `sleep()` to allow time for the motion to complete before initiating a new motion.

**NOTE**: The Dynamixel servos do not use the Hello Robot RPC protocol. As such, the head, wrist, and gripper will move immediately upon issuing a motion command. 

The stepper actuators support a synchronous mode, allowing the base, arm, and lift to synchronously track trajectories.  Thus, the following code will cause the base, arm, and lift to initiate motion simultaneously:

```python
robot.arm.move_by(0.1)
robot.lift.move_by(0.1)
robot.base.translate_by(0.1)
robot.push_command()
```

Commanding robot motion through the Stretch_Body interface is covered in more detail in the Robot Motion section.

### Stowing and Homing

After power up the robot requires homing in order for its joint encoders to find their zero position. The homing procedure will run the robot through a series of moves to find these zeros. It can be done programatically:

```python
if not robot.is_calibrated():
	robot.home() #blocking
```

Or it can be done manually after boot using the command line tool:

```console
$ stretch_robot_home.py
```

Likewise, stowing is a robot procedure that will cause it to move its arm and tool safely within the footprint of the base. 

```python
robot.stow() #blocking
```

Or it can be done manually from the command line when needed:

```console
$ stretch_robot_stow.py
```

## Scripting the Robot

A simplified design pattern to script the Robot is as follows

```python linenums="1"
#!/usr/bin/env python
import time
import stretch_body.robot
from stretch_body.hello_utils import ThreadServiceExit

robot=stretch_body.robot.Robot()
robot.startup()

x_move_base = 0
x_move_arm = 0
x_move_lift = 0
x_move_head_pan = 0
x_move_head_tilt = 0
x_move_wrist_yaw = 0
x_move_gripper = 0

def update_my_behavior(status):
    #Update the joint commands based on the status data
    pass 

try:
	while True:
       	#Get a snapshot of the robot status data
        status=robot.get_status()
        
        #Compute new position targets based on sensor data 
        update_my_behavior(status)
        
        #Queue new targets to devices
        robot.base.translate_by(x_move_base) #or robot.base.rotate_by()
        robot.arm.move_by(x_move_arm)
        robot.lift.move_by(x_move_lift)
        robot.head.move_by('head_pan', x_move_head_pan)
        robot.head.move_by('head_tilt', x_move_head_tilt)
        robot.end_of_arm.move_by('wrist_yaw', x_move_wrist_yaw)
        robot.end_of_arm.move_by('stretch_gripper', x_move_gripper)
        
        #Synchronized send of new position targets 
        robot.push_command()
        
        #Wait for next control cycle
        time.sleep(0.1)
except (KeyboardInterrupt, SystemExit, ThreadServiceExit)
    pass

robot.stop()

```

