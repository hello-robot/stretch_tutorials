# Stretch Body API Reference

Stretch Body is the Python interface to working with the Stretch RE1. This page serves as a reference of the interfaces defined in the `stretch_body` library. 
See the [Stretch Body Tutorials](shttps://docs.hello-robot.com/0.2/stretch-tutorials/stretch_body/) for additional information on working with this library.

## The Robot Class

### Using the Robot class

The most common interface to Stretch is the [Robot](#stretch_body.robot.Robot) class. It is typically initialized as:

```python linenums='1'
import stretch_body.robot

r = stretch_body.robot.Robot()
if not r.startup():
    exit() # failed to start robot!

# home the joints to find zero, if necessary
if not r.is_calibrated():
    r.home()

# interact with the robot here
```

The [`startup()`](#stretch_body.robot.Robot.startup) and [`home()`](#stretch_body.robot.Robot.home) methods starts communication with and homes each of the robot's devices, respectively. Through the [Robot](#stretch_body.robot.Robot) class, users can interact with all other devices on the robot. For example, continuing the example above:

```python linenums='12'
# moving joints on the robot
r.arm.pretty_print()
r.lift.pretty_print()
r.base.pretty_print()
r.head.pretty_print()
r.end_of_arm.pretty_print()

# other devices on the robot
r.wacc.pretty_print()
r.pimu.pretty_print()

r.stop()
```

Each of these devices are defined in other modules within `stretch_body`. In the [following section](#the-device-classes), we'll look at the API of these classes. The [`stop()`](#stretch_body.robot.Robot.stop) method shuts down communication with the robot's devices. All of [Robot's](#stretch_body.robot.Robot) subroutines are documented below.

::: stretch_body.robot.Robot

## The Device Classes

The `stretch_body` library is modular in design. Each subcomponent of Stretch is defined in its own class and [the Robot class](#the-robot-class) provides an interface that ties all of these classes together. This modularity allows users to plug in new/modified subcomponents into the [Robot](#stretch_body.robot.Robot) interface by extending a device class.

It is possible to interface with a single subcomponent of Stretch by initializing its device class directly. In this section, we'll look at the API of seven device classes: the [arm](#stretch_body.arm.Arm), [lift](#stretch_body.lift.Lift), [base](#stretch_body.base.Base), [head](#stretch_body.head.Head), [end of arm](#stretch_body.end_of_arm.EndOfArm), [wacc](#stretch_body.wacc.Wacc), and [pimu](#stretch_body.pimu.Pimu) subcomponents of Stretch.

### Using the Arm class

![Top down Stretch arm blueprint](./images/arm_top.png#only-light)
![Top down Stretch arm blueprint](./images/arm_top.png#only-dark)

The interface to Stretch's telescoping arm is the [Arm](#stretch_body.arm.Arm) class. It is typically initialized as:

```python linenums='1'
import stretch_body.arm

a = stretch_body.arm.Arm()
a.motor.disable_sync_mode()
if not a.startup():
    exit() # failed to start arm!

a.home()

# interact with the arm here
```

Since both [Arm](#stretch_body.arm.Arm) and [Robot](#stretch_body.robot.Robot) subclass [Device](#stretch_body.device.Device), the same [`startup()`](#stretch_body.arm.Arm.startup) and [`stop()`](#stretch_body.arm.Arm.stop) methods are available here, as well as other [Device](#stretch_body.device.Device) methods such as [`home()`](#stretch_body.arm.Arm.home). Using the [Arm](#stretch_body.arm.Arm) class, we can read the arm's current state and send commands to the joint. For example, continuing the example above:

```python linenums='11'
starting_position = a.status['pos']

# move out by 10cm
a.move_to(starting_position + 0.1)
a.push_command()
a.motor.wait_until_at_setpoint()

# move back to starting position quickly
a.move_to(starting_position, v_m=0.2, a_m=0.25)
a.push_command()
a.motor.wait_until_at_setpoint()

a.move_by(0.1) # move out by 10cm
a.push_command()
a.motor.wait_until_at_setpoint()
```

The [`move_to()`](#stretch_body.arm.Arm.move_to) and [`move_by()`](#stretch_body.arm.Arm.move_by) methods queue absolute and relative commands to the arm respectively, while the nonblocking [`push_command()`](#stretch_body.arm.Arm.push_command) method pushes the queued command to the hardware for execution. Arm's attribute `motor`, an instance of the [Stepper](#stretch_body.stepper.Stepper) class, has [`wait_until_at_setpoint()`](#stretch_body.stepper.Stepper.wait_until_at_setpoint) which blocks program execution until the joint reaches the commanded goal. With [P1 or greater firmware](https://github.com/hello-robot/stretch_firmware/blob/master/tutorials/docs/updating_firmware.md) installed, it is also possible to queue a waypoint trajectory for the arm to follow:

```python linenums='26'
starting_position = a.status['pos']

# queue a trajectory consisting of four waypoints
a.trajectory.add(t_s=0, x_m=starting_position)
a.trajectory.add(t_s=3, x_m=0.15)
a.trajectory.add(t_s=6, x_m=0.1)
a.trajectory.add(t_s=9, x_m=0.2)

# trigger trajectory execution
a.follow_trajectory()
import time; time.sleep(9)
```

Arm's attribute `trajectory`, an instance of the [PrismaticTrajectory](#stretch_body.trajectories.PrismaticTrajectory) class, has [`add()`](#stretch_body.trajectories.PrismaticTrajectory.add) which adds a single waypoint in a linear sliding trajectory. For a well formed `trajectory` (see [`is_valid()`](#stretch_body.trajectories.Spline.is_valid)), the [`follow_trajectory()`](#stretch_body.arm.Arm.follow_trajectory) method kicks off trajectory following for the telescoping arm. It is also possible to dynamically restrict the arm joint's range:

```python linenums='37'
range_upper_limit = 0.3 # meters

# set soft limits on arm's range
a.set_soft_motion_limit_min(0)
a.set_soft_motion_limit_max(range_upper_limit)
a.push_command()

# command the arm outside the valid range
a.move_to(0.4)
a.push_command()
a.motor.wait_until_at_setpoint()
print(a.status['pos']) # we should expect to see ~0.3

a.stop()
```

The [`set_soft_motion_limit_min/max()`](#stretch_body.arm.Arm.set_soft_motion_limit_min) methods form the basis of an experimental [self-collision avoidance](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/robot_collision.py#L47) system built into Stretch Body. All of [Arm's](#stretch_body.arm.Arm) subroutines are documented below.

::: stretch_body.arm.Arm

### Using the Lift class

![Stretch lift blueprint](./images/lift_detail_rs.png#only-light)
![Stretch lift blueprint](./images/lift_detail_rs.png#only-dark)

The interface to Stretch's lift is the [Lift](#stretch_body.lift.Lift) class. It is typically initialized as:

```python linenums='1'
import stretch_body.lift

l = stretch_body.lift.Lift()
l.motor.disable_sync_mode()
if not l.startup():
    exit() # failed to start lift!

l.home()

# interact with the lift here
```

The [`startup()`](#stretch_body.lift.Lift.startup) and [`home()`](#stretch_body.lift.Lift.home) methods are extended from the [Device](#stretch_body.device.Device) class. Reading the lift's current state and sending commands to the joint occurs similarly to the [Arm](#stretch_body.arm.Arm):

```python linenums='11'
starting_position = l.status['pos']

# move up by 10cm
l.move_to(starting_position + 0.1)
l.push_command()
l.motor.wait_until_at_setpoint()
```

[Lift's](#stretch_body.lift.Lift) attribute `status` is a dictionary of the joint's current status. This state information is updated in the background in real time by default (disable by initializing as [`startup(threading=False)`](#stretch_body.lift.Lift.startup)). Use the [`pretty_print()`](#stretch_body.lift.Lift.pretty_print) method to print out this state info in a human interpretable format. Setting up waypoint trajectories for the lift is also similar to the [Arm](#stretch_body.arm.Arm):

```python linenums='17'
starting_position = l.status['pos']

# queue a trajectory consisting of three waypoints
l.trajectory.add(t_s=0, x_m=starting_position, v_m=0.0)
l.trajectory.add(t_s=3, x_m=0.5,               v_m=0.0)
l.trajectory.add(t_s=6, x_m=0.6,               v_m=0.0)

# trigger trajectory execution
l.follow_trajectory()
import time; time.sleep(6)
```

[Lift's](#stretch_body.lift.Lift) attribute `trajectory` is also an instance of the [PrismaticTrajectory](#stretch_body.trajectories.PrismaticTrajectory) class, and by providing the instantaneous velocity argument `v_m` to the [`add()`](#stretch_body.trajectories.PrismaticTrajectory.add) method, a cubic spline has been loaded into the joint's `trajectory`. The call to [`follow_trajectory()`](#stretch_body.lift.Lift.follow_trajectory) begins hardware tracking of the spline. Finally, setting soft motion limits for the lift's range happens using:

```python linenums='27'
# cut out 0.2m from the top and bottom of the lift's range
l.set_soft_motion_limit_min(0.2)
l.set_soft_motion_limit_max(0.8)
l.push_command()

l.stop()
```

The [`set_soft_motion_limit_min/max()`](#stretch_body.lift.Lift.set_soft_motion_limit_min) methods perform clipping of the joint's range at the firmware level (can persist across reboots). All of [Lift's](#stretch_body.lift.Lift) subroutines are documented below.

::: stretch_body.lift.Lift

### Using the Base class

![Stretch mobile base diagram](./images/hw_image_1.png#only-light)
![Stretch mobile base diagram](./images/hw_image_1.png#only-dark)

|         | Item           | Notes                                                     |
| ------- | -------------- | --------------------------------------------------------- |
| A       | Drive wheels   | 4 inch diameter, urethane rubber shore 60A                |
| B       | Cliff sensors  | Sharp GP2Y0A51SK0F, Analog, range 2-15 cm                 |
| C       | Mecanum wheel  | Diameter 50mm                                             |

The interface to Stretch's mobile base is the [Base](#stretch_body.base.Base) class. It is typically initialized as:

```python linenums='1'
import stretch_body.base

b = stretch_body.base.Base()
b.left_wheel.disable_sync_mode()
b.right_wheel.disable_sync_mode()
if not b.startup():
    exit() # failed to start base!

# interact with the base here
```

Stretch's mobile base is a differential drive configuration. The left and right wheels are accessible through [Base's](#stretch_body.base.Base) `left_wheel` and  `right_wheel` attributes, both of which are instances of the [Stepper](#stretch_body.stepper.Stepper) class. The [`startup()`](#stretch_body.base.Base.startup) method is extended from the [Device](#stretch_body.device.Device) class. Since the mobile base is unconstrained, there is no homing method. We can read the base's current state and send commands using:

```python linenums='10'
b.pretty_print()

# translate forward by 10cm
b.translate_by(0.1)
b.push_command()
b.left_wheel.wait_until_at_setpoint()

# rotate counter-clockwise by 90 degrees
b.rotate_by(1.57)
b.push_command()
b.left_wheel.wait_until_at_setpoint()
```

The [`pretty_print()`](#stretch_body.base.Base.pretty_print) method prints out mobile base state info in a human interpretable format. The [`translate_by()`](#stretch_body.base.Base.translate_by) and [`rotate_by()`](#stretch_body.base.Base.rotate_by) methods send relative commands similar to the way [`move_by()`](#stretch_body.lift.Lift.move_by) behaves for the single degree of freedom joints. The mobile base also supports velocity control:

```python linenums='21'
# command the base to translate forward at 5cm / second
b.set_translate_velocity(0.05)
b.push_command()
import time; time.sleep(1)

# command the base to rotate counter-clockwise at 0.1rad / second
b.set_rotational_velocity(0.1)
b.push_command()
time.sleep(1)

# command the base with translational and rotational velocities
b.set_velocity(0.05, 0.1)
b.push_command()
time.sleep(1)

# stop base motion
b.enable_freewheel_mode()
b.push_command()
```

The [`set_translate_velocity()`](#stretch_body.base.Base.set_translate_velocity)/[`set_rotational_velocity()`](#stretch_body.base.Base.set_rotational_velocity) give velocity control over the translational/rotational components of the mobile base independently. The [`set_velocity()`](#stretch_body.base.Base.set_velocity) method gives control over both of these components simultaneously. To halt motion, you can command zero velocities or command the base into freewheel mode using [`enable_freewheel_mode()`](#stretch_body.base.Base.enable_freewheel_mode). The mobile base also supports waypoint trajectory following, but the waypoints are part of the SE2 group (a.k.a. each of the base's desired waypoints is defined as a (x, y) point and a theta orientation):

```python linenums='39'
# reset odometry calculation
b.first_step = True
b.pull_status()

# queue a trajectory consisting of three waypoints
b.trajectory.add(time=0, x=0.0, y=0.0, theta=0.0)
b.trajectory.add(time=3, x=0.1, y=0.0, theta=0.0)
b.trajectory.add(time=6, x=0.0, y=0.0, theta=0.0)

# trigger trajectory execution
b.follow_trajectory()
import time; time.sleep(6)
print(b.status['x'], b.status['y'], b.status['theta']) # we should expect to see around (0.0, 0.0, 0.0 or 6.28)

b.stop()
```

!!! warning
    The [Base's](#stretch_body.base.Base) waypoint trajectory following has no notion of obstacles in the environment. It will blindly follow the commanded waypoints. For obstacle avoidance, perception and a path planner should be employed.

[Base's](#stretch_body.base.Base) attribute `trajectory` is an instance of the [DiffDriveTrajectory](#stretch_body.trajectories.DiffDriveTrajectory) class. The call to [`follow_trajectory()`](#stretch_body.base.Base.follow_trajectory) begins hardware tracking of the spline. All of [Base's](#stretch_body.base.Base) subroutines are documented below.

::: stretch_body.base.Base

### Using the Head class

![Stretch head blueprint](./images/tilt_detail_rs.png#only-light)
![Stretch head blueprint](./images/tilt_detail_rs.png#only-dark)

The interface to Stretch's head is the [Head](#stretch_body.head.Head) class. Stretch's head contains a Intel Realsense D435i depth camera, so the pan/tilt joints in the head allows Stretch to swivel and capture depth imagery of its surrounding. The head is typically initialized as:

```python linenums='1'
import stretch_body.head

h = stretch_body.head.Head()
if not h.startup():
    exit() # failed to start head!

h.home()

# interact with the head here
```

[Head](#stretch_body.head.Head) is a subclass of [DynamixelXChain](#stretch_body.dynamixel_X_chain.DynamixelXChain), which in turn subclasses the [Device](#stretch_body.device.Device) class. Therefore, some of [Head's](#stretch_body.head.Head) methods, such as [`startup()`](#stretch_body.head.Head.startup) and [`home()`](#stretch_body.head.Head.home) methods are extended from the [Device](#stretch_body.device.Device) class, while other come from the [DynamixelXChain](#stretch_body.dynamixel_X_chain.DynamixelXChain) class. Reading the head's current state and sending commands to its revolute joints (head pan and tilt) happens using:

```python linenums='10'
starting_position = h.status['head_pan']['pos']

# look right by 90 degrees
h.move_to('head_pan', starting_position + 1.57)
h.get_joint('head_pan').wait_until_at_setpoint()

# tilt up by 30 degrees
h.move_by('head_tilt', -1.57 / 3)
h.get_joint('head_tilt').wait_until_at_setpoint()

# look down towards the wheels
h.pose('wheels')
import time; time.sleep(3)

# look ahead
h.pose('ahead')
time.sleep(3)
```

[Head's](#stretch_body.head.Head) attribute `status` is a dictionary of dictionaries, where each subdictionary is the status of one of the head's joints. This state information is updated in the background in real time by default (disable by initializing as [`startup(threading=False)`](#stretch_body.head.Head.startup)). Use the [`pretty_print()`](#stretch_body.head.Head.pretty_print) method to print out this state info in a human interpretable format. Commanding the head's revolute joints is done through the [`move_to()`](#stretch_body.head.Head.move_to) and [`move_by()`](#stretch_body.head.Head.move_by) methods. Notice that unlike the previous joints, no push command call is required here. These joints are Dynamixel servos, which behave differently than the Hello Robot steppers. Their commands are not queued; they're executed as soon as they're received. [Head's](#stretch_body.head.Head) two joints, the 'head_pan' and 'head_tilt', are instances of the [DynamixelHelloXL430](#stretch_body.dynamixel_hello_XL430.DynamixelHelloXL430) class, and are retreiveable using the [`get_joint()`](#stretch_body.head.Head.get_joint) method. They have the [`wait_until_at_setpoint()`](#stretch_body.dynamixel_hello_XL430.DynamixelHelloXL430.wait_until_at_setpoint) method, which blocks program execution until the joint reaches the commanded goal. The [`pose()`](#stretch_body.head.Head.pose) method makes it easy to command the head to common head poses (e.g. looking 'ahead', at the end of arm 'tool', obstacles in front of the 'wheels', or 'up'). The head supports waypoint trajectories as well:

```python linenums='27'
# queue a trajectory consisting of three waypoints
h.get_joint('head_tilt').trajectory.add(t_s=0, x_r=0.0)
h.get_joint('head_tilt').trajectory.add(t_s=3, x_r=-1.0)
h.get_joint('head_tilt').trajectory.add(t_s=6, x_r=0.0)
h.get_joint('head_pan').trajectory.add(t_s=0, x_r=0.1)
h.get_joint('head_pan').trajectory.add(t_s=3, x_r=-0.9)
h.get_joint('head_pan').trajectory.add(t_s=6, x_r=0.1)

# trigger trajectory execution
h.follow_trajectory()
import time; time.sleep(6)
```

The head pan/tilt [DynamixelHelloXL430](#stretch_body.dynamixel_hello_XL430.DynamixelHelloXL430) instances have an attribute `trajectory`, which is an instance of the [RevoluteTrajectory](#stretch_body.trajectories.RevoluteTrajectory) class. The call to [`follow_trajectory()`](#stretch_body.dynamixel_X_chain.DynamixelXChain.follow_trajectory) begins software tracking of the spline. Finally, setting soft motion limits for the head's pan/tilt range happens using:

```python linenums='38'
# clip the head_pan's range
h.get_joint('head_pan').set_soft_motion_limit_min(-1.0)
h.get_joint('head_pan').set_soft_motion_limit_max(1.0)

# clip the head_tilt's range
h.get_joint('head_tilt').set_soft_motion_limit_min(-1.0)
h.get_joint('head_tilt').set_soft_motion_limit_max(0.1)

h.stop()
```

The [`set_soft_motion_limit_min/max()`](#stretch_body.dynamixel_X_chain.DynamixelXChain.set_soft_motion_limit_min) methods perform clipping of the joint's range at the software level (cannot persist across reboots). All of [Head's](#stretch_body.head.Head) subroutines are documented below.

::: stretch_body.head.Head

### Using the EndOfArm class

The interface to Stretch's end of arm is the [EndOfArm](#stretch_body.end_of_arm.EndOfArm) class. It is typically initialized as:

```python linenums='1'
import stretch_body.end_of_arm

e = stretch_body.end_of_arm.EndOfArm()
if not e.startup(threaded=True):
    exit() # failed to start end of arm!

# interact with the end of arm here

e.stop()
```

[EndOfArm's](#stretch_body.end_of_arm.EndOfArm) subroutines are documented below.

::: stretch_body.end_of_arm.EndOfArm

### Using the Wacc class

The interface to Stretch's wrist board is the [Wacc](#stretch_body.wacc.Wacc) (wrist + accelerometer) class. This board provides an Arduino and accelerometer sensor that is accessible from the [Wacc](#stretch_body.wacc.Wacc) class. It is typically initialized as:

```python linenums='1'
import stretch_body.wacc

w = stretch_body.wacc.Wacc()
if not w.startup(threaded=True):
    exit() # failed to start wacc!

# interact with the wacc here

w.stop()
```

[Wacc's](#stretch_body.wacc.Wacc) subroutines are documented below.

::: stretch_body.wacc.Wacc

### Using the Pimu class

The interface to Stretch's power board is the [Pimu](#stretch_body.pimu.Pimu) (power + IMU) class. This board provides an 9 DOF IMUthat is accessible from the [Pimu](#stretch_body.pimu.Pimu) class. It is typically initialized as:

```python linenums='1'
import stretch_body.pimu

p = stretch_body.pimu.Pimu()
if not p.startup(threaded=True):
    exit() # failed to start pimu!

# interact with the pimu here

p.stop()
```

[Pimu's](#stretch_body.pimu.Pimu) subroutines are documented below.

::: stretch_body.pimu.Pimu

------
<div align="center"> All materials are Copyright 2022 by Hello Robot Inc. Hello Robot and Stretch are registered trademarks.</div>