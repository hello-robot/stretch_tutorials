# Tutorial: Robot Motion

As we've seen in previous tutorials, commanding robot motion can be simple and straight forward. For example, incremental motion of the arm can be commanded by:

```python linenums="1"
import stretch_body.robot
robot=stretch_body.robot.Robot()
robot.startup()

robot.arm.move_by(0.1)
robot.push_command()
time.sleep(2.0)
	
robot.stop()
```

Or, absolute motion can be commanded by:

```python linenums="1"
import stretch_body.robot
robot=stretch_body.robot.Robot()
robot.startup()

robot.arm.move_to(0.1)
robot.push_command()
time.sleep(2.0)
	
robot.stop()
```

## Waiting on Motion

In the above examples we execute a `time.sleep()` after `robot.push_command()`. This allows the joint time to complete its motion. Instead we can use the `wait_until_at_setpoint()` method that polls the joint position versus the target position. We can also interrupt a motion by sending a new motion command at anytime.  For example, try the following script:



```python linenums="1"
import stretch_body.robot
robot=stretch_body.robot.Robot()
robot.startup()

#Move to full retraction
robot.arm.move_to(0.0)
robot.push_command()
robot.arm.wait_until_at_setpoint()

#Move to full extension
robot.arm.move_to(0.5)
robot.push_command()

#Interrupt motion midway and retract again
time.sleep(2.0)
robot.arm.move_to(0.0)
robot.arm.wait_until_at_setpoint()
	
robot.stop()
```

You will see the arm fully retract, begin to extend, and then fully retract again.

## Motion Profiles

All joints support [trapezoidal based motion](https://www.motioncontroltips.com/what-is-a-motion-profile/) generation. Other types of controllers are available (splined trajectory, PID, velocity, etc) but they are not covered here . The trapezoidal motion controllers require three values:

* x: target position of joint
* v: maximum velocity of motion
* a: acceleration of motion

We provide 'default' settings for the velocity and acceleration settings, as well as 'fast', and 'slow' settings. These values have been tuned to be appropriate for safe motion of the robot. These values can queried  using the `stretch_params.py` tool:

```bash
>>$stretch_params.py | grep arm | grep motion | grep default
stretch_body.robot_params.nominal_params      param.arm.motion.fast.accel_m     0.14                           
stretch_body.robot_params.nominal_params      param.arm.motion.fast.vel_m       0.14 

```

We see that the arm motion in 'default' mode will move with a velocity of 0.14 m/s and an acceleration of 0.14 m/s^2. 

The `move_by` and `move_to` commands support optional motion profile parameters. For example, for a fast motion: 

```python
v = robot.arm.params['motion']['fast']['vel_m']
a = robot.arm.params['motion']['fast']['accel_m']

robot.arm.move_by(x_m=0.1, v_m=v, a_m=a)
robot.push_command()
```

The motion will use the 'default' motion profile settings if no values are specified.

## Motion Limits

All joints obey motion limits which are specified in the robot parameters. 

```bash
>>$ stretch_params.py | grep arm | grep range_m
stretch_user_params.yaml          param.arm.range_m     [0.0, 0.515] 
```

These are the mechanical limits of the joint. These limits have been set at the factory to prevent damage to the hardware. It is not recommended to set them to be greater than the factory specified values. However, they can be further limited if desired by setting soft motion limits:

```python
import stretch_body.robot
robot=stretch_body.robot.Robot()
robot.startup()

robot.arm.set_soft_motion_limit_min(0.2)
robot.arm.set_soft_motion_limit_max(0.4)

#Will move to position 0.2
robot.arm.move_to(0.0)
robot.push_command()
robot.arm.wait_until_at_setpoint()

#Will move to position 0.4
robot.arm.move_to(0.5)
robot.push_command()
robot.arm.wait_until_at_setpoint()
	
robot.stop()
```



## Controlling Dynamixel Motion

The above examples have focused on the motion of the arm. Like the lift and the base, the arm utilizes Hello Robot's custom stepper motor controller. Control of the Dynamixels of the head and the end-of-arm is very similar to that of the arm (though not identical).

As we see here, the `robot.push_command` call is not required as the motion begins instantaneously and is not queued. In addition, the Dynamixel servos are managed as a chain of devices -- so we must pass in the joint name along with the command.



```python
import stretch_body.robot
from stretch_body.hello_utils import deg_to_rad

robot=stretch_body.robot.Robot()
robot.startup()

robot.head.move_to('head_pan',0)
robot.head.move_to('head_tilt',0)

time.sleep(3.0)

robot.head.move_to('head_pan',deg_to_rad(90.0))
robot.head.move_to('head_tilt',deg_to_rad(45.0))

time.sleep(3.0)
	
robot.stop()
```

Similarly to the stepper joints, the Dynamixel joints accept motion profile and motion limit commands. For example, here we restrict the head pan range of motion while executing both a fast and slow move:

```python
import stretch_body.robot
robot=stretch_body.robot.Robot()
robot.startup()

#Limit range of motion of head_pan
robot.head.get_motor('head_pan').set_soft_motion_limit_min(deg_to_rad(-45.0))
robot.head.get_motor('head_pan').set_soft_motion_limit_max(deg_to_rad(45.0))

#Do a fast motion
v = robot.params['head_pan']['motion']['fast']['vel']
a = robot.params['head_pan']['motion']['fast']['accel']
robot.head.move_to('head_pan',deg_to_rad(-90.0),v_r=v, a_r=a)

time.sleep(3.0)

#Do a slow motion
v = robot.params['head_pan']['motion']['slow']['vel']
a = robot.params['head_pan']['motion']['slow']['accel']
robot.head.move_to('head_pan',deg_to_rad(90.0),v_r=v, a_r=a)

time.sleep(3.0)
	
robot.stop()
```



## Base Velocity Control

The Base also supports a velocity control mode which can be useful for use with navigation planner. The Base controllers will automatically switch between velocity and position based control.  For example:

```python
robot.base.translate_by(x_m=0.5)
robot.push_command()
time.sleep(4.0) #wait

robot.base.set_rotational_velocity(v_r=0.1) #switch to velocity controller
robot.push_command()
time.sleep(4.0) #wait

robot.base.set_rotational_velocity(v_r=0.0) #stop motion
robot.push_command()
```

As shown, care should be taken to set commanded velocities to zero on exit to avoid runaway.

## Advanced Topics

### Stepper Control Modes

Most users will control robot motion using the `move_to` and `move_by` commands as described above. There are numerous other low-level controller modes available. While these are a topic for advanced users, it is worth noting that each joint has a default safety mode and a default position control mode.  These are:

| Joint           | Default Safety Mode         | Default Position Control Mode |
| --------------- | --------------------------- | ----------------------------- |
| left_wheel      | Freewheel                   | Trapezoidal position control  |
| right_wheel     | Freewheel                   | Trapezoidal position control  |
| lift            | Gravity compensated 'float' | Trapezoidal position control  |
| arm             | Freewheel                   | Trapezoidal position control  |
| head_pan        | Torque disabled             | Trapezoidal position control  |
| head_tilt       | Torque disabled             | Trapezoidal position control  |
| wrist_yaw       | Torque disabled             | Trapezoidal position control  |
| stretch_gripper | Torque disabled             | Trapezoidal position control  |

Each joint remains in Safety Mode when no program is running. When the `<device>.startup()` function is called, the joint controller transitions from Safety Mode to its Default Position Control Mode. It is then placed back in Safety Mode when `<device>.stop()` is called.

### Motion Runstop

Runstop activation will cause  the Base, Arm, and Lift to switch to Safety Mode and for subsequent motion commands will be ignored. The motion commands will resume smoothly  when the runstop is deactivated.  This is usually done via the runstop button. However, it can also be done via the Pimu interface. For example:

```python
import stretch_body.robot
robot=stretch_body.robot.Robot()
robot.startup()

#Move to full retraction
robot.arm.move_to(0.0)
robot.push_command()
robot.arm.wait_until_at_setpoint()

#Move to full extension
robot.arm.move_to(0.5)
robot.push_command()

#Runstop motion midway through the motion
input('Hit enter to runstop motion')
robot.pimu.runstop_event_trigger()
robot.push_command()

input('Hit enter to restart motion')	
robot.pimu.runstop_event_reset()
robot.push_command()

robot.arm.move_to(0.5)
robot.push_command()
robot.arm.wait_until_at_setpoint()

robot.stop()
```



### Guarded Motion

The Arm, Lift, and Base support a guarded motion function.  It will automatically transition the actuator from Control mode to Safety mode when the exerted motor torque exceeds a threshold. 

This functionality is most useful for the Lift and the Arm. It allows these joints to safely stop upon contact. It can be used to:

* Safely stop when contacting an actuator hardstop
* Safely stop when making unexpected contact with the environment or a person
* Make a guarded motion where the robot reaches to a surface and then stops


For more information on guarded motion, see the [Contact Models Tutorial](./tutorial_contact_models.md)

### Synchronized Motion

The Arm, Lift, and Base actuators have a hardware synchronization mechanism. This allows for stepper controller commands to be time synchronized across joints.  This behavior can be disabled via the user YAML. By default the settings are:

```bash
>>$ stretch_params.py | grep enable_sync_mode
stretch_body.robot_params.nominal_params   param.hello-motor-arm.gains.enable_sync_mode         1                             
stretch_body.robot_params.nominal_params   param.hello-motor-left-wheel.gains.enable_sync_mode  1                             
stretch_body.robot_params.nominal_params   param.hello-motor-lift.gains.enable_sync_mode        1                             
stretch_body.robot_params.nominal_params   param.hello-motor-right-wheel.gains.enable_sync_mode 1  
```

### Motion Status

It can be useful to poll the status of a joint during motion in order to modify the robot behavior, etc. The useful status values include:

```python
robot.arm.status['pos']						#Joint position
robot.arm.status['vel']						#Joint velocity
robot.arm.motor.status['effort_pct']		#Joint effort (-100 to 100) (derived from motor current)	
robot.arm.motor.status['near_pos_setpoint']	#Is sensed position near commanded position
robot.arm.motor.status['near_vel_setpoint'] #Is sensed velocity near commanded velocity
robot.arm.motor.status['is_moving']			#Is the joint in motion
robot.arm.motor.status['in_guarded_event']	#Has a guarded event occured
robot.arm.motor.status['in_safety_event']	#Has a safety event occured
```

### Update Rates

The following update rates apply to Stretch:

| Item                                            | Rate | Notes                                                        |
| ----------------------------------------------- | ---- | ------------------------------------------------------------ |
| Status data for Arm, Lift, Base, Wacc, and Pimu | 25Hz | Polled automatically by Robot thread                         |
| Status data for End of Arm and Head servos      | 15Hz | Polled automatically by Robot thread                         |
| Command data for Arm, Lift, Base, Wacc, Pimu    | N/A  | Commands are queued and executed upon calling robot.push_command( ) |
| Command data for End of Arm and Head servos     | N/A  | Commands execute immediately                                 |

Motion commands are non-blocking and it is the responsibility of the user code to poll the Robot Status to determine when and if a motion target has been achieved.

The Stretch_Body interface is not designed to support high bandwidth control applications. The natural dynamics of the robot actuators do not support high bandwidth control, and the USB based interface limits high rate communication.

In practice, a Python based control loop that calls push_command( ) at 1Hz to 10Hz is sufficiently matched to the robot natural dynamics. 

------
<div align="center"> All materials are Copyright 2022 by Hello Robot Inc. Hello Robot and Stretch are registered trademarks. The Stretch RE1 and RE2 robots are covered by U.S. Patent 11,230,000 and other patents pending.</div>

