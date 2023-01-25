# Tutorial: Contact Models

This tutorial introduces the Stretch Body contact detection system and explains how to configure it for your application.

## What is Guarded Contact?

Guarded contact is our term for the Stretch contact sensitive behaviors. The guarded contact behavior is simply:

1. Detect when the actuator effort exceeds a user-specified threshold during joint motion.
2. If the threshold is exceeded:
    1. Enable the default safety controller for the joint
    2. Remain in safety mode until a subsequent joint command is received

Practically this enables the arm, for example, to move out yet stop upon collision. Let's test this out with the following script:

```python
#!/usr/bin/env python

import time
import stretch_body.robot
robot=stretch_body.robot.Robot()
robot.startup()

#Move to full retraction
robot.arm.move_to(0.0)
robot.push_command()
robot.arm.wait_until_at_setpoint()

input('Arm will extend and respond to contact. Manually attempt to stop it. Hit enter when ready')
robot.arm.move_to(0.3)
robot.push_command()
robot.arm.wait_until_at_setpoint(timeout=5.0)

#Now turn off guarded contacts
input('Arm will retract but will not respond to contact. Manually attempt to stop it. Hit enter when ready')
robot.arm.motor.disable_guarded_mode()
robot.arm.move_to(0.0)
robot.push_command()
robot.arm.wait_until_at_setpoint(timeout=5.0)

robot.stop()
```

You should see that the arm stops on contact when it extends, however, it doesn't stop on contact when it then retracts. This is the guarded contact behavior in action.

## Specifying Guarded Contacts

The four stepper joints (base, arm, and lift) all support guarded contact settings when executing motion. This is evident in their `move_to` and `move_by` methods. For example, we see in the Arm's base class of [PrismaticJoint](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/prismatic_joint.py):

```python
  def move_by(self,x_m,v_m=None, a_m=None, stiffness=None, contact_thresh_pos_N=None,contact_thresh_neg_N=None, req_calibration=True,contact_thresh_pos=None,contact_thresh_neg=None)
```

In this method, you can optionally specify a contact threshold in the positive and negative direction with `contact_thresh_pos` and `contact_thresh_neg` respectively. 

!!! note
    These optional parameters will default to `None`, in which case the motion will adopt the default settings as defined by the robot's parameters.

!!! warning
    The parameters `contact_thresh_pos_N` and `contact_thresh_neg_N` are deprecated and no longer supported.

```bash
>>$ stretch_params.py | grep arm | grep contact
...                                              
stretch_configuration_params.yaml            param.arm.contact_models.effort_pct.contact_thresh_default    [-45.0, 45.0]    
...
```

## Contact Models

A contact model is simply a function that, given a user-specified contact threshold, computes the motor current at which the motor controller will trigger a guarded contact. The following contact models are currently implemented:

### The Effort-Pct Contact Model

[Effort-Pct](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/prismatic_joint.py) is the default contact model for Stretch 2. It simply scales the maximum range of motor currents into the range of [-100,100]. Thus, if you desire to have the robot arm extend but stop at 50% of its maximum current, you would write:

```python
robot.arm.move_by(0.1,contact_thresh_pos=50.0)
```

## Adjusting Contact Behaviors

The default factory settings for contact thresholds are tuned to allow Stretch to move throughout its workspace without triggering false-positive guarded contact events. These settings are the worst-case tuning as they account for the internal disturbances of the Stretch drive-train across its entire workspace. 

It is possible to obtain greater contact sensitivity in carefully selected portions of the arm and lift workspace. Users who wish to programmatically adjust contact behaviors can create a simple test script and experiment with different values. For example:

```python
#!/usr/bin/env python

import time
import stretch_body.robot
robot=stretch_body.robot.Robot()
robot.startup()

cpos = 30.0
cneg = -30.0

robot.arm.move_to(0.0, contact_thresh_pos=cpos, contact_thresh_neg=cneg)
robot.push_command()
robot.arm.wait_until_at_setpoint()

robot.arm.move_to(0.5,contact_thresh_pos=cpos, contact_thresh_neg=cneg)
robot.push_command()
robot.arm.wait_until_at_setpoint(timeout=5.0)

robot.stop()
```

## Guarded Contact with the Base

Guarded contacts are enabled by default for the arm and lift as they typically require safe and contact-sensitive motion. They are turned off on the base by default as varying surface terrain can produce undesired false-positive events.

That being said, guarded contacts can be enabled on the base. They may be useful as a simple bump detector such that the base will stop when it runs into a wall. 

```python
#!/usr/bin/env python

import time
import stretch_body.robot
robot=stretch_body.robot.Robot()
robot.startup()

robot.base.left_wheel.enable_guarded_mode()
robot.base.right_wheel.enable_guarded_mode()

robot.base.move_by(0.025)
robot.push_command()
robot.base.wait_until_at_setpoint()

robot.base.move_by(-0.025)
robot.push_command()
robot.base.wait_until_at_setpoint()

robot.stop()
```

## Advanced: Calibrating Contact Thresholds

The Stretch Factory package provides a tool to allow advanced users to recalibrate the default guarded contact thresholds. This tool can be useful if you've added additional payload to the arm and are experiencing false-positive guarded contact detections.

The tool sweeps the joint through its range of motion for `n-cycle` iterations. It computes the maximum contact forces in both directions, adds padding, `contact_thresh_calibration_margin`, to this value, and stores it to the robot's configuration YAML.

```bash
>>$ REx_calibrate_guarded_contact.py -h
For use with S T R E T C H (R) RESEARCH EDITION from Hello Robot Inc.
---------------------------------------------------------------------

usage: REx_calibrate_guarded_contact.py [-h] (--lift | --arm) [--ncycle NCYCLE]

Calibrate the default guarded contacts for a joint.

optional arguments:
  -h, --help       show this help message and exit
  --lift           Calibrate the lift joint
  --arm            Calibrate the arm joint  
  --ncycle NCYCLE  Number of sweeps to run [4]  
```

------
<div align="center"> All materials are Copyright 2022 by Hello Robot Inc. Hello Robot and Stretch are registered trademarks.</div>
