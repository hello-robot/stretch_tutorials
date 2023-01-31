# Tutorial: Collision Avoidance

In this tutorial, we will discuss the simple collision avoidance system that runs as a part of Stretch Body.

## Overview

Stretch Body includes a system to prevent inadvertent self-collisions.  It will dynamically limit the range of motion of each joint to prevent self-collisions. 

!!! warning
    Self collisions are still possible while using the collision-avoidance system. The factory default collision models are coarse and not necessarily complete.

This system is turned off by default starting with Stretch 2. It may be turned off by default on many RE1 systems. First check if the collision detection system is turned on:

```{.bash .shell-prompt}
stretch_params.py | grep use_collision_manager
```

Output:
```{.bash .no-copy}
stretch_body.robot_params.nominal_params       param.robot.use_collision_manager         1
```

If it is turned off you can enable it by adding the following to your stretch_user_yaml.py:

```{.bash .no-copy}
robot:
  use_collision_manager: 1
```

## Common Self Collisions

Fortunately, the simple kinematics of Stretch make self-collisions fairly uncommon and simple to predict. The primary places where self-collisions may occur are

* The lift lowering the wrist or tool into the base
* The arm retracting the wrist or tool into the base
* The head_pan at `pos==0` and head_tilt at `pos=-90 deg` and the lift raising the arm into the camera (minor collision)
* The Dex Wrist (if installed) colliding with itself
* The Dex Wrist (if installed) colliding with the base

## Joint Limits

The collision avoidance system works by dynamically modifying the acceptable range of motion for each joint. By default, a joint's range is set to the physical hard stop limits. For example, the lift has a mechanical throw of 1.1m:

```{.bash .shell-prompt}
stretch_params.py | grep range | grep lift
```

Output:
```{.bash .no-copy}
stretch_body.robot_params.factory_params       param.lift.range_m      [0.0, 1.1]                
```

A reduced range of motion can be set at run-time by setting the Soft Motion Limit. For example, to limit the lift range of motion to 0.3 meters off the base:

```python
import stretch_body.robot as robot
r=robot.Robot()
r.startup()
r.lift.set_soft_motion_limit_min(0.3)
```

We see in the [API](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/lift.py), the value of `None` is used to designate no soft limit.

It is possible that when setting the Soft Motion Limit the joint's current position is outside of the specified range. In this case, the joint will move to the nearest soft limit to comply with the limits. This can be demonstrated by:

```python
import stretch_body.robot as robot
import time

r=robot.Robot()
r.startup()

#Move to 0.2
r.lift.move_to(0.2)
r.push_command()
time.sleep(5.0) 

#Will move to 0.3
r.lift.set_soft_motion_limit_min(0.3)

```

## Collision Models

The [RobotCollision](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/robot_collision.py) class manages a set of [RobotCollisionModels](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/robot_collision.py). Each [RobotCollisionModel](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/robot_collision.py) computes the soft limits for a subset of joints based on a simple geometric model. This geometric model captures the enumerated set of potential collisions listed above.

We can see which collision models will execute when `use_collision_manager` is set to 1:

```{.bash .shell-prompt}
stretch_params.py | grep collision | grep enabled
```

Output:
```{.bash .no-copy}
stretch_body.robot_params.nominal_params    param.collision_arm_camera.enabled           1                             
stretch_body.robot_params.nominal_params    param.collision_stretch_gripper.enabled      1  
```

We see two models. One that protects the camera from the arm, and one that protects the base from the gripper. Each model is registered with the [RobotCollision](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/robot_collision.py) instance as a loadable plug-in. The [Robot](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/robot.py) class calls the `RobotCollision.step` method periodically at approximately 10hz. 

`RobotCollision.step` computes the 'AND' of the limits specified across each Collision Model such that the most restrictive joint limits are set for each joint using the `set_soft_motion_limit_min` and `set_soft_motion_limt_max` methods. 

## Default Collision Models

The default collision models for Stretch Body are found in [robot_collision_models.py](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/robot_collision_models.py). As of this writing, the provided models are:

* [CollisionArmCamera](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/robot_collision_models.py#L8): Avoid collision of the head camera with the arm
* [CollisionStretchGripper](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/robot_collision_models.py#L75): Avoid collision of the wrist-yaw and gripper with the base and ground

!!! warning
    The provided collision models are coarse and are provided to avoid common potentially harmful collisions only. Using these models it is still possible to collide the robot with itself in some cases.

!!! info
    Additional collision models are provided for the DexWrist

### Working with Models

The collision models to be used by Stretch Body are defined with the `robot_collision` parameter. For example, we see in `robot_params.py`  that the CollisionArmCamera is loaded by default:

```python
"robot_collision": {'models': ['collision_arm_camera']},
```

We also see that model `collision_arm_camera` is defined as:

```python
  "collision_arm_camera": {
        'enabled': 1,
        'py_class_name': 'CollisionArmCamera',
        'py_module_name': 'stretch_body.robot_collision_models'
    }
```

This instructs RobotCollision to construct a model of type `CollisionArmCamera` and enable it by default. One can disable this model by default by specifying the following `stretch_re1_user_params.yaml`:

```yaml
collision_arm_camera:
  enabled: 0
```

The  entire collision avoidance system can be disabled in `stretch_re1_user_params.yaml` by:

```yaml
robot:
  use_collision_manager: 0
```

A specific collision model can be enabled or disabled during runtime by:

```python
import stretch_body.robot
r=stretch_body.robot.Robot()
r.startup() 
... #Do some work
r.collision.disable_model('collsion_arm_camera')
... #Do some work
r.collision.enable_model('collsion_arm_camera')
```

Finally, if we want to also use the CollisionStretchGripper model, we can add to `stretch_re1_user_params.py`:

```yaml
robot_collision:
  models:
  - collision_arm_camera
  - collision_stretch_gripper
```

### Creating Custom Collision Models

The `step` method of a RobotCollisionModel returns the desired joint limits given that model. For example, the base class is simply:

```python
    class RobotCollisionModel(Device):
    	def step(self, status):
        	return {'head_pan': [None, None],'head_tilt': [None, None], 
                    'lift': [None, None],'arm': [None, None],'wrist_yaw': [None, None]}
```

where the value of `None` specifies that no limit is specified and the full range of motion for the joint is acceptable.  

We could define a new collision model that simply limits the lift range of motion to 1 meter by:

```python
    class MyCollisionModel(Device):
    	def step(self, status):
        	return {'head_pan': [None, None],'head_tilt': [None, None], 
                    'lift': [None, 1.0],'arm': [None, None],'wrist_yaw': [None, None]}
```

It is straightforward to create a custom collision model. As an example, we will create a model that avoids collision of the arm with a tabletop by

* Preventing the lift from descending below the table top when the arm is extended 
* Allowing the lift to descend below the tabletop so long as the arm retracted

This assumes the arm is initially above the tabletop. To start, in a file `collision_arm_table.py` we add:

```python
from stretch_body.robot_collision import *
from stretch_body.hello_utils import *

class CollisionArmTable(RobotCollisionModel):
    def __init__(self, collision_manager):
        RobotCollisionModel.__init__(self, collision_manager, 'collision_arm_table')

    def step(self, status):
        limits = {'lift': [None, None],'arm': [None, None]}
        table_height = 0.5 #m
        arm_safe_retract = 0.1 #m
        safety_margin=.05#m

        x_arm = status['arm']['pos']
        x_lift = status['lift']['pos']

        #Force arm to stay retracted if below table
        if x_lift<table_height:
            limits['arm'] = [None, arm_safe_retract-safety_margin]
        else:
            limits['arm'] = [None, None]

        #Force lift to stay above table unless arm is retracted
        if x_arm<arm_safe_retract:
            limits['lift'] =[None,None]
        else:
            limits['lift']=[table_height+safety_margin,None]
        return limits
```
In this example, we include the `safety_margin` as a way to introduce some hysteresis around state changes to avoid toggling between the soft limits.

The following command should be run to add the working directory to the PYTHONPATH env. This can also be added to our `.bashrc` to permanently edit the path:

```{.bash .shell-prompt}
export PYTHONPATH=$PYTHONPATH:/<path_to_modules>
```

Next, we configure RobotCollision to use our CollisionArmTable model in `stretch_re1_user_yaml`:

```yaml
robot_collision:
  models:
  - collision_arm_table

collision_arm_table:
  enabled: 1
  py_class_name: 'CollisionArmTable'
  py_module_name: 'collision_arm_table'

```

Finally, test out the model by driving the arm and lift around using the Xbox teleoperation tool:

```{.bash .shell-prompt}
stretch_xbox_controller_teleop.py
```

------
<div align="center"> All materials are Copyright 2022 by Hello Robot Inc. Hello Robot and Stretch are registered trademarks.</div>
