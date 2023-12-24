
# Tutorial: Custom Wrist DOF

In this tutorial, we explore how to add additional degrees of freedom to the Stretch wrist. 

Stretch exposes a Dynamixel X-Series TTL control bus at the end of its arm. It uses the [Dynamixel XL430-W250](https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/) for the [Wrist Yaw](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/wrist_yaw.py) and the [Stretch Gripper](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/stretch_gripper.py) that comes standard with the robot. 

See the [Hardware User Guide](https://docs.hello-robot.com/0.2/stretch-hardware-guides/docs/hardware_guide_re2/#wrist-tool-plate) to learn how to mechanically attach additional DOFs to the robot.

!!! note
    Stretch is compatible with any [Dynamixel X Series servo](https://emanual.robotis.com/docs/en/dxl/x/) that utilizes the TTL level Multidrop Bus.

## Adding a Custom DOF

Adding one or more custom Dynamixel X Series servos to Stretch wrist involves:

* Creating a new class that derives from [DynamixelHelloXL430](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/dynamixel_hello_XL430.py)
* Adding YAML parameters to `stretch_user_params.yaml` that configure the servo as desired
* Adding YAML parameters to `stretch_user_params.yaml` that tell Stretch to include this class in its EndOfArm list of servos

Let's create a new DOF called MyWristPitch in a file named [my_wrist_pitch.py](./custom_wrist_dof/my_wrist_pitch.py). Place the file somewhere on the $PYTHONPATH.

```python
from stretch_body.dynamixel_hello_XL430 import DynamixelHelloXL430
from stretch_body.hello_utils import *

class MyWristPitch(DynamixelHelloXL430):
    def __init__(self, chain=None):
        DynamixelHelloXL430.__init__(self, 'my_wrist_pitch', chain)
        self.poses = {'tool_up': deg_to_rad(45),
                      'tool_down': deg_to_rad(-45)}

    def pose(self,p,v_r=None,a_r=None):
        self.move_to(self.poses[p],v_r,a_r)
```

Now let's add the tools' parameters to your `stretch_user_params.yaml` to configure this servo. You may want to adapt these parameters to your application but the nominal values [found here](./custom_wrist_dof/stretch_user_params.yaml) usually work well. Below we highlight some of the more useful parameters.

```yaml
my_wrist_pitch:
  id: 1							#ID on the Dynamixel bus
  range_t:						#Range of servo, in ticks
  - 0
  - 4096
  req_calibration: 0			#Does the joint require homing after startup
  use_multiturn: 0				#Single turn or multi-turn mode of rotation
  zero_t: 2048					#Position in ticks that corresponds to zero radians
```

For this example, we are assuming a single-turn joint that doesn't require hard stop-based homing. We also assume the servo has the Robotis default ID of 1.

At this point, your MyWristPitch class is ready to use. Plug the servo into the cable leaving the Stretch WristYaw joint. Experiment with the API from iPython

```python
In [1]: import my_wrist_pitch

In [2]: w=wrist_pitch.WristPitch()

In [3]: w.startup()

In [4]: w.move_by(0.1)

In [5]: w.pose('tool_up')

In [6]: w.pose('tool_down')
```

Finally, you'll want to make your WristPitch available from `stretch_body.robot`. Add the following [YAML](./custom_wrist_dof/stretch_user_params.yaml) to your `stretch_user_params.yaml`

```yaml
end_of_arm:
  devices:
    wrist_pitch:
      py_class_name: WristPitch
      py_module_name: wrist_pitch
```

This tells `stretch_body.robot` to manage a `wrist_pitch.WristPitch` instance and add it to the [EndOfArm](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/end_of_arm.py) list of tools. Try it from iPython:

```python
In [1]: import stretch_body.robot as robot

In [2]: r=robot.Robot()

In [3]: r.startup()

In [4]: r.end_of_arm.move_by('wrist_pitch',0.1)

```

------
<div align="center"> All materials are Copyright 2022 by Hello Robot Inc. Hello Robot and Stretch are registered trademarks.</div>
