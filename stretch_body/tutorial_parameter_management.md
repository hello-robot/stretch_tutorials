# Tutorial: Parameter Management

In this tutorial, we will discuss how parameters are managed in Stretch Body and show examples of how to customize your robot by overriding parameters.

## Overview

Stretch Body shares a global set of parameters across all of the hardware it manages. All members of the [Device class](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/device.py) have an instance of [RobotParams](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/robot_params.py). This class constructs a dictionary of the device parameters as well as the global parameters for each device. For example, from iPython try:

```python
import stretch_body.arm
a=stretch_body.arm.Arm()

a.params
```

```{.python .no-copy}
Out[7]: 
{'usb_name': '/dev/hello-motor-arm',
 'force_N_per_A': 55.9
 'chain_pitch': 0.0167,
 'chain_sprocket_teeth': 10,
 'gr_spur': 3.875,
 'i_feedforward': 0,
 'calibration_range_bounds': [0.515, 0.525],
 'contact_model': 'effort_pct',
 'contact_model_homing': 'effort_pct',
 'contact_models': {'effort_pct': {'contact_thresh_calibration_margin': 10.0,
   'contact_thresh_max': [-90.0, 90.0],
   'contact_thresh_default': [-45.0, 45.0],
   'contact_thresh_homing': [-45.0, 45.0]}},
 'motion': {'default': {'accel_m': 0.14, 'vel_m': 0.14},
  'fast': {'accel_m': 0.3, 'vel_m': 0.3},
  'max': {'accel_m': 0.4, 'vel_m': 0.4},
  'slow': {'accel_m': 0.05, 'vel_m': 0.05},
  'trajectory_max': {'vel_m': 0.4, 'accel_m': 0.4}},
 'range_m': [0.0, 0.52]}
```

or to access another device params:

```python
a.robot_params['lift']
```

```{.python .no-copy}
Out[9]: 
{'usb_name': '/dev/hello-motor-lift',
 'force_N_per_A': 75.0
 'calibration_range_bounds': [1.094, 1.106],
 'contact_model': 'effort_pct',
 'contact_model_homing': 'effort_pct',
 'contact_models': {'effort_pct': {'contact_thresh_calibration_margin': 10.0,
   'contact_thresh_max': [-100, 100],
   'contact_thresh_default': [-69.0, 69.0],
   'contact_thresh_homing': [-69.0, 69.0]}},
 'belt_pitch_m': 0.005,
 'motion': {'default': {'accel_m': 0.2, 'vel_m': 0.11},
  'fast': {'accel_m': 0.25, 'vel_m': 0.13},
  'max': {'accel_m': 0.3, 'vel_m': 0.15},
  'slow': {'accel_m': 0.05, 'vel_m': 0.05},
  'trajectory_max': {'accel_m': 0.3, 'vel_m': 0.15}},
 'pinion_t': 12,
 'i_feedforward': 1.2,
 'range_m': [0.0, 1.1]}
```

## Parameter Organization

Stretch Body utilizes a prioritized parameter organization such that default settings can be easily overridden 

| Priority | Name                  | Location                                                     | Description                                                  |
| -------- | --------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| 1        | user_params           | $HELLO_FLEET_PATH/$HELLO_FLEET_ID/ stretch_user_params.yaml  | Yaml file for users to override default settings and to define custom configurations. |
| 2        | configuration_params  | $HELLO_FLEET_PATH/$HELLO_FLEET_ID/  stretch_configuration_params.yaml | Robot specific data  (eg, serial numbers and calibrations). Calibration tools may update these. |
| 3        | external_params       | Imported via a list defined as `params` in stretch_user_params.yaml | External Python parameter dictionaries for 3rd party devices and peripherals. |
| 4        | nominal_params        | stretch_body.robot_params_RE2V0.py                           | Generic systems settings (common across all robots of a given model. |
| 5        | nominal_system_params | stretch_body.robot_params.py                                 | Generic systems settings (common across all robot models).  |

 This allows the user to override any of the parameters by defining it in their `stretch_user_params.yaml`. It also allows Hello Robot to periodically update parameters defined in the Python files via Pip updates.

The tool `stretch_params.py` will print out all of the robot parameters as well as their origin. For example:

```{.bash .shell-prompt}
stretch_params.py 
```
```{.bash .no-copy}
############################################################ Parameters for stretch-re2-xxxx 
Origin          Parameter                                                              Value                         
--------------------------------------------------------------------------------------------------------------------------------- ...         
stretch_body.robot_params.nominal_params         param.arm.chain_pitch          0.0167                        
stretch_body.robot_params.nominal_params         param.arm.chain_sprocket_teeth     	10                                     ...               
stretch_configuration_params.yaml       param.arm.contact_models.effort_pct.contact_thresh_default     [-45.0, 45.0]         
```

## Manually Querying and Modifying Parameters

A quick way to query parameters is with the `stretch_params.py` tool. For example, to look at parameters relating to the arm motion:

```{.bash .shell-prompt}
stretch_params.py | grep arm | grep motion
```
```{.bash .no-copy}
stretch_body.robot_params.nominal_params	param.arm.motion.default.accel_m	0.14                          
stretch_body.robot_params.nominal_params	param.arm.motion.default.vel_m		0.14                          
stretch_body.robot_params.nominal_params	param.arm.motion.fast.accel_m		0.3                           
stretch_body.robot_params.nominal_params	param.arm.motion.fast.vel_m			0.3                           
stretch_body.robot_params.nominal_params	param.arm.motion.max.accel_m		0.4                           
stretch_body.robot_params.nominal_params	param.arm.motion.max.vel_m			0.4                           
stretch_body.robot_params.nominal_params	param.arm.motion.slow.accel_m		0.05                          
stretch_body.robot_params.nominal_params	param.arm.motion.slow.vel_m			0.05                          
...               
```

The tool displays each parameter's value as well as which parameter file it was loaded from.

For example, if you want to override the default motion settings for the arm, you could add the following to your `stretch_user_params.yaml`:

```yaml
arm:
  motion:
    default:
      vel_m: 0.1
      accel_m: 0.1
```

Run the tool again and we see:

```{.bash .shell-prompt}
stretch_params.py | grep arm | grep motion | grep default
```
```{.bash .no-copy}
stretch_body.robot_params.nominal_params	param.arm.motion.default.accel_m	0.1                          
stretch_body.robot_params.nominal_params	param.arm.motion.default.vel_m		0.1  
```

!!! note
    The factory parameter settings should suffice for most use cases. 

## Programmatically Modifying and Storing Parameters

A user can compute the value of a parameter programmatically and modify the robot settings accordingly. For example, in the Stretch Factory tool [REx_base_calibrate_wheel_separation.py](https://github.com/hello-robot/stretch_factory/blob/master/python/tools/REx_base_calibrate_wheel_separation.py) we see that the parameter `wheel_separation_m` is recomputed as the variable `d_avg`. This new value could be used during the robot execution by simply:

```python
robot.base.params['wheel_separation_m']=d_vag
```

or it could be saved as a user override:

```python
robot.write_user_param_to_YAML('base.wheel_separation_m', d_avg)
```

This will update the file `stretch_user_params.yaml`.

------
<div align="center"> All materials are Copyright 2022 by Hello Robot Inc. Hello Robot and Stretch are registered trademarks.</div>
