# Tutorial: Safety Features

Stretch includes a number of built-in functions that help it maintain safe operating conditions. These functions can be disabled and enabled via the robot user parameters.

## Logging

Upon instantiation, the Robot class opens a new log file for warning and informational messages to be written to. These timestamped logs are found under $HELLO_FLEET_DIRECTORY/log.

The logging messages can additionally be echoed to the console by setting:

```yaml
robot:
  log_to_console: 1
```

## Runstop Functions

The runstop deactivates all robot motion. It can be triggered by the physical button on the robot's head. It can also be triggered by internal monitors of the system state.  The default configuration of these parameters is:

```bash
>>$ stretch_params.py | grep stop_at
stretch_body.robot_params.nominal_params param.pimu.config.stop_at_cliff       0                             
stretch_body.robot_params.nominal_params param.pimu.config.stop_at_high_current   0                             
stretch_body.robot_params.nominal_params param.pimu.config.stop_at_low_voltage  1                             
stretch_body.robot_params.nominal_params param.pimu.config.stop_at_runstop 1                             
stretch_body.robot_params.nominal_params param.pimu.config.stop_at_tilt   0 
```

| Parameter            | Function                                                |
| -------------------- | ------------------------------------------------------- |
| stop_at_low_voltage  | Trigger runstop / beep when voltage too low             |
| stop_at_high_current | Trigger runstop when bus current too high               |
| stop_at_cliff        | Trigger runstop when a cliff sensor is outside of range |
| stop_at_runstop      | Allow runstop to disable motors                         |
| stop_at_tilt         | Trigger runstop when robot tilts too far                |

The [Pimu firmware](https://github.com/hello-robot/stretch_firmware/tree/master/arduino/hello_pimu) details the implementation of these functions.

**NOTE**: The `stop_at_cliff` and `stop_at_tilt` functions are disabled by default as they are not robust to normal operating conditions of the robot. Therefore do not rely on these functions for robot safety.

## Robot Monitor

The [Robot Monitor](https://github.com/hello-robot/stretch_body/blob/master/python/stretch_body/robot_monitor.py) is a thread that monitors the Robot Status data for significant events. For example, it can monitor the error flags from the Dynamixel servos and notify when a thermal overload occurs. The Robot Monitor logs warnings to a log file by default. T

The default parameters associated with RobotMonitor are:

```bash
>>$ stretch_params.py | grep monitor
...             
stretch_body.robot_params.nominal_params   param.robot.use_monitor            1               
stretch_body.robot_params.nominal_params  param.robot_monitor.monitor_base_bump_event    1                        
stretch_body.robot_params.nominal_params   param.robot_monitor.monitor_base_cliff_event          1                             
stretch_body.robot_params.nominal_params    param.robot_monitor.monitor_current       1                             
stretch_body.robot_params.nominal_params   param.robot_monitor.monitor_dynamixel_flags              1                             
stretch_body.robot_params.nominal_params   param.robot_monitor.monitor_guarded_contact               1      
stretch_body.robot_params.nominal_params   param.robot_monitor.monitor_over_tilt_alert     1                             
stretch_body.robot_params.nominal_params     param.robot_monitor.monitor_runstop        1                     
stretch_body.robot_params.nominal_params     param.robot_monitor.monitor_voltage                 1             
stretch_body.robot_params.nominal_params      param.robot_monitor.monitor_wrist_single_tap          1
```

| YAML                     | Function                                                     |
| ------------------------ | ------------------------------------------------------------ |
| monitor_base_bump_event  | Report when the accelerometer detects a bump event           |
| monitor_base_cliff_event | Report when a cliff sensor event occurs                      |
| monitor_current          | Report when the battery current exceeds desired range        |
| monitor_dynamixel_flags  | Report when a Dynamixel servo enters an error state          |
| monitor_guarded_contact  | Report when a guarded contact event occurs                   |
| monitor_over_tilt_alert  | Report when an over-tilt event occurs                        |
| monitor_runstop          | Report when the runstop is activated / deactivated           |
| monitor_voltage          | Report when the battery voltage is out of range              |
| monitor_wrist_single_tap | Report when the wrist accelerometer reports a single tap event |

Test out the RobotMonitor system by first enabling the console logging in stretch_user_params.yaml:

```yaml
robot:
  log_to_console: 1
```

The run the tool and hit the runstop button, then hold it down for 2 seconds:

```bash
>>$ stretch_robot_monitor.py
stretch_robot_monitor.py 
For use with S T R E T C H (R) RESEARCH EDITION from Hello Robot Inc.
---------------------------------------------------------------------

Starting Robot Monitor. Ctrl-C to exit
[INFO] [robot_monitor]: Runstop activated
[INFO] [robot_monitor]: Runstop deactivated
```



## Robot Sentry

The [Robot Sentry](https://github.com/hello-robot/stretch_body/blob/master/python/stretch_body/robot_sentry.py) is a thread that can override and also generate commands to the robot hardware. It's purpose is to keep the robot operating within a safe regime. For example, the Robot Sentry monitors the position of the Lift and Arm and limits the maximum base velocity and acceleration (in order to reduce the chance of toppling). The Robot Sentry reports events to the log file as well. 

| YAML                     | Function                                                     |
| ------------------------ | ------------------------------------------------------------ |
| base_fan_control         | Turn the fan on when CPU temp exceeds range                  |
| base_max_velocity        | Limit the base velocity when robot CG is high                |
| stretch_gripper_overload | Reset commanded position to prevent thermal overload during grasp |
| wrist_yaw_overload       | Reset commanded position to prevent thermal overload during pushing |

## Collision Avoidance

See the [Collision Avoidance Tutorial](./tutorial_collision_avoidance.md) for more information the the Stretch collision avoidance system.

------
<div align="center"> All materials are Copyright 2022 by Hello Robot Inc. Hello Robot and Stretch are registered trademarks.</div>
