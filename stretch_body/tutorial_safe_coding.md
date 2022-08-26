## Safe Operation Features

Stretch includes a number of built-in functions that help it maintain safe operating conditions. These functions can be disabled and enabled via the robot YAML parameters.

### Logging

Upon instantiation, the Robot class opens a new log file for warning and informational messages to be written to. These timestamped logs are found under $HELLO_FLEET_DIRECTORY/log.

The logging messages can be echoed to the console by setting:

```yaml
robot:
  log_to_console: 1
```

### Runstop Functions

| YAML                 | Function                                                |
| -------------------- | ------------------------------------------------------- |
| stop_at_low_voltage  | Trigger runstop / beep when voltage too low             |
| stop_at_high_current | Trigger runstop when bus current too high               |
| stop_at_cliff        | Trigger runstop when a cliff sensor is outside of range |
| stop_at_runstop      | Allow runstop to disable motors                         |
| stop_at_tilt         | Trigger runstop when robot tilts too far                |

### Robot Monitor

The [Robot Monitor](https://github.com/hello-robot/stretch_body/blob/master/python/stretch_body/robot_monitor.py) is a thread that monitors the Robot Status data for significant events. For example, it can monitor the error flags from the Dynamixel servos and notify when a thermal overload occurs. The Robot Monitor logs warnings to a log file by default. 

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

The YAML below illustrates the types of events that are can be configured.

```yaml
robot:
  log_to_console: 0
  use_monitor: 1
  use_sentry: 1
  
robot_monitor:
  monitor_base_bump_event: 1
  monitor_base_cliff_event: 1
  monitor_current: 1
  monitor_dynamixel_flags: 1
  monitor_guarded_contact: 1
  monitor_over_tilt_alert: 1
  monitor_runstop: 1
  monitor_voltage: 1
  monitor_wrist_single_tap: 1

robot_sentry:
  base_fan_control: 1
  base_max_velocity: 1
  stretch_gripper_overload: 1
  wrist_yaw_overload: 1
```

### Robot Sentry

The [Robot Sentry](https://github.com/hello-robot/stretch_body/blob/master/python/stretch_body/robot_sentry.py) is a thread that can override and also generate commands to the robot hardware. It's purpose is to keep the robot operating within a safe regime. For example, the Robot Sentry monitors the position of the Lift and Arm and limits the maximum base velocity and acceleration (in order to reduce the chance of toppling). The Robot Sentry reports events to the log file as well. 

| YAML                     | Function                                                     |
| ------------------------ | ------------------------------------------------------------ |
| base_fan_control         | Turn the fan on when CPU temp exceeds range                  |
| base_max_velocity        | Limit the base velocity when robot CG is high                |
| stretch_gripper_overload | Reset commanded position to prevent thermal overload during grasp |
| wrist_yaw_overload       | Reset commanded position to prevent thermal overload during pushing |

------
