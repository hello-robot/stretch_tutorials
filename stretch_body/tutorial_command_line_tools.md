# Tutorial: Stretch Body Command Line Tools

Stretch Body includes the package [hello-robot-stretch-body-tools](https://github.com/hello-robot/stretch_body/tree/master/tools) --  a suite of command line tools that allow direct interaction with hardware subsystems. 

These tools are useful when developing and debugging applications. They also serve as code examples when developing applications for Stretch_Body.

These tools can be found by tab completion of  'stretch_' from a terminal.

```console
$ stretch_

stretch_about.py
stretch_about_text.py
stretch_arm_home.py
stretch_arm_jog.py
stretch_audio_test.py
stretch_base_jog.py
stretch_gripper_home.py
stretch_gripper_jog.py
stretch_hardware_echo.py
stretch_head_jog.py
stretch_lift_home.py
stretch_lift_jog.py
stretch_params.py
stretch_pimu_jog.py
stretch_pimu_scope.py
stretch_realsense_visualizer.py
stretch_respeaker_test.py
stretch_robot_battery_check.py
stretch_robot_dynamixel_reboot.py
stretch_robot_home.py
stretch_robot_jog.py
stretch_robot_keyboard_teleop.py
stretch_robot_monitor.py
stretch_robot_stow.py
stretch_robot_system_check.py
stretch_robot_urdf_visualizer.py
stretch_rp_lidar_jog.py
stretch_trajectory_jog.py
stretch_version.sh
stretch_wacc_jog.py
stretch_wacc_scope.py
stretch_wrist_yaw_home.py
stretch_wrist_yaw_jog.py
stretch_xbox_controller_teleop.py
```

All tools accept '--help' as a command line argument to learn its function. For example:

```console
>>$ stretch_pimu_scope.py --help
For use with S T R E T C H (R) RESEARCH EDITION from Hello Robot Inc.
---------------------------------------------------------------------

usage: stretch_pimu_scope.py [-h] [--cliff] [--at_cliff] [--voltage] [--current] [--temp] [--ax] [--ay] [--az] [--mx] [--my] [--mz] [--gx] [--gy] [--gz] [--roll] [--pitch] [--heading] [--bump]

Visualize Pimu (Power+IMU) board data with an oscilloscope

optional arguments:
  -h, --help  show this help message and exit
  --cliff     Scope base cliff sensors
  --at_cliff  Scope base at_cliff signal
  --voltage   Scope bus voltage (V)
  --current   Scope bus current (A)
  --temp      Scope base internal temperature (C)
  --ax        Scope base accelerometer AX
  --ay        Scope base accelerometer AY
  --az        Scope base accelerometer AZ
  --mx        Scope base magnetometer MX
  --my        Scope base magnetometer MY
  --mz        Scope base magnetometer MZ
  --gx        Scope base gyro GX
  --gy        Scope base gyro GY
  --gz        Scope base gyro GZ
  --roll      Scope base imu Roll
  --pitch     Scope base imu Pitch
  --heading   Scope base imu Heading
  --bump      Scope base imu bump level
```

### Commonly Used Tools

These are the tools a typical user will want to become familiar with.

| **Tool**                              | **Utility**                                                  |
| ------------------------------------- | ------------------------------------------------------------ |
| **stretch_robot_home.py**             | Commonly run after booting up the robot in-order to calibrate the joints |
| **stretch_robot_system_check.py**     | Scans for all hardware devices and ensure they are present on the bus and reporting valid values. Useful to verify that the robot is in good working order prior to commanding motion. It will report all success in green, failures in red. |
| **stretch_robot_stow.py**             | Useful to return the robot arm and tool to a safe position within the base footprint. It can also be useful if a program fails to exit cleanly and the robot joints are not backdriveable. It will restore them to their 'Safety' state. |
| **stretch_robot_battery_check.py**    | Quick way to check the battery voltage / current consumption |
| **stretch_xbox_controller_teleop.py** | Useful to quickly test if a robot can achieve a task by manually teleoperating the robot |
| **stretch_robot_dynamixel_reboot.py** | This will reset all Dynamixels in the robot, which may be needed if a servo overheats during high use and enters an error state. |

Take a minute to explore each of these tools from the console.



------
<div align="center"> All materials are Copyright 2022 by Hello Robot Inc. Hello Robot and Stretch are registered trademarks. The Stretch RE1 and RE2 robots are covered by U.S. Patent 11,230,000 and other patents pending.</div>

