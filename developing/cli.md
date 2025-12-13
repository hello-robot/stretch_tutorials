# Command Line Tools

This tutorial covers the command line tools included in the Stretch CLI. For a primer on using CLIs and the terminal, see [The Basics](basics.md#terminal) tutorial. The Stretch CLI is split among Stretch's Python packages. They are:

 - Stretch Body - Tools that perform common tasks that are useful when developing with Stretch.
 - Stretch PyFUNMAP - Tools that explore the capabilities of PyFUNMAP.
 - Stretch Diagnostics - Tools for diagnosing issues with your robot.
 - Stretch URDF - Tools for working with Stretch's URDFs.
 - Stretch Factory - Tools used at Hello Robot during the robot's bring-up.

## Stretch Body Tools

These tools perform common tasks that are useful when working with Stretch (e.g. homing routine, joint jogging, etc.).

### `stretch_robot_system_check.py`
[Link to source code](https://github.com/hello-robot/stretch_body/blob/master/tools/bin/stretch_robot_system_check.py)

The robot system check tool runs a series of tests, such as if each subsystem is online, and reports pass or fail. If everything passes in the report, the robot is ready to be used. You would use this tool every time you use the robot. A passing output should look like:

<div class="shell-prompt highlight"><pre><span></span><code tabindex="0">stretch_robot_system_check.py<span class="w"> </span>
For<span class="w"> </span>use<span class="w"> </span>with<span class="w"> </span>S<span class="w"> </span>T<span class="w"> </span>R<span class="w"> </span>E<span class="w"> </span>T<span class="w"> </span>C<span class="w"> </span>H<span class="w"> </span><span class="o">(</span>R<span class="o">)</span><span class="w"> </span>RESEARCH<span class="w"> </span>EDITION<span class="w"> </span>from<span class="w"> </span>Hello<span class="w"> </span>Robot<span class="w"> </span>Inc.
---------------------------------------------------------------------

----<span class="w"> </span>Checking<span class="w"> </span>Devices<span class="w"> </span>----
<span class="s">[Pass] : hello-wacc</span>
<span class="s">[Pass] : hello-motor-left-wheel</span>
<span class="s">[Pass] : hello-pimu</span>
<span class="s">[Pass] : hello-lrf</span>
<span class="s">[Pass] : hello-dynamixel-head</span>
<span class="s">[Pass] : hello-dynamixel-wrist</span>
<span class="s">[Pass] : hello-motor-arm</span>
<span class="s">[Pass] : hello-motor-right-wheel</span>
<span class="s">[Pass] : hello-motor-lift</span>
<span class="s">[Pass] : hello-respeaker</span>

----<span class="w"> </span>Checking<span class="w"> </span>Pimu<span class="w"> </span>----
<span class="s">[Pass] Voltage = 12.863744497299194</span>
<span class="s">[Pass] Current = 2.67002009121435</span>
<span class="s">[Pass] Temperature = 23.45359845039172</span>
<span class="s">[Pass] Cliff-0 = 16.7066650390625</span>
<span class="s">[Pass] Cliff-1 = 0.47015380859375</span>
<span class="s">[Pass] Cliff-2 = -15.3138427734375</span>
<span class="s">[Pass] Cliff-3 = -5.50537109375</span>
<span class="s">[Pass] IMU AZ = -9.81534139672</span>
<span class="s">[Pass] IMU Pitch = 0.0</span>
<span class="s">[Pass] IMU Roll = 0.0</span>

----<span class="w"> </span>Checking<span class="w"> </span>EndOfArm<span class="w"> </span>----
<span class="s">[Pass] Ping of: wrist_pitch</span>
<span class="s">[Pass] Ping of: wrist_roll</span>
<span class="s">[Pass] Ping of: wrist_yaw</span>
<span class="s">[Pass] Homed: wrist_yaw</span>
<span class="s">[Pass] Ping of: stretch_gripper</span>
<span class="s">[Pass] Homed: stretch_gripper</span>


----<span class="w"> </span>Checking<span class="w"> </span>Head<span class="w"> </span>----
<span class="s">[Pass] Ping of: head_pan</span>
<span class="s">[Pass] Ping of: head_tilt</span>

----<span class="w"> </span>Checking<span class="w"> </span>Wacc<span class="w"> </span>----
<span class="s">[Pass] AX = 9.628660202026367</span>

----<span class="w"> </span>Checking<span class="w"> </span>hello-motor-left-wheel<span class="w"> </span>----
<span class="s">[Pass] Position = -3.035825729370117</span>

----<span class="w"> </span>Checking<span class="w"> </span>hello-motor-right-wheel<span class="w"> </span>----
<span class="s">[Pass] Position = 2.784149408340454</span>

----<span class="w"> </span>Checking<span class="w"> </span>hello-motor-arm<span class="w"> </span>----
<span class="s">[Pass] Position = 0.013962420634925365</span>
<span class="s">[Pass] Position Homed = True</span>

----<span class="w"> </span>Checking<span class="w"> </span>hello-motor-lift<span class="w"> </span>----
<span class="s">[Pass] Position = 31.580163955688477</span>
<span class="s">[Pass] Position Homed = True</span>

----<span class="w"> </span>Checking<span class="w"> </span><span class="w">for</span><span class="w"> </span>Intel<span class="w"> </span>D435i<span class="w"> </span>----
<span class="w">Bus 002 Device 002: ID 8086:0b3a Intel Corp. Intel(R) RealSense(TM) Depth Camera 435i</span>
<span class="s">[Pass] : Device found</span>

----<span class="w"> </span>Checking<span class="w"> </span>Software<span class="w"> </span>----
<span class="s">[Pass] Ubuntu 22.04 is ready</span>
<span class="s">[Pass] All APT pkgs are setup correctly</span>
<span class="s">[Pass] Firmware is up-to-date</span>
<span class="k">         hello-pimu = </span><span class="m">v0.6.2p4</span>
<span class="k">         hello-wacc = </span><span class="m">v0.5.1p3</span>
<span class="k">         hello-motor-arm = </span><span class="m">v0.6.2p4</span>
<span class="k">         hello-motor-lift = </span><span class="m">v0.6.3p4</span>
<span class="k">         hello-motor-left-wheel = </span><span class="m">v0.6.2p4</span>
<span class="k">         hello-motor-right-wheel = </span><span class="m">v0.6.2p4</span>
<span class="s">[Pass] Python pkgs are up-to-date</span>
<span class="k">         hello-robot-stretch-body = </span><span class="m">0.6.8</span>
<span class="k">         hello-robot-stretch-body-tools = </span><span class="m">.6.3</span>
<span class="k">         hello-robot-stretch-tool-share = </span><span class="m">0.2.8</span>
<span class="k">         hello-robot-stretch-factory = </span><span class="m">0.4.13</span>
<span class="k">         hello-robot-stretch-diagnostics = </span><span class="m">0.0.14</span>
<span class="k">         hello-robot-stretch-urdf = </span><span class="m">0.0.18</span>
<span class="s">[Pass] ROS 2 Humble is ready</span>
<span class="k">         Workspace at ~/ament_ws/src/stretch_ros2</span>
</code></pre></div>

In addition to checking the robot's hardware, this tool also prints out a software report. This includes which version of Stretch's Python, ROS 2, etc. software your system has installed. Check out the [Keeping your Software Up-to-date](../../../software/updating_software/#identifying-your-current-software) guide for a how-to on using this report to keep your robot's software up-to-date.

### `stretch_robot_home.py`
[Link to source code](https://github.com/hello-robot/stretch_body/blob/master/tools/bin/stretch_robot_home.py)

This tool will start Stretch's homing procedure, where every joint's zero is found. Robots with relative encoders (vs absolute encoders) need a homing procedure when they power on. For Stretch, it's a 30-second procedure that must occur every time the robot is powered on before you may send motion commands to or read correct joint positions from Stretch's joints. Normal output from this tool looks like:

```{.whatever .shell-prompt}
stretch_robot_home.py
For use with S T R E T C H (R) RESEARCH EDITION from Hello Robot Inc.
---------------------------------------------------------------------

--------- Homing Head ----
--------- Homing Lift ----
Homing Lift...
Hardstop detected at motor position (rad) 105.44721221923828
Marking Lift position to 1.096683 (m)
Marking Lift position to 0.000000 (m)
[INFO] [robot_monitor]: Guarded contact lift
Lift homing successful
--------- Homing Arm ----
Homing Arm...
Hardstop detected at motor position (rad) -1.5079643726348877
Marking Arm position to 0.000000 (m)
[INFO] [robot_monitor]: Guarded contact arm
[INFO] [robot_monitor]: Wrist single tap: 9
Arm homing successful
Moving to first hardstop...
First hardstop contact at position (ticks): 4097
-----
Homing offset was 3671
Marking current position to zero ticks
Homing offset is now  -427 (ticks)
-----
Current position (ticks): 24
Moving to calibrated zero: (rad)
[INFO] [robot_monitor]: Wrist single tap: 11
[INFO] [robot_monitor]: Wrist single tap: 15
[INFO] [robot_monitor]: Wrist single tap: 21
[INFO] [robot_monitor]: Wrist single tap: 27
[INFO] [robot_monitor]: Wrist single tap: 28
[INFO] [robot_monitor]: Wrist single tap: 33
[INFO] [robot_monitor]: Wrist single tap: 38
[INFO] [robot_monitor]: Wrist single tap: 40
Moving to first hardstop...
First hardstop contact at position (ticks): -9
-----
Homing offset was 2234
Marking current position to zero ticks
Homing offset is now  2237 (ticks)
-----
Current position (ticks): 35
Moving to calibrated zero: (rad)
```

### `stretch_robot_stow.py`

Useful to return the robot arm to a safe position within the base footprint. Normal output from this tool looks like:

```{.whatever .shell-prompt}
stretch_robot_stow.py
For use with S T R E T C H (R) RESEARCH EDITION from Hello Robot Inc.
---------------------------------------------------------------------

--------- Stowing Arm ----
--------- Stowing EOA_Wrist_DW3_Tool_SG3 ----
--------- Stowing Lift ----

```

### `stretch_robot_battery_check.py`

A quick way to check the robot's battery voltage / current consumption. Normal output from this tool looks like:

<div class="shell-prompt highlight"><pre><span></span><code>stretch_robot_battery_check.py
For use with S T R E T C H (R) RESEARCH EDITION from Hello Robot Inc.
---------------------------------------------------------------------

<span class="s">[Pass] Voltage with 12.791140079498291</span>
<span class="s">[Pass] Current with 6.154119995023523</span>
<span class="s">[Pass] CPU Temp with 45</span>
</code></pre></div>

### `stretch_robot_keyboard_teleop.py`

This tool enables jogging of the robot's joints from the keyboard.

5

What's the difference between jog and `stretch_robot_keyboard_teleop.py`

### `stretch_xbox_controller_teleop.py`

6

Useful to quickly test if a robot can achieve a task by manually teleoperating the robot

### `stretch_gamepad_teleop.py`

7

### `stretch_free_robot_process.py`

8

### `stretch_params.py`

9

This tool prints the Stretch parameters to the console.

### `stretch_realsense_visualizer.py`

10

This is a tool to test the Realsense D435i Camera. Pass the '-h' flag along with the command to see optional arguments.

### `stretch_rp_lidar_jog.py`

11

### `stretch_audio_test.py`

12

This tool allows you to test the audio system.

### `stretch_respeaker_test.py`

13

This tool allows you to record and playback audio via Respeaker.

### `stretch_<device>_home.py`

14

 - stretch_arm_home.py
 - stretch_gripper_home.py
 - stretch_lift_home.py
 - stretch_wrist_yaw_home.py

### `stretch_<device>_jog.py`

15

 - stretch_arm_jog.py
 - stretch_base_jog.py
 - stretch_gripper_jog.py
 - stretch_head_jog.py
 - stretch_lift_jog.py
 - stretch_pimu_jog.py
 - stretch_wacc_jog.py
 - stretch_wrist_yaw_jog.py

### `stretch_<device>_scope.py`

16

 - stretch_pimu_scope.py
   - This tool allows you to visualize Pimu (Power+IMU) board data with an oscilloscope. Pass the '-h' flag along with the command to see optional arguments.
 - stretch_wacc_scope.py
   - This is a tool to visualize Wacc (Wrist+Accel) board data with an oscilloscope. Pass the '-h' flag along with the command to see optional arguments.

### `stretch_about.py`

17

This tool displays the model and serial number information as an image.

### `stretch_about_text.py`

18

This tool displays the model and serial number information as text.

### `stretch_hardware_echo.py`

Can we get rid of this?

This tool echoes the robot and computer hardware details to the console.

### `stretch_trajectory_jog.py`

19

### `stretch_robot_dynamixel_reboot.py`

20

Resets all Dynamixels in the robot, which might be necessary if a servo overheats during use and enters an error state.

This tool reboots all Dynamixel servos on the robot.

### `stretch_robot_monitor.py`

21

What does this do?

This tool runs the Robot Monitor and prints to the console.

### `stretch_robot_urdf_visualizer.py`

Can we replace this with the new Stretch URDF tools?

This tool allows you to visualize robot URDF.

### `stretch_version.sh`

Can we get rid of this?

This script prints the version information for various software packages on the robot.


## Stretch PyFUNMAP Tools

### pyfunmap_head_scan_visualizer.py


## Stretch Diagnostics Tools

### `stretch_diagnostic_check.py`


## Stretch URDF Tools

### stretch_urdf_example.py

### stretch_urdf_viz.py


## Stretch Factory Tools

These tools are used at Hello Robot during the robot's system bring-up. They generally interact with the lowest level interface of the hardware, making measurements and writing calibration data to the robot's calibration folder.

!!! warning

    It is possible to cause bodily harm and/or break your robot with these tools. Used improperly, these tools might not respect joint torque and position limits. They may overwrite existing calibration data as well.

### RE1_migrate_contacts.py

### RE1_migrate_params.py

### REx_D435i_check.py

### REx_base_calibrate_imu_collect.py

### REx_base_calibrate_imu_process.py

### REx_base_calibrate_wheel_separation.py

### REx_calibrate_gravity_comp.py

### REx_calibrate_guarded_contact.py

### REx_calibrate_range.py

### REx_cliff_sensor_calibrate.py

### REx_comm_rates.py

### REx_discover_hello_devices.py

### REx_dmesg_monitor.py

### REx_dynamixel_id_change.py

### REx_dynamixel_id_scan.py

### REx_dynamixel_jog.py

### REx_dynamixel_reboot.py

### REx_dynamixel_set_baud.py

### REx_firmware_flash.py

### REx_firmware_updater.py

### REx_gamepad_configure.py

### REx_gripper_calibrate.py

### REx_hello_dynamixel_jog.py

### REx_stepper_calibration_YAML_to_flash.py

### REx_stepper_calibration_flash_to_YAML.py

### REx_stepper_calibration_run.py

### REx_stepper_ctrl_tuning.py

### REx_stepper_gains.py

### REx_stepper_jog.py

### REx_stepper_mechaduino_menu.py

### REx_trace_firmware.py

### REx_trace_robot.py

### REx_usb_reset.py

### REx_wacc_calibrate.py
