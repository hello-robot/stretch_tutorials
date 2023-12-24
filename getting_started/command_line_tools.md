# Command Line Tools

The Stretch robot comes with a set of command line tools that are helpful for introspection during general use or while troubleshooting issues. This page provides an overview of these tools. If you like, visit the [stretch_body](https://github.com/hello-robot/stretch_body/tree/master/tools/bin) repository to have a look under the hood.

You can execute these commands from anywhere in the terminal. We recommend you execute these commands as we follow each one of them. You can also find the description for the utility each tool provides by passing the optional '-h' flag along with the tool name in the terminal. For example, from anywhere in the terminal execute:
```{.bash .shell-prompt}
stretch_about.py -h
```

## System Information
### stretch_about.py
This tool displays the model and serial number information as an image.

### stretch_about_text.py
This tool displays the model and serial number information as text.

### stretch_version.sh
This script prints the version information for various software packages on the robot.

### stretch_params.py
This tool prints the Stretch parameters to the console.

### stretch_robot_monitor.py
This tool runs the Robot Monitor and prints to the console.

### stretch_robot_urdf_visualizer.py
This tool allows you to visualize robot URDF.

## Introspection
### stretch_robot_system_check.py
This tool checks that all robot hardware is present and is reporting sane values.

### stretch_robot_battery_check.py
This is a tool to print the battery state to the console.

### stretch_hardware_echo.py
This tool echoes the robot and computer hardware details to the console.

### stretch_robot_dynamixel_reboot.py
This tool reboots all Dynamixel servos on the robot.

### stretch_pimu_scope.py
This tool allows you to visualize Pimu (Power+IMU) board data with an oscilloscope. Pass the '-h' flag along with the command to see optional arguments.

### stretch_wacc_scope.py
This is a tool to visualize Wacc (Wrist+Accel) board data with an oscilloscope. Pass the '-h' flag along with the command to see optional arguments.

### stretch_realsense_visualizer.py
This is a tool to test the Realsense D435i Camera. Pass the '-h' flag along with the command to see optional arguments.

### stretch_respeaker_test.py
This tool allows you to record and playback audio via Respeaker.

### stretch_audio_test.py
This tool allows you to test the audio system.

## Homing Joints
### stretch_robot_home.py
This tool calibrates the robot by finding zeros for all robot joints.

### stretch_gripper_home.py
This tool calibrates the gripper position by closing until the motion stops.

### stretch_wrist_yaw_home.py
This tool calibrates the wrist_yaw position by moving to both hardstops.

### stretch_arm_home.py
This tool calibrates arm position by moving to hardstop.

### stretch_lift_home.py
This tool calibrates the lift position by moving to the upper hardstop.

## Jogging Joints
### stretch_robot_jog.py
This tool prints all robot data to the console.

### stretch_gripper_jog.py
This tool allows you to jog the griper from the keyboard.

### stretch_wrist_yaw_jog.py
This tool allows you to jog the wrist_yaw joint from the keyboard.

### stretch_arm_jog.py
This tool allows you to jog the arm motion from the keyboard.

### stretch_lift_jog.py
This tool allows you to jog the lift motion from the keyboard.

### stretch_base_jog.py
This tool allows you to jog the base motion from the keyboard.

### stretch_head_jog.py
This tool allows you to jog the head from the keyboard.

## Jogging Modules
### stretch_wacc_jog.py
This tool allows you to command and query the Wacc (Wrist Accelerometer) board from the keyboard.

### stretch_pimu_jog.py
This tool allows you to command and query the Pimu (Power+IMU) board from the keyboard.

### stretch_rp_lidar_jog.py
This is a tool to control the RP-Lidar. Pass the '-h' flag along with the command to see optional arguments.

### stretch_trajectory_jog.py
This tool allows you to test out splined trajectories on the various joint from a GUI or text menu. Pass the '-h' flag along with the command to see optional arguments.

## Teleoperation
### stretch_robot_stow.py
This tool moves the robot to stow position.

### stretch_robot_keyboard_teleop.py
This tool allows you to control the robot base, lift, arm, head, and tool from the keyboard.

### stretch_xbox_controller_teleop.py
This tool allows you to jog the robot from an Xbox Controller.