# Robot Drivers

This tutorial introduces the ROS 2 drivers for Stretch and its sensors. These drivers reside in the `stretch_core` package, which is part of a suite of ROS 2 packages available open-source [on Github](https://github.com/hello-robot/stretch_ros2/tree/humble/stretch_core#overview).

## Stretch Driver

The robot's driver is called "Stretch Driver". You can use it to send motion commands and read joint info. The command to launch Stretch Driver is:

```{.bash .shell-prompt .copy}
ros2 launch stretch_core stretch_driver.launch.py rviz:=True
```

<!-- ![Default Stretch Rviz view](#TODO) -->

### Modes

Stretch Driver has a few modes that change how the driver behaves. They are:

 - **Position:** The default and simplest mode. In this mode, you can control every joint on the robot using position commands. For example, you could move the telescoping arm 25cm out by sending the driver a command of 0.25m for the "joint_arm" joint. Two kinds of position commands are available for the mobile base: translation and rotation, with joint names "translate_mobile_base" and "rotate_mobile_base". They are mutually exclusive and these joints have no limits since the wheels can spin continuously.
    - Position commands are tracked by a trapezoidal motion profile.
    - Position commands are contact sensitive. This is helpful for manipulating objects in the world. For example, Stretch can [open a cabinet](https://youtu.be/SXgj9be3PdM) by reaching out with the telescoping arm and detecting contact with the door and its handle.
    - Position commands are preemptible, so you can issue a new position command before the previous one has finished executing and the robot will smoothly transition to executing the new command. This feature is helpful for reactive control (e.g visual servo-ing).
 - **Navigation:** In this mode, every joint behaves identically to "position" mode except for the mobile base, which takes velocity commands.
    - Velocity control of the base is a common way to move mobile robots around. For example, Nav2 is a piece of software that uses this mode to navigate Stretch within its environment.
    - Velocity control has the potential to cause unsafe "running away" behavior if your program were to command a velocity motion and neglect to stop the motion later. Stretch has a few safety features to prevent this "running away" behavior. The driver has a 0.5 second timeout, which means that if the driver doesn't receive a new command within 0.5s, the base will stop smoothly. Additionally, the firmware has a 1 second timeout for the wheels, which means that even if the robot's onboard computer were to crash, the robot will stop if the firmware doesn't receive a new command within 1 second.
 - **Trajectory:** In this mode, every joint follows a trajectory that is modeled as a spline. The key benefit of this mode is control over the timing at which the robot achieves positions (a.k.a waypoints), enabling smooth and coordinated motion through a preplanned trajectory.
    - Trajectory commands are contact sensitive.
    - The waypoints in trajectory commands are preemptible.
 - **Homing:** This is the mode reported by the driver when a homing sequence is happening. While this mode is active, no commands will be accepted by the driver and the mode cannot be switched. After the robot has completed its 30 second homing sequence, it will return to the mode it was in before.
 - **Stowing:** This is the mode reported by the driver when a stowing sequence is happening. While this mode is active, no commands will be accepted by the driver and this mode cannot be switched. After the robot has completed its stowing sequence, it will return to the mode it was in before.
 - **Runstopped:** This is the mode reported by the driver when the robot is in runstop. You can runstop Stretch by pressing the runstop button (i.e. the glowing white button in Stretch's head). While this mode is active, no commands will be accepted by the driver and the mode cannot be switched. After the robot has been taken out of runstop, it will return to the mode it was in before, or "position" mode if the driver was launched while the robot was runstopped.

The driver publishes its current mode at the `/stretch/mode` topic, so you can see the driver's current mode using:

```{.bash .shell-prompt .copy}
ros2 topic echo /stretch/mode
# TODO - include output
```

### Keyboard Teleop

The keyboard teleop node is an easy way to send position commands. First, switch the driver into "position" mode using the `/stretch/switch_to_position_mode` service:

```{.bash .shell-prompt .copy}
ros2 service call /stretch/switch_to_position_mode
# TODO - verify command and include output
```

Then, start the keyboard teleop node:

```{.bash .shell-prompt .copy}
ros2 run stretch_core keyboard_teleop
```

### Joints & Ranges

You can read the robot's current joint state by echo-ing the `/stretch/joint_states` topic:

```{.bash .shell-prompt .copy}
ros2 topic echo /stretch/joint_states
# TODO - verify output
header:
  seq: 70999
  stamp:
    secs: 1420
    nsecs:   2000000
  frame_id: ''
name: [joint_arm_l0, joint_arm_l1, joint_arm_l2, joint_arm_l3, joint_gripper_finger_left,
  joint_gripper_finger_right, joint_head_pan, joint_head_tilt, joint_left_wheel, joint_lift,
  joint_right_wheel, joint_wrist_yaw]
position: [-1.6137320244357253e-08, -2.9392484829061376e-07, -2.8036125938539207e-07, -2.056847528567165e-07, -2.0518734302754638e-06, -5.98271107676851e-06, 2.9291786329821434e-07, 1.3802900147297237e-06, 0.08154086954434359, 1.4361499260374905e-07, 0.4139061738340768, 9.32603306580404e-07]
velocity: [0.00015598730463972836, -0.00029395074514369584, -0.0002803845454217379, 1.322424459109634e-05, -0.00035084643762840415, 0.0012164337445918797, 0.0002138814988808099, 0.00010419792027496809, 4.0575263146426684e-05, 0.00022487596895736357, -0.0007751929074042957, 0.0002451588607332439]
effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---
```

To stop the stream of joint states, press ++ctrl+c++. The position, velocity, and effort arrays match the length of the joint names array, so each joint's state is found by indexing into the arrays at the same index that the joint has in the joint names array. E.g. "joint_wrist_yaw" has position 9.32603306580404e-07, velocity 0.0002451588607332439, and effort 0.0.

### Motion Profiles

 - Position commands are tracked in the firmware by a trapezoidal motion profile, and specifying the optional velocity and acceleration in [JointTrajectoryPoint](https://docs.ros.org/en/noetic/api/trajectory_msgs/html/msg/JointTrajectoryPoint.html) changes the shape of the trapezoid. More details about the motion profile [in the tutorial](https://docs.hello-robot.com/0.2/stretch-tutorials/ros1/follow_joint_trajectory/).

### Contact Sensitivity

 - In order to specify contact thresholds for a position command, the optional effort in [JointTrajectoryPoint](https://docs.ros.org/en/noetic/api/trajectory_msgs/html/msg/JointTrajectoryPoint.html) is misused to mean a threshold for the effort the robot will apply while executing the position command.

### Preemption

### Other Services

```{.bash .shell-prompt .copy}
ros2 service call /stretch/home_the_robot
ros2 service call /stretch/stow_the_robot
ros2 service call /stretch/stop_the_robot
# TODO - verify commands and include outputs
```
