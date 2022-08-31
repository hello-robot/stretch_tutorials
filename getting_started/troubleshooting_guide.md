# Stretch RE1 and RE2: Troubleshooting Guide

This guide covers common issues and ways to resolve them. Please check the [Hello Robot Forum](https://forum.hello-robot.com) for additional topics not covered here.

## XBox teleoperation is not working 

The provided Easy SMX wireless controller can accidentally be placed in the wrong mode. The mode is indicated by the round illuminated ring (shown as Connect below). The top 2 LEDs only should be illuminated. If a different LED pattern is shown then the button mapping expected by [stretch_xbox_controller_teleop.py](https://github.com/hello-robot/stretch_body/blob/master/tools/bin/stretch_xbox_controller_teleop.py) will be incorrect.

To set the controller into the correct mode:

- Hold the center button down for 5s. It will switch modes. Release.
- Repeat until top half of ring (upper two lights) are illuminated.

In addition, check that the provided USB dongle is plugged into the robot USB port in its trunk.

. 

![](./images/xbox.png)

## Battery is not staying charged

=== "Stretch RE1"

    Please review the troubleshooting section of the [RE1 Battery Maintenance Guide](https://docs.hello-robot.com/0.2/stretch-hardware-guides/battery_maintenance_guide_re1/).

=== "Stretch RE2"

    Please review the troubleshooting section of the [RE2 Battery Maintenance Guide](https://docs.hello-robot.com/0.2/stretch-hardware-guides/battery_maintenance_guide_re2).

## RPC Transport Errors (Stretch doesn't respond to commands)

If more than one instance of Stretch Body's [Robot](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/robot.py) class is instantiated at a time, Stretch Body will report communication errors and will not always execute motion commands as expected. This is because [Robot](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/robot.py)  manages communications with the robot hardware and it doesn't support multiple writes to the USB devices.

These errors can appear as

```
Transport RX Error on RPC_ACK_SEND_BLOCK_MORE False 0 102
---- Debug Exception
--------------- New RPC -------------------------
Framer sent RPC_START_NEW_RPC
...
```

or as

```
IOError(None): None
...
```

To check if a instance of [Robot](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/robot.py) is already instantiated, you may use the Unix [top](https://www.unixtutorial.org/commands/top) command to monitor active processes. You may use the Unix [pkill](https://linuxize.com/post/pkill-command-in-linux/) command to end the background instance of Robot.

```console
$ pkill -9 python
```

As shipped, Stretch launches [stretch_xbox_controller_teleop.py](https://github.com/hello-robot/stretch_body/blob/master/tools/bin/stretch_xbox_controller_teleop.py) upon boot. It is necessary to turn off this automatic launch feature, otherwise your own Robot instance will conflict with this script. Additionally, if you are logged into multiple accounts, a Robot instance may be active in another user account.

To turn it off, search for 'Startup' from Ubuntu Activities. Uncheck the box for 'hello_robot_xbox_teleop'.

![](./images/xbox_off_rs.png)

------
<div align="center"> All materials are Copyright 2022 by Hello Robot Inc. Hello Robot and Stretch are registered trademarks. The Stretch RE1 and RE2 robots are covered by U.S. Patent 11,230,000 and other patents pending.</div>

