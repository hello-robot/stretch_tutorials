# Stretch Troubleshooting Guide

This guide covers common issues and ways to resolve them. Please check the [Hello Robot Forum](https://forum.hello-robot.com) for additional topics not covered here.

## Xbox teleoperation is not working 

The provided Easy SMX wireless controller can accidentally be placed in the wrong mode. The mode is indicated by the round illuminated ring (shown as Connect below). The top 2 LEDs only should be illuminated. If a different LED pattern is shown then the button mapping expected by [stretch_xbox_controller_teleop.py](https://github.com/hello-robot/stretch_body/blob/master/tools/bin/stretch_xbox_controller_teleop.py) will be incorrect.

To set the controller into the correct mode:

- Hold the center button down for 5s. It will switch modes. Release.
- Repeat until the top half of the ring (upper two lights) is illuminated.

In addition, check that the provided USB dongle is plugged into the robot USB port in its trunk.

![](./images/xbox.png)

## Battery is not staying charged

=== "Stretch RE1"

    Please review the troubleshooting section of the [Stretch RE1 Battery Maintenance Guide](https://docs.hello-robot.com/0.2/stretch-hardware-guides/battery_maintenance_guide_re1/).

=== "Stretch 2"

    Please review the troubleshooting section of the [Stretch 2 Battery Maintenance Guide](https://docs.hello-robot.com/0.2/stretch-hardware-guides/battery_maintenance_guide_re2).

## Port Busy or RPC Transport Errors

If more than one instance of Stretch Body's [Robot](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/robot.py) class is instantiated at a time, Stretch Body will report communication errors and will not always execute motion commands as expected. This is because the [Robot](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/robot.py) class manages communications with the robot hardware and it doesn't support multiple writes to the USB devices.

<details>
    <summary>Examples of busy port or RPC errors</summary>

    ```{.bash .no-copy}
    [ERROR] [pimu]: Port /dev/hello-pimu is busy. Check if another Stretch Body process is already running
    [WARNING] [pimu]: Unable to open serial port for device /dev/hello-pimu
    [ERROR] [hello-motor-left-wheel]: Port /dev/hello-motor-left-wheel is busy. Check if another Stretch Body process is already running
    [WARNING] [hello-motor-left-wheel]: Unable to open serial port for device /dev/hello-motor-left-wheel
    [ERROR] [hello-motor-right-wheel]: Port /dev/hello-motor-right-wheel is busy. Check if another Stretch Body process is already running
    [WARNING] [hello-motor-right-wheel]: Unable to open serial port for device /dev/hello-motor-right-wheel
    [ERROR] [hello-motor-lift]: Port /dev/hello-motor-lift is busy. Check if another Stretch Body process is already running
    [WARNING] [hello-motor-lift]: Unable to open serial port for device /dev/hello-motor-lift
    [ERROR] [hello-motor-arm]: Port /dev/hello-motor-arm is busy. Check if another Stretch Body process is already running
    [WARNING] [hello-motor-arm]: Unable to open serial port for device /dev/hello-motor-arm
    [ERROR] [wacc]: Port /dev/hello-wacc is busy. Check if another Stretch Body process is already running
    [WARNING] [wacc]: Unable to open serial port for device /dev/hello-wacc
    [ERROR] [pimu]: Port /dev/hello-pimu is busy. Check if another Stretch Body process is already running
    [WARNING] [pimu]: Unable to open serial port for device /dev/hello-pimu
    [ERROR] [hello-motor-left-wheel]: Port /dev/hello-motor-left-wheel is busy. Check if another Stretch Body process is already running
    [WARNING] [hello-motor-left-wheel]: Unable to open serial port for device /dev/hello-motor-left-wheel
    [ERROR] [hello-motor-lift]: Port /dev/hello-motor-lift is busy. Check if another Stretch Body process is already running
    [WARNING] [hello-motor-lift]: Unable to open serial port for device /dev/hello-motor-lift
    [ERROR] [hello-motor-arm]: Port /dev/hello-motor-arm is busy. Check if another Stretch Body process is already running
    [WARNING] [hello-motor-arm]: Unable to open serial port for device /dev/hello-motor-arm
    ```

    or

    ```{.bash .no-copy}
    [WARNING] [head_tilt]: DynamixelHelloXL430 Ping failed... head_tilt
    DynamixelHelloXL430 Ping failed... head_tilt
    ```

    or

    ```{.bash .no-copy}
    serial.serialutil.SerialException: device reports readiness to read but returned no data (device disconnected or multiple access on port?)
    ```

    or

    ```{.bash .no-copy}
    Traceback (most recent call last):
    File "/home/hello-robot/.local/bin/stretch_robot_system_check.py", line 42, in <module>
        r.startup()
    File "/home/hello-robot/repos/stretch_body/body/stretch_body/robot.py", line 167, in startup
        if not self.devices[k].startup(threaded=False):
    File "/home/hello-robot/repos/stretch_body/body/stretch_body/end_of_arm.py", line 26, in startup
        return DynamixelXChain.startup(self, threaded=threaded)
    File "/home/hello-robot/repos/stretch_body/body/stretch_body/dynamixel_X_chain.py", line 80, in startup
        if not self.motors[mk].startup(threaded=False):
    File "/home/hello-robot/repos/stretch_body/body/stretch_body/stretch_gripper.py", line 22, in startup
        return DynamixelHelloXL430.startup(self, threaded=threaded)
    File "/home/hello-robot/repos/stretch_body/body/stretch_body/dynamixel_hello_XL430.py", line 193, in startup
        self.motor.set_temperature_limit(self.params['temperature_limit'])
    File "/home/hello-robot/repos/stretch_body/body/stretch_body/dynamixel_XL430.py", line 707, in set_temperature_limit
        dxl_comm_result, dxl_error =   self.packet_handler.write1ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_TEMPERATURE_LIMIT, int(x))
    File "/home/hello-robot/.local/lib/python3.8/site-packages/dynamixel_sdk/protocol2_packet_handler.py", line 653, in write1ByteTxRx
        return self.writeTxRx(port, dxl_id, address, 1, data_write)
    File "/home/hello-robot/.local/lib/python3.8/site-packages/dynamixel_sdk/protocol2_packet_handler.py", line 643, in writeTxRx
        rxpacket, result, error = self.txRxPacket(port, txpacket)
    File "/home/hello-robot/.local/lib/python3.8/site-packages/dynamixel_sdk/protocol2_packet_handler.py", line 346, in txRxPacket
        rxpacket, result = self.rxPacket(port)
    File "/home/hello-robot/.local/lib/python3.8/site-packages/dynamixel_sdk/protocol2_packet_handler.py", line 257, in rxPacket
        rxpacket.extend(port.readPort(wait_length - rx_length))
    File "/home/hello-robot/.local/lib/python3.8/site-packages/dynamixel_sdk/port_handler.py", line 78, in readPort
        return self.ser.read(length)
    File "/usr/lib/python3/dist-packages/serial/serialposix.py", line 509, in read
        raise SerialException('read failed: {}'.format(e))
    serial.serialutil.SerialException: read failed: device reports readiness to read but returned no data (device disconnected or multiple access on port?)
    ```

    or

    ```{.bash .no-copy}
    Transport RX Error on RPC_ACK_SEND_BLOCK_MORE False 0 102
    ---- Debug Exception
    --------------- New RPC -------------------------
    Framer sent RPC_START_NEW_RPC
    ...
    ```

    or

    ```{.bash .no-copy}
    IOError(None): None
    ...
    ```
</details>

To check if an instance of [Robot](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/robot.py) is already instantiated, you may use the Unix [top](https://www.unixtutorial.org/commands/top) command to monitor active processes. You may use the Unix [pkill](https://linuxize.com/post/pkill-command-in-linux/) command to end the background instance of Robot.

```{.bash .shell-prompt}
pkill -9 python
```

As shipped, Stretch launches [stretch_xbox_controller_teleop.py](https://github.com/hello-robot/stretch_body/blob/master/tools/bin/stretch_xbox_controller_teleop.py) upon boot. It is necessary to turn off this automatic launch feature, otherwise, your own Robot instance will conflict with this script. Additionally, if you are logged into multiple accounts, a Robot instance may be active in another user account.

To turn it off, search for 'Startup' from Ubuntu Activities. Uncheck the box for 'hello_robot_xbox_teleop'.

![](./images/xbox_off_rs.png)

## Invalid homing offset for single-turn mode Dynamixel

If you see a warning similar to:

```{.no-copy}
[WARNING] [head_tilt]: Dynamixel head_tilt: Servo is in single-turn mode yet has invalid homing offset of -1682.
 This may cause unexpected results if not set to zero (using REx_dynamixel_jog.py)
 Please contact Hello Robot support for more information
```

You have a "single-turn" Dynamixel servo (e.g. `head_tilt` in the above warning) that has an invalid "homing offset" saved to the servo's flash memory. This can lead to the motor returning its joint position incorrectly. In order to reset the homing offset to zero, create an instance of the Dynamixel class in Stretch Body and set the homing offset to zero. For example, doing this for the `head_tilt` servo looks like:

```{.python .no-copy}
hello-robot@stretch-re1-xxxx:~$ ipython3
Python 3.8.10 (default, Mar 13 2023, 10:26:41)
Type 'copyright', 'credits' or 'license' for more information
IPython 8.7.0 -- An enhanced Interactive Python. Type '?' for help.

In [1]: import stretch_body.dynamixel_hello_XL430

In [2]: servo = stretch_body.dynamixel_hello_XL430.DynamixelHelloXL430(name="head_tilt", chain=None)

In [3]: servo.startup()
[WARNING] [head_tilt]: Dynamixel head_tilt: Servo is in single-turn mode yet has invalid homing offset of -1682.
 This may cause unexpected results if not set to zero (using REx_dynamixel_jog.py)
 Please contact Hello Robot support for more information
Out[3]: True

In [4]: servo.disable_torque()

In [5]: servo.motor.set_homing_offset(0)

In [6]: servo.enable_torque()

In [7]: servo.stop()

In [8]:
Do you really want to exit ([y]/n)?
```

------
<div align="center"> All materials are Copyright 2022 by Hello Robot Inc. Hello Robot and Stretch are registered trademarks.</div>