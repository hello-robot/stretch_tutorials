# Tutorial: Robot Sensors

## Introduction

Stretch Body exposes a host of sensor data through the status dictionaries of its devices. In this tutorial, we'll cover how to access, view, and configure this sensor data. 

## Tools to View Sensor Data

There are two useful tools for scoping Pimu and Wacc sensor data in real-time:

```{.bash .shell-prompt}
stretch_pimu_scope.py --help
```
```{.bash .no-copy}
For use with S T R E T C H (R) RESEARCH EDITION from Hello Robot Inc.
---------------------------------------------------------------------

usage: stretch_pimu_scope.py [-h] (--cliff | --at_cliff | --voltage | --current | --temp | --ax | --ay | --az | --mx | --my | --mz | --gx | --gy | --gz | --roll | --pitch | --heading | --bump)

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

and,

```{.bash .shell-prompt}
stretch_wacc_scope.py --help
```
```{.bash .no-copy}
For use with S T R E T C H (R) RESEARCH EDITION from Hello Robot Inc.
---------------------------------------------------------------------

usage: stretch_wacc_scope.py [-h] [--ax] [--ay] [--az] [--a0] [--d0] [--d1] [--tap]

Visualize Wacc (Wrist+Accel) board data with an oscilloscope

optional arguments:
  -h, --help  show this help message and exit
  --ax        Scope accelerometer AX
  --ay        Scope accelerometer AY
  --az        Scope accelerometer AZ
  --a0        Scope analog-in-0
  --d0        Scope digital-in-0
  --d1        Scope digital-in-1
  --tap       Scope single tap

```

Each motor also has associated sensor data available in its status dictionaries. The corresponding 'jog' tool for each joint will pretty-print the sensor data for that motor to the console. For example:

```{.bash .shell-prompt}
stretch_arm_jog.py 
```
```{.bash .no-copy}
For use with S T R E T C H (R) RESEARCH EDITION from Hello Robot Inc.
---------------------------------------------------------------------
...
----- Arm ------ 
Pos (m):  0.0032848120914969895
Vel (m/s):  0.0002031017627742426
Soft motion limits (m) [0.0, 0.52]
Timestamp PC (s): 1661797443.1212385
-----------
Mode MODE_SAFETY
x_des (rad) 0 (deg) 0.0
v_des (rad) 25 (deg) 1432.3944878270581
a_des (rad) 15 (deg) 859.4366926962349
Stiffness 1
Feedforward 0
Pos (rad) 0.47890087962150574 (deg) 27.438999207414973
Vel (rad/s) 0.029610708355903625 (deg) 1.6965686171860386
Effort (Ticks) 0.0
Effort (Pct) 0.0
Current (A) 0.0
Error (deg) 0.0
Debug 0.0
Guarded Events: 0
Diag 00000000000000000000000100000000
       Position Calibrated: False
       Runstop on: False
       Near Pos Setpoint: False
       Near Vel Setpoint: False
       Is Moving: False
       Is Moving Filtered: 0
       At Current Limit: False
       Is MG Accelerating: False
       Is MG Moving: False
       Encoder Calibration in Flash: True
       In Guarded Event: False
       In Safety Event: False
       Waiting on Sync: False
Waypoint Trajectory
       State: idle
       Setpoint: (rad) 0.0 | (deg) 0.0
       Segment ID: 0
Timestamp (s) 1661797443.110996
Read error 0
Board variant: Stepper.1
Firmware version: Stepper.v0.2.0p1
...
```

## Accessing the Status Dictionaries

Each Robot device has a status dictionary that is automatically updated with the latest sensor data. The primary dictionaries are:

* [Stepper Status](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/stepper.py#L104)
* [Wacc Status](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/wacc.py#L44)
* [Pimu Status](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/pimu.py#L27)
* [Dynamixel Status](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/dynamixel_hello_XL430.py#L61)

Each of these dictionaries can be accessed through the Robot instance. For example, try in iPython:

```python
import stretch_body.robot
import time

r=stretch_body.robot.Robot()
r.startup()
for i in range(10):
    print('Arm position (m)%f'%r.arm.status['pos'])
    time.sleep(0.1) 
```

## Base IMU

The base has a 9-DoF IMU using the 9-DoF FXOS8700 + FXAS21002 chipset. This is the same chipset used on the [Adafruit NXP IMU board](https://www.adafruit.com/product/3463). 

The [Pimu](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/pimu.py) reports back the IMU sensor readings in its [IMU status dictionary](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/pimu.py#L27). For example, from iPython try:

```python
import stretch_body.robot
r=stretch_body.robot.Robot()
r.startup()

r.pimu.status['imu']
Out[6]: 
{'ax': 0.30007487535476685,
 'ay': -0.355493426322937,
 'az': -9.736297607421875,
 'gx': 0.0009544769418425858,
 'gy': 0.00013635384675581008,
 'gz': 0.00027270769351162016,
 'mx': -10.699999809265137,
 'my': -42.900001525878906,
 'mz': -51.0,
 'roll': 0.03657745230780231,
 'pitch': -0.02960890640560868,
 'heading': 1.2786458241584955,
 'timestamp': 1661788669.3042662,
 'qw': 0.0009681061492301524,
 'qx': 0.59670090675354,
 'qy': -0.8021157383918762,
 'qz': 0.023505505174398422,
 'bump': -0.9188174605369568}

r.pimu.pretty_print()
----------IMU -------------
AX (m/s^2) 0.30007487535476685
AY (m/s^2) -0.355493426322937
AZ (m/s^2) -9.736297607421875
GX (rad/s) 0.0009544769418425858
GY (rad/s) 0.00013635384675581008
GZ (rad/s) 0.00027270769351162016
MX (uTesla) -10.699999809265137
MY (uTesla) -42.900001525878906
MZ (uTesla) -51.0
QW 0.0009681061492301524
QX 0.59670090675354
QY -0.8021157383918762
QZ 0.023505505174398422
Roll (deg) 2.095733642578125
Pitch (deg) -1.6964653730392456
Heading (deg) 73.26100921630858
```

It reports:

* Acceleration (AX, AY, AZ)
* Gravity (GX, GY GZ)
* Magnetic field (MX, MY, MZ)
* Quaternion orientation (QW, QX, QY, QZ)
* Euler angle orientation (Roll, Pitch, Heading)

These values are computed on the Pimu. As we can see in [its firmware code](https://github.com/hello-robot/stretch_firmware/blob/master/arduino/hello_pimu/IMU.cpp), a 100Hz Madgwick filter is used to compute the orientation. 

Stretch Body also implements a bump detector using the IMU accelerometers. This detector simply [computes the sum of squares of AX, AY, and AZ](https://github.com/hello-robot/stretch_firmware/blob/master/arduino/hello_pimu/IMU.cpp#L223). This value is then compared to the following threshold to determine if a bump is detected:

```{.bash .shell-prompt}
stretch_params.py | grep pimu | grep bump
```
```{.bash .no-copy}
stretch_body.robot_params.nominal_params   param.pimu.config.bump_thresh       20.0 
```

You can experiment with the bump detector with the following code:

```python
import stretch_body.robot
r=stretch_body.robot.Robot()
r.startup()

r.pimu.config['bump_thresh']= 20.0 #Experiment with values
r.pimu.set_config(p.config)
r.push_command()

for i in range(100):
    time.sleep(0.1)
    print('Bump %f'%r.pimu.status['bump'])
    print('Bump event count %d'%r.pimu.status['bump_event_cnt'])
```

!!! note
    The IMU is calibrated by Hello Robot at the factory. Please contact Hello Robot support for details on recalibrating your IMU.

## Wrist Accelerometer

The wrist includes a 3 axis [ADXL343](https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL343.pdf) accelerometer which provides bump and tap detection capabilities. The Wacc reports back AX, AY, and AZ [in its status dictionary](https://github.com/hello-robot/stretch_body/blob/master/body/stretch_body/wacc.py#L44). For example, from iPython try:

```python
import stretch_body.robot
r=stretch_body.robot.Robot()
r.startup()
r.wacc.status

Out[5]: 
{'ax': 10.093315124511719,
 'ay': 0.10557432472705841,
 'az': -0.45386940240859985,
 'a0': 155,
 'd0': 1,
 'd1': 1,
 'd2': 0,
 'd3': 0,
 'single_tap_count': 15,
 'state': 0,
 'debug': 0,
 'timestamp': 1661795676.203578,
 'transport': {'rate': 0.4572487091345871,
  'read_error': 0,
  'write_error': 0,
  'itr': 3,
  'transaction_time_avg': 0,
  'transaction_time_max': 0,
  'timestamp_pc': 0}}

r.wacc.pretty_print()
------------------------------
Ax (m/s^2) 10.093315124511719
Ay (m/s^2) 0.10557432472705841
Az (m/s^2) -0.45386940240859985
A0 155
D0 (In) 1
D1 (In) 1
D2 (Out) 0
D3 (Out) 0
Single Tap Count 15
State  0
Debug 0
Timestamp (s) 1661795676.203578
Board variant: Wacc.1
Firmware version: Wacc.v0.2.0p1

```

In addition to AX, AY, and AZ we also see the `single_tap_count` value which reports back a count of the number of single-tap contacts the accelerometer has experienced since power-up. 

The following Wacc parameters configure the accelerometer low-pass filter and single-tap settings. See the [ADXL343](https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL343.pdf) datasheet for more details.

```{.bash .shell-prompt}
stretch_params.py | grep wacc
```
```{.bash .no-copy}
stretch_body.robot_params.nominal_params      param.wacc.config.accel_LPF         10.0                          
stretch_body.robot_params.nominal_params      param.wacc.config.accel_range_g        4                             
stretch_body.robot_params.nominal_params      param.wacc.config.accel_single_tap_dur       70                            
stretch_body.robot_params.nominal_params      param.wacc.config.accel_single_tap_thresh    50                                     
stretch_configuration_params.yaml              param.wacc.config.accel_gravity_scale      1.0 
```

## Cliff Sensors

Stretch has four Sharp GP2Y0A51SK0F IR cliff sensors pointed toward the floor. These report the distance to the floor, allowing for the detection of thresholds, stair edges, etc. 

Relevant parameters for the cliff sensors are:

```{.bash .shell-prompt}
stretch_params.py | grep cliff
```
```{.bash .no-copy}
stretch_body.robot_params.nominal_params  param.pimu.config.cliff_LPF           10.0                          
stretch_body.robot_params.nominal_params  param.pimu.config.cliff_thresh        -50                           
stretch_body.robot_params.nominal_params  param.pimu.config.stop_at_cliff        0                           
stretch_configuration_params.yaml         param.pimu.config.cliff_zero          [518.6307312011719, 530.9835095214844, 500.7268048095703, 509.92264434814456]         
stretch_body.robot_params.nominal_params  param.robot_monitor.monitor_base_cliff_event   1  
```

The sensors are calibrated such that a zero value (as defined by `cliff_zero`) indicates the sensor is at the correct height from the floor surface. A negative value indicates a drop off such as a stair ledge while a positive value indicates an obstacle like a threshold or high pile carpet.  You may want to recalibrate this zero based on the surface the robot is on (eg, carpet, tile, etc). To do this:

```{.bash .shell-prompt}
REx_cliff_sensor_calibrate.py 
```
```{.bash .no-copy}
For use with S T R E T C H (R) RESEARCH EDITION from Hello Robot Inc.
---------------------------------------------------------------------

Ensure cliff sensors are not obstructed and base is on a flat surface
Hit enter when ready
Itr 0 Val [518.630126953125, 530.4168701171875, 500.863037109375, 510.0032958984375]
...
Itr 99 Val [518.8374633789062, 530.858154296875, 500.5805969238281, 509.9013671875]
Got cliff zeros of:  [518.6307312011719, 530.9835095214844, 500.7268048095703, 509.92264434814456]
Calibration passed. Storing to YAML...
```

The  `stop_at_cliff` field causes the robot to execute a Runstop when the cliff sensor readings exceed the value `cliff_thresh`. The parameter `cliff_LPF` defines the low-pass-filter rate (Hz) on the analog sensor readings.

!!! note
    As configured at the factory,  `stop_at_cliff` is set to zero and Stretch does not stop its motion based on the cliff sensor readings. Hello Robot makes no guarantees as to the reliability of Stretch's ability to avoid driving over ledges and stairs when this flag is enabled.

The range values from the sensors can be read from the `robot.pimu.status` message. The relevant fields are:

```python
import stretch_body.robot
r=stretch_body.robot.Robot()
r.startup()

r.pimu.status['cliff_range']
Out[4]: [0.39227294921875, -0.2047119140625, -0.26422119140625, 0.006134033203125]

r.pimu.status['at_cliff']
Out[5]: [False, False, False, False]

r.pimu.status['cliff_event']
Out[5]: False

```

The `cliff_event` flag is set when any of the four sensor readings exceed `cliff_thresh` and `stop_at_cliff` is enabled. In the event of a Cliff Event, it must be reset by `robot.pimu.cliff_event_reset()` to reset the generated Runstop.

The cliff detection logic can be found in the [Pimu firmware](https://github.com/hello-robot/stretch_firmware/blob/master/arduino/hello_pimu/Pimu.cpp).

------
<div align="center"> All materials are Copyright 2022 by Hello Robot Inc. Hello Robot and Stretch are registered trademarks.</div>