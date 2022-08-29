## Sensors

### Base IMU

The base has a 9 DOF IMU using the 9 DOF FXOS8700 + FXAS21002 chipset. This is the same chipset as used on the [Adafruit NXP IMU board](https://www.adafruit.com/product/3463). 

The Pimu reports back the IMU sensor readings

### Wrist Accelerometer

Coming soon.

### Cliff Sensors

Stretch has [four IR cliff sensors](https://docs.hello-robot.com/hardware_user_guide/#base) pointed towards the floor. These report the distance to the floor, allowing for detection of thresholds, stair edges, etc. 

Relevant parameters in the factory YAML are

```yaml
pimu:
  config:
    cliff_LPF: 10.0
    cliff_thresh: -50
    cliff_zero:
    - 523.7940936279297
    - 508.10246490478517
    - 496.55742706298827
    - 525.149652709961
    stop_at_cliff: 0
```

The  `stop_at_cliff` field causes the robot to execute a Runstop when the cliff sensor readings are out of bounds. 

**Note: As configured at the factory,  `stop_at_cliff` is set to zero and Stretch does not stop its motion based on the cliff sensor readings. Hello Robot makes no guarantees as to the reliability of Stretch's ability to avoid driving over ledges and stairs when this flag is enabled.**

The sensors are calibrated such that a zero value indicates the sensor is at the correct height from the floor surface. A negative value indicates a drop off such as a stair ledge while a positive value indicates an obstacle like a threshold or high pile carpet.

The calibrated range values from the sensors can be read from the `robot.pimu.status` message. Relevant fields are:

```python

In [1]: robot.pimu.pretty_print()
------ Pimu -----
...
At Cliff [False, False, False, False]
Cliff Range [2.043212890625, 3.710906982421875, 1.6026611328125, 1.95098876953125]
Cliff Event False
...
```

A Cliff Event flag is set when any of the four sensor readings exceed `cliff_thresh` and `stop_at_cliff` is enabled. In the event of a Cliff Event, it must be reset by `robot.pimu.cliff_event_reset()`in order to reset the generated Runstop.

The cliff detection logic can be found in the [Pimu firmware](https://github.com/hello-robot/stretch_firmware/blob/master/arduino/hello_pimu/Pimu.cpp).

