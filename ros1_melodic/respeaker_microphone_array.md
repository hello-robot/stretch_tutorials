## ReSpeaker Microphone Array

For this tutorial, we will go over on a high level how to use Stretch's [ReSpeaker Mic Array v2.0](https://wiki.seeedstudio.com/ReSpeaker_Mic_Array_v2.0/).  



<p align="center">
  <img src="https://raw.githubusercontent.com/hello-robot/stretch_tutorials/main/images/respeaker.jpg"/>
</p>


### Stretch Body Package
In this tutorial's section we will use command line tools in the [Stretch_Body](https://github.com/hello-robot/stretch_body) package, a low level Python API for Stretch's hardware, to directly interact with the ReSpeaker.

Begin by typing the following command in a new terminal.

```bash
stretch_respeaker_test.py
```

The following will be displayed in your terminal
```bash
hello-robot@stretch-re1-1005:~$ stretch_respeaker_test.py
For use with S T R E T C H (TM) RESEARCH EDITION from Hello Robot Inc.


* waiting for audio...
* recording 3 seconds
* done
* playing audio
* done

```

The ReSpeaker Mico Array will wait until it hears audio loud enough to trigger its recording feature. Stretch will record audio for 3 seconds and then replay it through its speakers. This command line is a good method to see if the hardware is working correctly.

To stop the python script, type **Ctrl** + **c** in the terminal.

### ReSpeaker_ROS Package

A [ROS package for the ReSpeaker](https://index.ros.org/p/respeaker_ros/#melodic) is utilized for this tutorial's section.

Begin by running the `sample_respeaker.launch` file in a terminal.

```bash
# Terminal 1
roslaunch respeaker_ros sample_respeaker.launch
```
This will bring up the necessary nodes that will allow the ReSpeaker to implement a voice and sound interface with the robot.

Below are executables you can run and see the ReSpeaker results.

```bash
rostopic echo /sound_direction    # Result of Direction (in Radians) of Audio
rostopic echo /sound_localization # Result of Direction as Pose (Quaternion values)
rostopic echo /is_speeching       # Result of Voice Activity Detector
rostopic echo /audio              # Raw audio data
rostopic echo /speech_audio       # Raw audio data when there is speech
rostopic echo /speech_to_text     # Voice recognition
```

An example is when you run the `speech_to_text` executable and speak near the microphone array. In this instance, "hello robot" was said.

```bash
# Terminal 2
hello-robot@stretch-re1-1005:~$ rostopic echo /speech_to_text
transcript:
  - hello robot
confidence: []
---
```

You can also set various parameters via`dynamic_reconfigure` running the following command in a new terminal.

```bash
# Terminal 3
rosrun rqt_reconfigure rqt_reconfigure
```
