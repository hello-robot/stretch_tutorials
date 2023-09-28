# ReSpeaker Microphone Array
For this tutorial, we will get a high-level view of how to use Stretch's [ReSpeaker Mic Array v2.0](https://wiki.seeedstudio.com/ReSpeaker_Mic_Array_v2.0/).  

<p align="center">
  <img src="https://raw.githubusercontent.com/hello-robot/stretch_tutorials/noetic/images/respeaker.jpg"/>
</p>

## Stretch Body Package
In this section we will use command line tools in the [Stretch_Body](https://github.com/hello-robot/stretch_body) package, a low-level Python API for Stretch's hardware, to directly interact with the ReSpeaker.

Begin by typing the following command in a new terminal.

```{.bash .shell-prompt}
stretch_respeaker_test.py
```

The following will be displayed in your terminal:

```{.bash .no-copy}
For use with S T R E T C H (TM) RESEARCH EDITION from Hello Robot Inc.

* waiting for audio...
* recording 3 seconds
* done
* playing audio
* done
```

The ReSpeaker Mico Array will wait until it hears audio loud enough to trigger its recording feature. Stretch will record audio for 3 seconds and then replay it through its speakers. This command line is a good method to see if the hardware is working correctly.

To stop the python script, type `Ctrl` + `c` in the terminal.

## ReSpeaker_ROS Package
A [ROS package for the ReSpeaker](https://index.ros.org/p/respeaker_ros/#melodic) is utilized for this section.

Begin by running the `sample_respeaker.launch.py` file in a terminal.

```{.bash .shell-prompt}
ros2 launch respeaker_ros2 respeaker.launch.py
```

This will bring up the necessary nodes that will allow the ReSpeaker to implement a voice and sound interface with the robot.

## ReSpeaker Topics
Below are executables you can run to see the ReSpeaker results.

```{.bash .shell-prompt}
ros2 topic echo /sound_direction
ros2 topic echo /sound_localization
ros2 topic echo /is_speeching
ros2 topic echo /audio
ros2 topic echo /speech_audio
ros2 topic echo /speech_to_text
```

There's also another topic called `/status_led`, with this topic you can change the color of the LEDs in the ReSpeaker, you need to publish the desired color in the terminal using `ros2 topic pub`. We will explore this topics in the next tutorial.

You can also set various parameters via `dynamic_reconfigure` by running the following command in a new terminal.

```{.bash .shell-prompt}
ros2 run rqt_reconfigure rqt_reconfigure