## Respeaker Microphone Array

For this tutorial, we will go over on a high level how to use Stretch's [Respeaker Mic Array v2.0](https://wiki.seeedstudio.com/ReSpeaker_Mic_Array_v2.0/).  

<p align="center">
  <img src="https://raw.githubusercontent.com/hello-robot/stretch_tutorials/noetic/images/respeaker.jpg"/>
</p>


### Stretch Respeaker Tool

We'll begin by using the command-line Stretch Respeaker Tool to quickly try out the robot's microphone array and speakers. This tool doesn't use ROS, but we'll cover how to use the Respeaker from ROS in the next section. Type the following command in a new terminal:

```bash
stretch_respeaker_test.py
```

The following will be outputted in your terminal:

```bash
hello-robot@stretch-re1-1005:~$ stretch_respeaker_test.py
For use with S T R E T C H (TM) RESEARCH EDITION from Hello Robot Inc.


* waiting for audio...
* recording 3 seconds
* done
* playing audio
* done

* waiting for audio...
```

The Stretch Respeaker tool will wait until it hears audio loud enough to trigger VAD (voice activity detection). Then, the tool will record audio for 3 seconds, replay it through its speakers, and go back to waiting for audio. This tool is a good way to confirm the hardware is working correctly.

To stop the tool, type **Ctrl** + **C** in the terminal.

### Respeaker ROS Package

#### Prerequisite

Before getting started with the Respeaker ROS package, we'll confirm it is available to use. Type the following into a new terminal:

```bash
rospack find respeaker_ros
```

If you get an error like `[rospack] Error: package 'respeaker_ros' not found`, refresh your ROS workspace using the `stretch_catkin_refresh.sh` tool.

#### Getting started

Run the `respeaker.launch` file in a new terminal using:

```bash
roslaunch respeaker_ros respeaker.launch
```
This will bring up the necessary ROS nodes to interface with the robot's microphone array and speakers. After initialization, you will see the following outputted in your terminal:

```bash
[INFO] [1672818306.618280]: Initializing Respeaker device (takes 10 seconds)
[INFO] [1672818317.082498]: Respeaker device initialized (Version: 16)
[INFO] [1672818317.521955]: Found 6: ReSpeaker 4 Mic Array (UAC1.0): USB Audio (hw:1,0) (channels: 6)
[INFO] [1672818317.526263]: Using channels range(0, 6)
```

Explore some of the notable ROS topics being published using the following commands: 

```bash
rostopic echo /audio              # Raw audio data
rostopic echo /speech_audio       # Raw audio data when there is speech
rostopic echo /sound_direction    # Direction (in Degrees) of audio source
rostopic echo /sound_localization # Direction (in Quaternion part of SE3 pose) of audio source
rostopic echo /is_speeching       # Result of Voice Activity Detector (VAD)
rostopic echo /speech_to_text     # Voice recognition
```

#### Speech Recognition

In particular, the `/speech_to_text` topic can be helpful for prototyping voice command based programs. In a new terminal, run `rostopic echo /speech_to_text`. Then, stand near the robot and say "hello robot". You will see something like this outputted to the terminal:

```bash
transcript:
  - hello robot
confidence: [0.9876290559768677]
```

A internet connection is required since the audio is being transcribed by the Google Speech API. To switch to using offline transciption, you can swap the speech recognizer used in the [speech_to_text node](https://github.com/hello-binit/respeaker_ros/blob/4880cd50cb4b2272ca3221680632bdcf7390a978/scripts/speech_to_text.py#L73-L74). See the [SpeechRecognition docs](https://pypi.org/project/SpeechRecognition/) for other available speech recognizers.

#### Direction of Arrival

The `/sound_direction` and `/sound_localization` topics give the DOA, or direction of arrival, in degrees and quaternion respectively. New messages are published when the DOA changes. And the 12 Respeaker LEDs show the DOA coarsely by coloring the LED closest to the DOA teal, while the rest of the LEDs show a darker blue color. In a new terminal, run `rostopic echo /sound_direction` or `rostopic echo /sound_localization`. Then, stand in front of the robot and say anything. You will see something like this outputted to the terminal:

```bash
# /sound_direction
data: 0

# or /sound_localization
header:
  seq: 1
  stamp:
    secs: 1672863834
    nsecs: 646940469
  frame_id: "respeaker_base"
pose:
  position:
    x: 0.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 1.0
    w: 0.0
```

The DOA range in degrees is [-180, 180], and the diagram below shows the DOA range visually:

```
TODO
```

Due to echo-ing within the shell covering the Respeaker, the DOA can be inaccurate. If you are experiencing this issue, contact Hello Robot Support for instructions on removing the head shell, which will reduce echo. Furthermore, it's possible to get more accurate DOA with audio post-processing. The Respeaker calculates the DOA on-hardware, however audio streams from the four microphones are published, so it is possible to use real-time audio source localization software like [ODAS](https://wiki.seeedstudio.com/ReSpeaker_Mic_Array_v2.0/#realtime-sound-source-localization-and-tracking) to better determine the DOA.

#### Dynamic Reconfigure

You can also set various parameters via `dynamic_reconfigure`, by running the following command in a new terminal:

```bash
rosrun rqt_reconfigure rqt_reconfigure
```
