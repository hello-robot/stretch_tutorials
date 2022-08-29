# Example 8

This example will showcase how to save the interpreted speech from Stretch's [ReSpeaker Mic Array v2.0](https://wiki.seeedstudio.com/ReSpeaker_Mic_Array_v2.0/) to a text file.

<p align="center">
  <img src="images/respeaker.jpg"/>
</p>

Begin by running the `respeaker.launch` file in a terminal.
```bash
# Terminal 1
roslaunch respeaker_ros sample_respeaker.launch
```
Then run the speech_text node.

```bash
# Terminal 2
cd catkin_ws/src/stretch_tutorials/src/
python3 speech_text.py
```
The ReSpeaker will be listening and will start to interpret speech and save the transcript to a text file.  To stop shutdown the node, type **Ctrl** + **c** in the terminal.

### The Code
```python
#!/usr/bin/env python3

import rospy
import os
from speech_recognition_msgs.msg import SpeechRecognitionCandidates

class SpeechText:
    """
    A class that saves the interpreted speech from the ReSpeaker Microphone Array to a text file.
    """
    def __init__(self):
        """
        Initialize subscriber and directory to save speech to text file.
        """
        self.sub = rospy.Subscriber("speech_to_text", SpeechRecognitionCandidates, self.callback)
        self.save_path = '/home/hello-robot/catkin_ws/src/stretch_tutorials/stored_data'
        rospy.loginfo("Listening to speech.")

    def callback(self,msg):
        """
        A callback function that receives the speech transcript and appends the
        transcript to a text file.
        :param self: The self reference.
        :param msg: The SpeechRecognitionCandidates message type.
        """
        transcript = ' '.join(map(str,msg.transcript))
        file_name = 'speech.txt'
        completeName = os.path.join(self.save_path, file_name)

        with open(completeName, "a+") as file_object:
            file_object.write("\n")
            file_object.write(transcript)

if __name__ == '__main__':
    rospy.init_node('speech_text')
    SpeechText()
    rospy.spin()
```

### The Code Explained
Now let's break the code down.

```python
#!/usr/bin/env python3
```
Every Python ROS [Node](http://wiki.ros.org/Nodes) will have this declaration at the top. The first line makes sure your script is executed as a Python3 script.

```python
import rospy
import os
```
You need to import `rospy` if you are writing a ROS [Node](http://wiki.ros.org/Nodes).

```python
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
```
Import `SpeechRecognitionCandidates` from the `speech_recgonition_msgs.msg` so that we can receive the interpreted speech.

```python
def __init__(self):
    """
    Initialize subscriber and directory to save speech to text file.
    """
    self.sub = rospy.Subscriber("speech_to_text", SpeechRecognitionCandidates, self.callback)
```
Set up a subscriber.  We're going to subscribe to the topic "*speech_to_text*", looking for `SpeechRecognitionCandidates` messages. When a message comes in, ROS is going to pass it to the function "callback" automatically.

```python
self.save_path = '/home/hello-robot/catkin_ws/src/stretch_tutorials/stored_data
```
Define the directory to save the text file.

```python
transcript = ' '.join(map(str,msg.transcript))
```
Take all items in the iterable list and join them into a single string named transcript.

```python
file_name = 'speech.txt'
completeName = os.path.join(self.save_path, file_name)
```
Define the file name and create a complete path directory.

```python
with open(completeName, "a+") as file_object:
    file_object.write("\n")
    file_object.write(transcript)
```
Append the transcript to the text file.

```python
rospy.init_node('speech_text')
SpeechText()

```
The next line, `rospy.init_node(NAME, ...)`, is very important as it tells rospy the name of your node -- until rospy has this information, it cannot start communicating with the ROS Master. **NOTE:** the name must be a base name, i.e. it cannot contain any slashes "/".

Instantiate the `SpeechText()` class.

```python
rospy.spin()
```
Give control to ROS.  This will allow the callback to be called whenever new
messages come in.  If we don't put this line in, then the node will not work,
and ROS will not process any messages.

**Previous Example** [Capture Image](example_7.md)
**Next Example** [Voice Teleoperation of Base](example_9.md)
