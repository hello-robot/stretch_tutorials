## Example 8

This example will showcase how to save the interpreted speech from Stretch's [ReSpeaker Mic Array v2.0](https://wiki.seeedstudio.com/ReSpeaker_Mic_Array_v2.0/) to a text file.

<p align="center">
  <img src="https://raw.githubusercontent.com/hello-robot/stretch_tutorials/noetic/images/respeaker.jpg"/>
</p>

Begin by running the `respeaker.launch.py` file in a terminal.

```{.bash .shell-prompt}
ros2 launch respeaker_ros2 respeaker.launch.py
```
Then run the [speech_text.py](https://github.com/hello-robot/stretch_tutorials/blob/humble/stretch_ros_tutorials/speech_text.py) node. In a new terminal, execute:

```{.bash .shell-prompt}
cd ament_ws/src/stretch_tutorials/stretch_ros_tutorials/
python3 speech_text.py
```

The ReSpeaker will be listening and will start to interpret speech and save the transcript to a text file.  To shut down the node, type `Ctrl` + `c` in the terminal.

### The Code

```python
#!/usr/bin/env python3

# Import modules
import rclpy
import os
from rclpy.node import Node

# Import SpeechRecognitionCandidates from the speech_recognition_msgs package
from speech_recognition_msgs.msg import SpeechRecognitionCandidates

class SpeechText(Node):
    def __init__(self):
        super().__init__('stretch_speech_text')
        # Initialize subscriber
        self.sub = self.create_subscription(SpeechRecognitionCandidates, "speech_to_text", self.callback, 1)

        # Create path to save captured images to the stored data folder
        self.save_path = '/home/hello-robot/ament_ws/src/stretch_tutorials/stored_data'

        # Create log message
        self.get_logger().info("Listening to speech")

    def callback(self,msg):
        # Take all items in the iterable list and join them into a single string
        transcript = ' '.join(map(str,msg.transcript))

        # Define the file name and create a complete path name
        file_name = 'speech.txt'
        completeName = os.path.join(self.save_path, file_name)

        # Append 'hello' at the end of file
        with open(completeName, "a+") as file_object:
            file_object.write("\n")
            file_object.write(transcript)

def main(args=None):
    # Initialize the node and name it speech_text
    rclpy.init(args=args)

    # Instantiate the SpeechText class
    speech_txt = SpeechText()

    # Give control to ROS.  This will allow the callback to be called whenever new
    # messages come in.  If we don't put this line in, then the node will not work,
    # and ROS will not process any messages
    rclpy.spin(speech_txt)

if __name__ == '__main__':
    main()
```

### The Code Explained
Now let's break the code down.

```python
#!/usr/bin/env python3
```

Every Python ROS [Node](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html) will have this declaration at the top. The first line makes sure your script is executed as a Python3 script.

```python
import rclpy
import os
from rclpy.node import Node
```

You need to import rclpy if you are writing a ROS Node.

```python
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
```

Import `SpeechRecognitionCandidates` from the `speech_recgonition_msgs.msg` so that we can receive the interpreted speech.

```python
def __init__(self):
    super().__init__('stretch_speech_text')
    self.sub = self.create_subscription(SpeechRecognitionCandidates, "speech_to_text", self.callback, 1)
```

Set up a subscriber.  We're going to subscribe to the topic `speech_to_text`, looking for `SpeechRecognitionCandidates` messages. When a message comes in, ROS is going to pass it to the function "callback" automatically.

```python
self.save_path = '/home/hello-robot/ament_ws/src/stretch_tutorials/stored_data'
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
def main(args=None):
    rclpy.init(args=args)
    speech_txt = SpeechText()
```

The next line, rclpy.init() method initializes the node. In this case, your node will take on the name 'stretch_speech_text'. Instantiate the `SpeechText()` class.

!!! note
    The name must be a base name, i.e. it cannot contain any slashes "/".

```python
rclpy.spin(speech_txt)
```

Give control to ROS with `rclpy.spin()`. This will allow the callback to be called whenever new messages come in. If we don't put this line in, then the node will not work, and ROS will not process any messages.