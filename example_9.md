## Example 9

The aim of example 9 is to combine the [ReSpeaker Microphone Array](respeaker_microphone_array.md) and [Follow Joint Trajectory](follow_joint_trajectory.md) tutorials to voice teleoperate the mobile base of the Stretch robot.

Begin by running the following command in a new terminal.

```bash
# Terminal 1
roslaunch stretch_core stretch_driver.launch
```
Switch the mode to *position* mode using a rosservice call. Then run the `respeaker.launch`.

```bash
# Terminal 2
rosservice call /switch_to_position_mode
roslaunch stretch_core respeaker.launch
```
Then run the voice teleoperation base node in a new terminal.

```bash
# Terminal 3
cd catkin_ws/src/stretch_tutorials/src/
python3 voice_teleoperation_base.py
```
In terminal 3, a menu of voice commands is printed. You can reference this menu layout below.  

```

------------ VOICE TELEOP MENU ------------

VOICE COMMANDS              
"forward": BASE FORWARD                   
"back"   : BASE BACK                      
"left"   : BASE ROTATE LEFT               
"right"  : BASE ROTATE RIGHT              
"stretch": BASE ROTATES TOWARDS SOUND     

STEP SIZE                 
"big"    : BIG                            
"medium" : MEDIUM                         
"small"  : SMALL                          


"quit"   : QUIT AND CLOSE NODE            

-------------------------------------------

```
To stop the node from sending twist messages, type **Ctrl** + **c** or say "**quit**".


### The Code
```python
#!/usr/bin/env python3

import math
import rospy
import sys

from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import hello_helpers.hello_misc as hm
from speech_recognition_msgs.msg import SpeechRecognitionCandidates

class GetVoiceCommands:
    """
    A class that subscribes to the speech to text recognition messages, prints
    a voice command menu, and defines step size for translational and rotational
    mobile base motion.
    """
    def __init__(self):
        """
        A function that initializes subscribers and defines the three different
        step sizes.
        :param self: The self reference.
        """
        self.step_size = 'medium'
        self.rad_per_deg = math.pi/180.0

        self.small_deg = 5.0
        self.small_rad = self.rad_per_deg * self.small_deg
        self.small_translate = 0.025

        self.medium_deg = 10.0
        self.medium_rad = self.rad_per_deg * self.medium_deg
        self.medium_translate = 0.05

        self.big_deg = 20.0
        self.big_rad = self.rad_per_deg * self.big_deg
        self.big_translate = 0.1

        self.voice_command = None
        self.sound_direction = 0
        self.speech_to_text_sub  = rospy.Subscriber("/speech_to_text",  SpeechRecognitionCandidates, self.callback_speech)
        self.sound_direction_sub = rospy.Subscriber("/sound_direction", Int32,                       self.callback_direction)

    def callback_direction(self, msg):
        """
        A callback function that converts the sound direction from degrees to radians.
        :param self: The self reference.
        :param msg: The Int32 message type.
        """
        self.sound_direction = msg.data * -self.rad_per_deg

    def callback_speech(self,msg):
        """
        A callback function that takes all items in the iterable list and join
        them into a single string.
        :param self: The self reference.
        :param msg: The SpeechRecognitionCandidates message type.
        """
        self.voice_command = ' '.join(map(str,msg.transcript))

    def get_inc(self):
        """
        A function that sets the increment size for translational and rotational
        base motion.
        :param self:The self reference.

        :returns inc: A dictionary type.
        """
        if self.step_size == 'small':
            inc = {'rad': self.small_rad, 'translate': self.small_translate}
        if self.step_size == 'medium':
            inc = {'rad': self.medium_rad, 'translate': self.medium_translate}
        if self.step_size == 'big':
            inc = {'rad': self.big_rad, 'translate': self.big_translate}
        return inc

    def print_commands(self):
        """
        A function that prints the voice teleoperation menu.
        :param self: The self reference.
        """
        print('                                           ')
        print('------------ VOICE TELEOP MENU ------------')
        print('                                           ')
        print('               VOICE COMMANDS              ')
        print(' "forward": BASE FORWARD                   ')
        print(' "back"   : BASE BACK                      ')
        print(' "left"   : BASE ROTATE LEFT               ')
        print(' "right"  : BASE ROTATE RIGHT              ')
        print(' "stretch": BASE ROTATES TOWARDS SOUND     ')
        print('                                           ')
        print('                 STEP SIZE                 ')
        print(' "big"    : BIG                            ')
        print(' "medium" : MEDIUM                         ')
        print(' "small"  : SMALL                          ')
        print('                                           ')
        print('                                           ')
        print(' "quit"   : QUIT AND CLOSE NODE            ')
        print('                                           ')
        print('-------------------------------------------')

    def get_command(self):
        """
        A function that defines the teleoperation command based on the voice command.
        :param self: The self reference.

        :returns command: A dictionary type.
        """
        command = None
        if self.voice_command == 'forward':
            command = {'joint': 'translate_mobile_base', 'inc': self.get_inc()['translate']}
        if self.voice_command == 'back':
            command = {'joint': 'translate_mobile_base', 'inc': -self.get_inc()['translate']}
        if self.voice_command == 'left':
            command = {'joint': 'rotate_mobile_base', 'inc': self.get_inc()['rad']}
        if self.voice_command == 'right':
            command = {'joint': 'rotate_mobile_base', 'inc': -self.get_inc()['rad']}
        if self.voice_command == 'stretch':
            command = {'joint': 'rotate_mobile_base', 'inc': self.sound_direction}
        if (self.voice_command == "small") or (self.voice_command == "medium") or (self.voice_command == "big"):
            self.step_size = self.voice_command
            rospy.loginfo('Step size = {0}'.format(self.step_size))
        if self.voice_command == 'quit':
            rospy.signal_shutdown("done")
            sys.exit(0)

        self.voice_command = None
        return command


class VoiceTeleopNode(hm.HelloNode):
    """
    A class that inherits the HelloNode class from hm and sends joint trajectory
    commands.
    """
    def __init__(self):
        """
        A function that declares object from the GetVoiceCommands class, instantiates
        the HelloNode class, and set the publishing rate.
        """
        hm.HelloNode.__init__(self)
        self.rate = 10.0
        self.joint_state = None
        self.speech = GetVoiceCommands()

    def joint_states_callback(self, msg):
        """
        A callback function that stores Stretch's joint states.
        :param self: The self reference.
        :param msg: The JointState message type.
        """
        self.joint_state = msg

    def send_command(self, command):
        """
        Function that makes an action call and sends base joint trajectory goals
        :param self: The self reference.
        :param command: A dictionary type.
        """
        joint_state = self.joint_state
        if (joint_state is not None) and (command is not None):
            point = JointTrajectoryPoint()
            point.time_from_start = rospy.Duration(0.0)
            trajectory_goal = FollowJointTrajectoryGoal()
            trajectory_goal.goal_time_tolerance = rospy.Time(1.0)
            joint_name = command['joint']
            trajectory_goal.trajectory.joint_names = [joint_name]

            inc = command['inc']
            rospy.loginfo('inc = {0}'.format(inc))
            new_value = inc

            point.positions = [new_value]
            trajectory_goal.trajectory.points = [point]
            trajectory_goal.trajectory.header.stamp = rospy.Time.now()
            rospy.loginfo('joint_name = {0}, trajectory_goal = {1}'.format(joint_name, trajectory_goal))
            self.trajectory_client.send_goal(trajectory_goal)
            rospy.loginfo('Done sending command.')
            self.speech.print_commands()

    def main(self):
        """
        The main function that instantiates the HelloNode class, initializes the subscriber,
        and call other methods in both the VoiceTeleopNode and GetVoiceCommands classes.
        :param self: The self reference.
        """
        hm.HelloNode.main(self, 'voice_teleop', 'voice_teleop', wait_for_first_pointcloud=False)
        rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)
        rate = rospy.Rate(self.rate)
        self.speech.print_commands()

        while not rospy.is_shutdown():
            command = self.speech.get_command()
            self.send_command(command)
            rate.sleep()

if __name__ == '__main__':
    try:
        node = VoiceTeleopNode()
        node.main()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')
```

### The Code Explained
This code is similar to that of the [multipoint_command](https://github.com/hello-robot/stretch_tutorials/blob/main/src/multipoint_command.py) and [joint_state_printer](https://github.com/hello-robot/stretch_tutorials/blob/main/src/joint_state_printer.py) node. Therefore, this example will highlight sections that are different from those tutorials. Now let's break the code down.

```python
#!/usr/bin/env python3
```
Every Python ROS [Node](http://wiki.ros.org/Nodes) will have this declaration at the top. The first line makes sure your script is executed as a Python script.

```python
import math
import rospy
import sys

from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import hello_helpers.hello_misc as hm
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
```
You need to import rospy if you are writing a ROS Node. Import the `FollowJointTrajectoryGoal` from the `control_msgs.msg` package to control the Stretch robot. Import `JointTrajectoryPoint` from the `trajectory_msgs` package to define robot trajectories. The `hello_helpers` package consists of a module that provides various Python scripts used across stretch_ros. In this instance, we are importing the `hello_misc` script.  Import `sensor_msgs.msg` so that we can subscribe to JointState messages.

```python
class GetVoiceCommands:
```
Create a class that subscribes to the speech to text recognition messages, prints a voice command menu, and defines step size for translational and rotational mobile base motion.

```python
self.step_size = 'medium'
self.rad_per_deg = math.pi/180.0
```
Set the default step size as medium and create a float value, *self.rad_per_deg*, to convert degrees to radians.

```python
self.small_deg = 5.0
self.small_rad = self.rad_per_deg * self.small_deg
self.small_translate = 0.025

self.medium_deg = 10.0
self.medium_rad = self.rad_per_deg * self.medium_deg
self.medium_translate = 0.05

self.big_deg = 20.0
self.big_rad = self.rad_per_deg * self.big_deg
self.big_translate = 0.1
```
Define the three rotation and translation step sizes.

```python
self.voice_command = None
self.sound_direction = 0
self.speech_to_text_sub  = rospy.Subscriber("/speech_to_text",  SpeechRecognitionCandidates, self.callback_speech)
self.sound_direction_sub = rospy.Subscriber("/sound_direction", Int32,                       self.callback_direction)
```
Initialize the voice command and sound direction to values that will not result in moving the base.

Set up two subscribers.  The first one subscribes to the topic */speech_to_text*, looking for `SpeechRecognitionCandidates` messages.  When a message comes in, ROS is going to pass it to the function `callback_speech` automatically. The second subscribes to */sound_direction* message and passes it to the `callback_direction` function.

```python
self.sound_direction = msg.data * -self.rad_per_deg
```
The `callback_direction` function converts the *sound_direction* topic from degrees to radians.

```python
if self.step_size == 'small':
    inc = {'rad': self.small_rad, 'translate': self.small_translate}
if self.step_size == 'medium':
    inc = {'rad': self.medium_rad, 'translate': self.medium_translate}
if self.step_size == 'big':
    inc = {'rad': self.big_rad, 'translate': self.big_translate}
return inc
```
The `callback_speech` stores the increment size for translational and rotational base motion to *inc*. The increment size is contingent on the *self.step_size* string value.

```python
command = None
if self.voice_command == 'forward':
    command = {'joint': 'translate_mobile_base', 'inc': self.get_inc()['translate']}
if self.voice_command == 'back':
    command = {'joint': 'translate_mobile_base', 'inc': -self.get_inc()['translate']}
if self.voice_command == 'left':
    command = {'joint': 'rotate_mobile_base', 'inc': self.get_inc()['rad']}
if self.voice_command == 'right':
    command = {'joint': 'rotate_mobile_base', 'inc': -self.get_inc()['rad']}
if self.voice_command == 'stretch':
    command = {'joint': 'rotate_mobile_base', 'inc': self.sound_direction}
```
In the `get_command()` function, the *command* is initialized as None, or set as a dictionary where the *joint* and *inc* values are stored. The *command* message type is dependent on the *self.voice_command* string value.

```python
if (self.voice_command == "small") or (self.voice_command == "medium") or (self.voice_command == "big"):
    self.step_size = self.voice_command
    rospy.loginfo('Step size = {0}'.format(self.step_size))
```
Based on the *self.voice_command* value, set the step size for the increments.

```python
if self.voice_command == 'quit':
    rospy.signal_shutdown("done")
    sys.exit(0)
```
If the *self.voice_command* is equal to "quit", then initiate a clean shutdown of ROS and exit the Python interpreter.

```python
class VoiceTeleopNode(hm.HelloNode):
    """
    A class that inherits the HelloNode class from hm and sends joint trajectory
    commands.
    """
    def __init__(self):
        """
        A function that declares object from the GetVoiceCommands class, instantiates
        the HelloNode class, and set the publishing rate.
        """
        hm.HelloNode.__init__(self)
        self.rate = 10.0
        self.joint_state = None
        self.speech = GetVoiceCommands()
```
A class that inherits the `HelloNode` class from `hm`, declares object from the `GetVoiceCommands` class, and sends joint trajectory commands.

```python
def send_command(self, command):
    """
    Function that makes an action call and sends base joint trajectory goals
    :param self: The self reference.
    :param command: A dictionary type.
    """
    joint_state = self.joint_state
    if (joint_state is not None) and (command is not None):
        point = JointTrajectoryPoint()
        point.time_from_start = rospy.Duration(0.0)
```
The `send_command` function stores the joint state message and uses a conditional statement to send joint trajectory goals. Also, assign *point* as a `JointTrajectoryPoint` message type.

```python
trajectory_goal = FollowJointTrajectoryGoal()
trajectory_goal.goal_time_tolerance = rospy.Time(1.0)
```
Assign *trajectory_goal* as a `FollowJointTrajectoryGoal` message type.

```python
joint_name = command['joint']
trajectory_goal.trajectory.joint_names = [joint_name]
```
Extract the joint name from the command dictionary.

```python
inc = command['inc']
rospy.loginfo('inc = {0}'.format(inc))
new_value = inc
```
Extract the increment type from the command dictionary.

```python
point.positions = [new_value]
trajectory_goal.trajectory.points = [point]
```
Assign the new value position to the trajectory goal message type.

```python
self.trajectory_client.send_goal(trajectory_goal)
rospy.loginfo('Done sending command.')
```
Make the action call and send goal of the new joint position.

```python
self.speech.print_commands()
```
Reprint the voice command menu after the trajectory goal is sent.

```python
def main(self):
      """
      The main function that instantiates the HelloNode class, initializes the subscriber,
      and call other methods in both the VoiceTeleopNode and GetVoiceCommands classes.
      :param self: The self reference.
      """
      hm.HelloNode.main(self, 'voice_teleop', 'voice_teleop', wait_for_first_pointcloud=False)
      rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)
      rate = rospy.Rate(self.rate)
      self.speech.print_commands()
```
The main function instantiates the `HelloNode` class, initializes the subscriber, and call other methods in both the `VoiceTeleopNode` and `GetVoiceCommands` classes.

```python
while not rospy.is_shutdown():
  command = self.speech.get_command()
  self.send_command(command)
  rate.sleep()
```
Run a while loop to continuously check speech commands and send those commands to execute an action.

```python
try:
  node = VoiceTeleopNode()
  node.main()
except KeyboardInterrupt:
  rospy.loginfo('interrupt received, so shutting down')
```
Declare a `VoiceTeleopNode` object. Then execute the `main()` method.

**Previous Example** [Voice to Text](example_8.md)
**Next Example** [Tf2 Broadcaster and Listener](example_10.md)
