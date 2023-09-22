## Example 9

This example aims to combine the [ReSpeaker Microphone Array](respeaker_mic_array.md) and [Follow Joint Trajectory](follow_joint_trajectory.md) tutorials to voice teleoperate the mobile base of the Stretch robot.

Begin by running the following command in a new terminal.

```{.bash .shell-prompt}
ros2 launch stretch_core stretch_driver.launch.py
```

Then run the `respeaker.launch.py` file. In a new terminal, execute:

```{.bash .shell-prompt}
ros2 launch respeaker_ros2 respeaker.launch.py
```

Then run the [voice_teleoperation_base.py](https://github.com/hello-robot/stretch_tutorials/blob/humble/stretch_ros_tutorials/voice_teleoperation_base.py) node in a new terminal.

```{.bash .shell-prompt}
cd ament_ws/src/stretch_tutorials/stretch_ros_tutorials/
python3 voice_teleoperation_base.py
```

In terminal 3, a menu of voice commands is printed. You can reference this menu layout below.  

```{.bash .no-copy}
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

To stop the node from sending twist messages, type `Ctrl` + `c` or say "**quit**".


### The Code

```python
#!/usr/bin/env python3

# Import modules
import math
import rclpy
import sys
from rclpy.duration import Duration

# We're going to subscribe to 64-bit integers, so we need to import the definition
# for them
from sensor_msgs.msg import JointState

# Import Int32 message typs from the std_msgs package
from std_msgs.msg import Int32

# Import the FollowJointTrajectory from the control_msgs.msg package to
# control the Stretch robot
from control_msgs.action import FollowJointTrajectory

# Import JointTrajectoryPoint from the trajectory_msgs package to define
# robot trajectories
from trajectory_msgs.msg import JointTrajectoryPoint

# Import hello_misc script for handling trajectory goals with an action client
import hello_helpers.hello_misc as hm

# Import SpeechRecognitionCandidates from the speech_recognition_msgs package
from speech_recognition_msgs.msg import SpeechRecognitionCandidates

class GetVoiceCommands:
    def __init__(self, node):
        self.node = node
        # Set step size as medium by default
        self.step_size = 'medium'
        self.rad_per_deg = math.pi/180.0

        # Small step size parameters
        self.small_deg = 5.0
        self.small_rad = self.rad_per_deg * self.small_deg
        self.small_translate = 0.025

        # Medium step size parameters
        self.medium_deg = 10.0
        self.medium_rad = self.rad_per_deg * self.medium_deg
        self.medium_translate = 0.05

        # Big step size parameters
        self.big_deg = 20.0
        self.big_rad = self.rad_per_deg * self.big_deg
        self.big_translate = 0.1

        # Initialize the voice command
        self.voice_command = None

        # Initialize the sound direction
        self.sound_direction = 0

        # Initialize subscribers
        self.speech_to_text_sub = self.node.create_subscription(SpeechRecognitionCandidates, "/speech_to_text", self.callback_speech, 1)
        self.sound_direction_sub = self.node.create_subscription(Int32, "/sound_direction", self.callback_direction, 1)

    def callback_direction(self, msg):
        self.sound_direction = msg.data * -self.rad_per_deg

    def callback_speech(self,msg):
        self.voice_command = ' '.join(map(str,msg.transcript))

    def get_inc(self):
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
        command = None
        # Move base forward command
        if self.voice_command == 'forward':
            command = {'joint': 'translate_mobile_base', 'inc': self.get_inc()['translate']}

        # Move base back command
        if self.voice_command == 'back':
            command = {'joint': 'translate_mobile_base', 'inc': -self.get_inc()['translate']}

        # Rotate base left command
        if self.voice_command == 'left':
            command = {'joint': 'rotate_mobile_base', 'inc': self.get_inc()['rad']}

        # Rotate base right command
        if self.voice_command == 'right':
            command = {'joint': 'rotate_mobile_base', 'inc': -self.get_inc()['rad']}

        # Move base to sound direction command
        if self.voice_command == 'stretch':
            command = {'joint': 'rotate_mobile_base', 'inc': self.sound_direction}

        # Set the step size of translational and rotational base motions
        if (self.voice_command == "small") or (self.voice_command == "medium") or (self.voice_command == "big"):
            self.step_size = self.voice_command
            self.node.get_logger().info('Step size = {0}'.format(self.step_size))

        if self.voice_command == 'quit':
            # Sends a signal to ros to shutdown the ROS interfaces
            self.node.get_logger().info("done")

            # Exit the Python interpreter
            sys.exit(0)

        # Reset voice command to None
        self.voice_command = None

        # return the updated command
        return command


class VoiceTeleopNode(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
        self.rate = 10.0
        self.joint_state = None
        hm.HelloNode.main(self, 'voice_teleop', 'voice_teleop', wait_for_first_pointcloud=False)
        self.speech = GetVoiceCommands(self)


    def joint_states_callback(self, msg):
        self.joint_state = msg

    def send_command(self, command):
        joint_state = self.joint_state
        # Conditional statement to send  joint trajectory goals
        if (joint_state is not None) and (command is not None):
            # Assign point as a JointTrajectoryPoint message type
            point = JointTrajectoryPoint()
            point.time_from_start = Duration(seconds=0).to_msg()

            # Assign trajectory_goal as a FollowJointTrajectoryGoal message type
            trajectory_goal = FollowJointTrajectory.Goal()
            trajectory_goal.goal_time_tolerance = Duration(seconds=0).to_msg()

            # Extract the joint name from the command dictionary
            joint_name = command['joint']
            trajectory_goal.trajectory.joint_names = [joint_name]

            # Extract the increment type from the command dictionary
            inc = command['inc']
            self.get_logger().info('inc = {0}'.format(inc))
            new_value = inc

            # Assign the new_value position to the trajectory goal message type
            point.positions = [new_value]
            trajectory_goal.trajectory.points = [point]
            trajectory_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
            self.get_logger().info('joint_name = {0}, trajectory_goal = {1}'.format(joint_name, trajectory_goal))

            # Make the action call and send goal of the new joint position
            self.trajectory_client.send_goal(trajectory_goal)
            self.get_logger().info('Done sending command.')

            # Reprint the voice command menu
            self.speech.print_commands()

    def timer_get_command(self):
        # Get voice command
            command = self.speech.get_command()

            # Send voice command for joint trajectory goals
            self.send_command(command)
    def main(self):
        self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 1)
        rate = self.create_rate(self.rate)
        self.speech.print_commands()
            
        self.sleep = self.create_timer(1/self.rate, self.timer_get_command)

def main(args=None):
    try:
        #rclpy.init(args=args)
        node = VoiceTeleopNode()
        node.main()
        node.new_thread.join()
    except KeyboardInterrupt:
        node.get_logger().info('interrupt received, so shutting down')
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()
```

### The Code Explained
This code is similar to that of the [multipoint_command](https://github.com/hello-robot/stretch_tutorials/blob/humble/stretch_ros_tutorials/multipoint_command.py) and [joint_state_printer](https://github.com/hello-robot/stretch_tutorials/blob/humble/stretch_ros_tutorials/joint_state_printer.py) node. Therefore, this example will highlight sections that are different from those tutorials. Now let's break the code down.

```python
#!/usr/bin/env python3
```

Every Python ROS [Node](http://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html) will have this declaration at the top. The first line makes sure your script is executed as a Python script.

```python
import math
import rclpy
import sys
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import hello_helpers.hello_misc as hm
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
```

You need to import `rclpy` if you are writing a ROS Node. Import the `FollowJointTrajectory` from the `control_msgs.action` package to control the Stretch robot. Import `JointTrajectoryPoint` from the `trajectory_msgs` package to define robot trajectories. The `hello_helpers` package consists of a module that provides various Python scripts used across stretch_ros. In this instance, we are importing the `hello_misc` script.  Import `sensor_msgs.msg` so that we can subscribe to JointState messages.

```python
class GetVoiceCommands:
```

Create a class that subscribes to the `speech-to-text` recognition messages, prints a voice command menu, and defines step size for translational and rotational mobile base motion.

```python
self.node = node
self.step_size = 'medium'
self.rad_per_deg = math.pi/180.0
```

Set the default step size as medium and create a float value, `self.rad_per_deg`, to convert degrees to radians.

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
self.speech_to_text_sub = self.node.create_subscription(SpeechRecognitionCandidates, "/speech_to_text", self.callback_speech, 1)
self.sound_direction_sub = self.node.create_subscription(Int32, "/sound_direction", self.callback_direction, 1)
```

Initialize the voice command and sound direction to values that will not result in moving the base.

Set up two subscribers.  The first one subscribes to the topic `/speech_to_text`, looking for `SpeechRecognitionCandidates` messages.  When a message comes in, ROS is going to pass it to the function `callback_speech` automatically. The second subscribes to `/sound_direction` message and passes it to the `callback_direction` function.

```python
def callback_direction(self, msg):
    self.sound_direction = msg.data * -self.rad_per_deg
```

The `callback_direction` function converts the `sound_direction` topic from degrees to radians.

```python
if self.step_size == 'small':
    inc = {'rad': self.small_rad, 'translate': self.small_translate}
if self.step_size == 'medium':
    inc = {'rad': self.medium_rad, 'translate': self.medium_translate}
if self.step_size == 'big':
    inc = {'rad': self.big_rad, 'translate': self.big_translate}
return inc
```

The `callback_speech` stores the increment size for translational and rotational base motion in `inc`. The increment size is contingent on the `self.step_size` string value.

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

In the `get_command()` function, the `command` is initialized as `None`, or set as a dictionary where the `joint` and `inc` values are stored. The `command` message type is dependent on the `self.voice_command` string value.

```python
if (self.voice_command == "small") or (self.voice_command == "medium") or (self.voice_command == "big"):
    self.step_size = self.voice_command
    self.node.get_logger().info('Step size = {0}'.format(self.step_size))
```

Based on the `self.voice_command` value set the step size for the increments.

```python
if self.voice_command == 'quit':
    self.node.get_logger().info("done")
    sys.exit(0)
```

If the `self.voice_command` is equal to `quit`, then initiate a clean shutdown of ROS and exit the Python interpreter.

```python
class VoiceTeleopNode(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
        self.rate = 10.0
        self.joint_state = None
        hm.HelloNode.main(self, 'voice_teleop', 'voice_teleop', wait_for_first_pointcloud=False)
        self.speech = GetVoiceCommands(self)
```

A class that inherits the `HelloNode` class from `hm` declares object from the `GetVoiceCommands` class and sends joint trajectory commands. The main function instantiates the `HelloNode` class.

```python
def send_command(self, command):
    joint_state = self.joint_state
    if (joint_state is not None) and (command is not None):
        point = JointTrajectoryPoint()
        point.time_from_start = Duration(seconds=0).to_msg()
```

The `send_command` function stores the joint state message and uses a conditional statement to send joint trajectory goals. Also, assign `point` as a `JointTrajectoryPoint` message type.

```python
trajectory_goal = FollowJointTrajectory.Goal()
trajectory_goal.goal_time_tolerance = Duration(seconds=0).to_msg()
```

Assign `trajectory_goal` as a `FollowJointTrajectory.Goal` message type.

```python
joint_name = command['joint']
trajectory_goal.trajectory.joint_names = [joint_name]
```

Extract the joint name from the command dictionary.

```python
inc = command['inc']
self.get_logger().info('inc = {0}'.format(inc))
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
self.get_logger().info('Done sending command.')
```

Make the action call and send the goal of the new joint position.

```python
self.speech.print_commands()
```

Reprint the voice command menu after the trajectory goal is sent.

```python
def main(self):
      self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 1)
      rate = self.create_rate(self.rate)
      self.speech.print_commands()
```

The main function initializes the subscriber and we are going to use the publishing rate that we set before.

```python
try:
    #rclpy.init(args=args)
    node = VoiceTeleopNode()
    node.main()
    node.new_thread.join()
except KeyboardInterrupt:
    node.get_logger().info('interrupt received, so shutting down')
    node.destroy_node()
    rclpy.shutdown()
```

Declare a `VoiceTeleopNode` object. Then execute the `main()` method.