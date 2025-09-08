## Example 5

In this example, we will review a Python script that prints out the positions of a selected group of Stretch joints. This script is helpful if you need the joint positions after you teleoperated Stretch with the Xbox controller or physically moved the robot to the desired configuration after hitting the run stop button.

If you are looking for a continuous print of the joint states while Stretch is in action, then you can use the [ros2 topic command-line tool](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html) shown in the [Internal State of Stretch Tutorial](https://github.com/hello-robot/stretch_tutorials/blob/master/ros2/internal_state_of_stretch.md).

Begin by starting up the stretch driver launch file.

```{.bash .shell-prompt}
ros2 launch stretch_core stretch_driver.launch.py
```

You can then hit the run-stop button (you should hear a beep and the LED light in the button blink) and move the robot's joints to a desired configuration. Once you are satisfied with the configuration, hold the run-stop button until you hear a beep. Then run the following command to execute the [joint_state_printer.py](https://github.com/hello-robot/stretch_tutorials/blob/humble/stretch_ros_tutorials/joint_state_printer.py) which will print the joint positions of the lift, arm, and wrist. In a new terminal, execute:

```{.bash .shell-prompt}
cd ament_ws/src/stretch_tutorials/stretch_ros_tutorials/
python3 joint_state_printer.py
```

Your terminal will output the `position` information of the previously mentioned joints shown below.
```{.bash .no-copy}
name: ['joint_lift', 'wrist_extension', 'joint_wrist_yaw']
position: [0.6043133175850597, 0.19873586673129257, 0.017257283863713464]
```

!!! note
	Stretch's arm has four prismatic joints and the sum of their positions gives the *wrist_extension* distance. The *wrist_extension* is needed when sending [joint trajectory commands](https://github.com/hello-robot/stretch_tutorials/blob/master/ros2/follow_joint_trajectory.md) to the robot. Further, you can not actuate an individual arm joint. Here is an image of the arm joints for reference:

<p align="center">
  <img src="https://raw.githubusercontent.com/hello-robot/stretch_tutorials/noetic/images/joints.png"/>
</p>

### The Code

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sys
import time

# We're going to subscribe to a JointState message type, so we need to import
# the definition for it
from sensor_msgs.msg import JointState

class JointStatePublisher(Node):
	"""
	A class that prints the positions of desired joints in Stretch.
	"""

	def __init__(self):
		"""
		Function that initializes the subscriber.
		:param self: The self reference
		"""
		super().__init__('stretch_joint_state')
		
        # Set up a subscriber. We're going to subscribe to the topic "joint_states"
		self.sub = self.create_subscription(JointState, 'joint_states', self.callback, 1)


	def callback(self, msg):
		"""
		Callback function to deal with the incoming JointState messages.
		:param self: The self reference.
		:param msg: The JointState message.
		"""
		# Store the joint messages for later use
		self.get_logger().info('Receiving JointState messages')
		self.joint_states = msg


	def print_states(self, joints):
		"""
		print_states function to deal with the incoming JointState messages.
		:param self: The self reference.
		:param joints: A list of string values of joint names.
		"""
		# Create an empty list that will store the positions of the requested joints
		joint_positions = []

		# Use of forloop to parse the names of the requested joints list.
		# The index() function returns the index at the first occurrence of
		# the name of the requested joint in the self.joint_states.name list
		for joint in joints:
			if joint == "wrist_extension":
				index = self.joint_states.name.index('joint_arm_l0')
				joint_positions.append(4*self.joint_states.position[index])
				continue
			
			index = self.joint_states.name.index(joint)
			joint_positions.append(self.joint_states.position[index])

		# Print the joint position values to the terminal
		print("name: " + str(joints))
		print("position: " + str(joint_positions))

		# Sends a signal to rclpy to shutdown the ROS interfaces
		rclpy.shutdown()

		# Exit the Python interpreter
		sys.exit(0)

def main(args=None):
	# Initialize the node
    rclpy.init(args=args)
    joint_publisher = JointStatePublisher()
    time.sleep(1)
    rclpy.spin_once(joint_publisher)

	# Create a list of the joints and name them joints. These will be an argument
	# for the print_states() function
    joints = ["joint_lift", "wrist_extension", "joint_wrist_yaw"]
    joint_publisher.print_states(joints)
    
    # Give control to ROS.  This will allow the callback to be called whenever new
	# messages come in.  If we don't put this line in, then the node will not work,
	# and ROS will not process any messages
    rclpy.spin(joint_publisher)


if __name__ == '__main__':
	main()
```

### The Code Explained
Now let's break the code down.

```python
#!/usr/bin/env python3
```

Every Python ROS [Node](http://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html) will have this declaration at the top. The first line makes sure your script is executed as a Python3 script.

```python
import rclpy
import sys
import time
from rclpy.node import Node
from sensor_msgs.msg import JointState
```

You need to import rclpy if you are writing a ROS 2 Node. Import `sensor_msgs.msg` so that we can subscribe to `JointState` messages.

```python
self.sub = self.create_subscription(JointState, 'joint_states', self.callback, 1)
```

Set up a subscriber.  We're going to subscribe to the topic `joint_states`, looking for `JointState` messages.  When a message comes in, ROS is going to pass it to the function "callback" automatically

```python
def callback(self, msg):
	self.joint_states = msg
```

This is the callback function where the `JointState` messages are stored as `self.joint_states`. Further information about this message type can be found here: [JointState Message](http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/JointState.html)

```python
def print_states(self, joints):
	joint_positions = []
```

This is the `print_states()` function which takes in a list of joints of interest as its argument. the is also an empty list set as `joint_positions` and this is where the positions of the requested joints will be appended.

```python
for joint in joints:
  if joint == "wrist_extension":
    index = self.joint_states.name.index('joint_arm_l0')
    joint_positions.append(4*self.joint_states.position[index])
    continue
  index = self.joint_states.name.index(joint)
  joint_positions.append(self.joint_states.position[index])
```

In this section of the code, a for loop is used to parse the names of the requested joints from the `self.joint_states` list. The `index()` function returns the index of the name of the requested joint and appends the respective position to the `joint_positions` list.

```python
rclpy.shutdown()
sys.exit(0)
```

The first line of code initiates a clean shutdown of ROS. The second line of code exits the Python interpreter.

```python
rclpy.init(args=args)
joint_publisher = JointStatePublisher()
time.sleep(1)
```

The next line, rclpy.init_node initializes the node. In this case, your node will take on the name 'stretch_joint_state'. **NOTE:** the name must be a base name, i.e. it cannot contain any slashes "/".

Declare object, *joint_publisher*, from the `JointStatePublisher` class.

The use of the `time.sleep()` function is to allow the *joint_publisher* class to initialize all of its features before requesting to publish joint positions of desired joints (running the `print_states()` method).

```python
joints = ["joint_lift", "wrist_extension", "joint_wrist_yaw"]
#joints = ["joint_head_pan","joint_head_tilt", joint_gripper_finger_left", "joint_gripper_finger_right"]
joint_publisher.print_states(joints)
```

Create a list of the desired joints that you want positions to be printed. Then use that list as an argument for the `print_states()` method.

```python
rclpy.spin(joint_publisher)
```

Give control to ROS. This will allow the callback to be called whenever new messages come in. If we don't put this line in, then the node will not work, and ROS will not process any messages.