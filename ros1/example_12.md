# Example 12
For this example, we will send follow joint trajectory commands for the head camera to search and locate an ArUco tag. In this instance, a Stretch robot will try to locate the docking station's ArUco tag.

## Modifying Stretch Marker Dictionary YAML File
When defining the ArUco markers on Stretch, hello robot utilizes a YAML file, [stretch_marker_dict.yaml](https://github.com/hello-robot/stretch_ros/blob/master/stretch_core/config/stretch_marker_dict.yaml), that holds the information about the markers. A further breakdown of the YAML file can be found in our [Aruco Marker Detection](aruco_marker_detection.md) tutorial.

Below is what needs to be included in the [stretch_marker_dict.yaml](https://github.com/hello-robot/stretch_ros/blob/master/stretch_core/config/stretch_marker_dict.yaml) file so the [detect_aruco_markers](https://github.com/hello-robot/stretch_ros/blob/master/stretch_core/nodes/detect_aruco_markers) node can find the docking station's ArUco tag.

```yaml
'245':
  'length_mm': 88.0
  'use_rgb_only': False
  'name': 'docking_station'
  'link': None
```

## Getting Started
Begin by running the stretch driver launch file.

```{.bash .shell-prompt}
roslaunch stretch_core stretch_driver.launch
```

To activate the RealSense camera and publish topics to be visualized, run the following launch file in a new terminal.

```{.bash .shell-prompt}
roslaunch stretch_core d435i_high_resolution.launch
```

Next, run the stretch ArUco launch file which will bring up the [detect_aruco_markers](https://github.com/hello-robot/stretch_ros/blob/master/stretch_core/nodes/detect_aruco_markers) node. In a new terminal, execute:

```{.bash .shell-prompt}
roslaunch stretch_core stretch_aruco.launch
```

Within this tutorial package, there is an [RViz config file](https://github.com/hello-robot/stretch_tutorials/blob/noetic/rviz/aruco_detector_example.rviz) with the topics for the transform frames in the Display tree. You can visualize these topics and the robot model by running the command below in a new terminal.

```{.bash .shell-prompt}
rosrun rviz rviz -d /home/hello-robot/catkin_ws/src/stretch_tutorials/rviz/aruco_detector_example.rviz
```

Then run the [aruco_tag_locator.py](https://github.com/hello-robot/stretch_tutorials/blob/noetic/src/aruco_tag_locator.py) node. In a new terminal, execute:

```{.bash .shell-prompt}
cd catkin_ws/src/stretch_tutorials/src/
python3 aruco_tag_locator.py
```

<p align="center">
  <img src="https://raw.githubusercontent.com/hello-robot/stretch_tutorials/noetic/images/aruco_locator.gif"/>
</p>

### The Code

```python
#! /usr/bin/env python3

import rospy
import time
import tf2_ros
import numpy as np
from math import pi

import hello_helpers.hello_misc as hm
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import TransformStamped

class LocateArUcoTag(hm.HelloNode):
    """
    A class that actuates the RealSense camera to find the docking station's
    ArUco tag and returns a Transform between the `base_link` and the requested tag.
    """
    def __init__(self):
        """
        A function that initializes the subscriber and other needed variables.
        :param self: The self reference.
        """
        hm.HelloNode.__init__(self)

        self.joint_states_sub = rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)
        self.transform_pub = rospy.Publisher('ArUco_transform', TransformStamped, queue_size=10)

        self.joint_state = None

        self.min_pan_position = -4.10
        self.max_pan_position =  1.50
        self.pan_num_steps = 10
        self.pan_step_size = abs(self.min_pan_position - self.max_pan_position)/self.pan_num_steps

        self.min_tilt_position = -0.75
        self.tilt_num_steps = 3
        self.tilt_step_size = pi/16

        self.rot_vel = 0.5 # radians per sec

    def joint_states_callback(self, msg):
        """
        A callback function that stores Stretch's joint states.
        :param self: The self reference.
        :param msg: The JointState message type.
        """
        self.joint_state = msg

    def send_command(self, command):
        '''
        Handles single joint control commands by constructing a FollowJointTrajectoryGoal
        message and sending it to the trajectory_client created in hello_misc.
        :param self: The self reference.
        :param command: A dictionary message type.
        '''
        if (self.joint_state is not None) and (command is not None):
            joint_name = command['joint']
            trajectory_goal = FollowJointTrajectoryGoal()
            trajectory_goal.trajectory.joint_names = [joint_name]
            point = JointTrajectoryPoint()

            if 'delta' in command:
                joint_index = self.joint_state.name.index(joint_name)
                joint_value = self.joint_state.position[joint_index]
                delta = command['delta']
                new_value = joint_value + delta
                point.positions = [new_value]

            elif 'position' in command:
                point.positions = [command['position']]

            point.velocities = [self.rot_vel]
            trajectory_goal.trajectory.points = [point]
            trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
            trajectory_goal.trajectory.header.frame_id = 'base_link'
            self.trajectory_client.send_goal(trajectory_goal)
            self.trajectory_client.wait_for_result()

    def find_tag(self, tag_name='docking_station'):
        """
        A function that actuates the camera to search for a defined ArUco tag
        marker. Then the function returns the pose
        :param self: The self reference.
        :param tag_name: A string value of the ArUco marker name.

        :returns transform: The docking station's TransformStamped message.
        """
        pan_command = {'joint': 'joint_head_pan', 'position': self.min_pan_position}
        self.send_command(pan_command)
        tilt_command = {'joint': 'joint_head_tilt', 'position': self.min_tilt_position}
        self.send_command(tilt_command)

        for i in range(self.tilt_num_steps):
            for j in range(self.pan_num_steps):
                pan_command = {'joint': 'joint_head_pan', 'delta': self.pan_step_size}
                self.send_command(pan_command)
                rospy.sleep(0.2)

                try:
                    transform = self.tf_buffer.lookup_transform('base_link',
                                                                tag_name,
                                                                rospy.Time())
                    rospy.loginfo("Found Requested Tag: \n%s", transform)
                    self.transform_pub.publish(transform)
                    return transform
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    continue

            pan_command = {'joint': 'joint_head_pan', 'position': self.min_pan_position}
            self.send_command(pan_command)
            tilt_command = {'joint': 'joint_head_tilt', 'delta': self.tilt_step_size}
            self.send_command(tilt_command)
            rospy.sleep(.25)

        rospy.loginfo("The requested tag '%s' was not found", tag_name)

    def main(self):
        """
        Function that initiates the issue_command function.
        :param self: The self reference.
        """
        hm.HelloNode.main(self, 'aruco_tag_locator', 'aruco_tag_locator', wait_for_first_pointcloud=False)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.sleep(1.0)
        rospy.loginfo('Searching for docking ArUco tag.')
        pose = self.find_tag("docking_station")

if __name__ == '__main__':
    try:
        node = LocateArUcoTag()
        node.main()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')

```

### The Code Explained
Now let's break the code down.

```python
#!/usr/bin/env python3
```

Every Python ROS [Node](http://wiki.ros.org/Nodes) will have this declaration at the top. The first line makes sure your script is executed as a Python3 script.

```python
import rospy
import time
import tf2_ros
import numpy as np
from math import pi

import hello_helpers.hello_misc as hm
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import TransformStamped
```

You need to import `rospy` if you are writing a ROS [Node](http://wiki.ros.org/Nodes). Import other python modules needed for this node. Import the `FollowJointTrajectoryGoal` from the [control_msgs.msg](http://wiki.ros.org/control_msgs) package to control the Stretch robot. Import `JointTrajectoryPoint` from the [trajectory_msgs](http://wiki.ros.org/trajectory_msgs) package to define robot trajectories. The [hello_helpers](https://github.com/hello-robot/stretch_ros/tree/master/hello_helpers) package consists of a module that provides various Python scripts used across [stretch_ros](https://github.com/hello-robot/stretch_ros). In this instance, we are importing the `hello_misc` script.

```python
def __init__(self):
    """
    A function that initializes the subscriber and other needed variables.
    :param self: The self reference.
    """
    hm.HelloNode.__init__(self)

    self.joint_states_sub = rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)
    self.transform_pub = rospy.Publisher('ArUco_transform', TransformStamped, queue_size=10)

    self.joint_state = None
```

The `LocateArUcoTag` class inherits the `HelloNode` class from `hm` and is instantiated.

Set up a subscriber with `rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)`.  We're going to subscribe to the topic `stretch/joint_states`, looking for `JointState` messages.  When a message comes in, ROS is going to pass it to the function `joint_states_callback()` automatically.

`rospy.Publisher('ArUco_transform', TransformStamped, queue_size=10)` declares that your node is publishing to the `ArUco_transform` topic using the message type `TransformStamped`. The `queue_size` argument limits the amount of queued messages if any subscriber is not receiving them fast enough.

```python
self.min_pan_position = -4.10
self.max_pan_position =  1.50
self.pan_num_steps = 10
self.pan_step_size = abs(self.min_pan_position - self.max_pan_position)/self.pan_num_steps
```

Provide the minimum and maximum joint positions for the head pan. These values are needed for sweeping the head to search for the ArUco tag. We also define the number of steps for the sweep, then create the step size for the head pan joint.

```python
self.min_tilt_position = -0.75
self.tilt_num_steps = 3
self.tilt_step_size = pi/16
```

Set the minimum position of the tilt joint, the number of steps, and the size of each step.

```python
self.rot_vel = 0.5 # radians per sec
```

Define the head actuation rotational velocity.

```python
def joint_states_callback(self, msg):
    """
    A callback function that stores Stretch's joint states.
    :param self: The self reference.
    :param msg: The JointState message type.
    """
    self.joint_state = msg
```

The `joint_states_callback()` function stores Stretch's joint states.

```python
def send_command(self, command):
    '''
    Handles single joint control commands by constructing a FollowJointTrajectoryGoal
    message and sending it to the trajectory_client created in hello_misc.
    :param self: The self reference.
    :param command: A dictionary message type.
    '''
    if (self.joint_state is not None) and (command is not None):
        joint_name = command['joint']
        trajectory_goal = FollowJointTrajectoryGoal()
        trajectory_goal.trajectory.joint_names = [joint_name]
        point = JointTrajectoryPoint()
```

Assign `trajectory_goal` as a `FollowJointTrajectoryGoal` message type. Then extract the string value from the `joint` key. Also, assign `point` as a `JointTrajectoryPoint` message type.

```python
if 'delta' in command:
    joint_index = self.joint_state.name.index(joint_name)
    joint_value = self.joint_state.position[joint_index]
    delta = command['delta']
    new_value = joint_value + delta
    point.positions = [new_value]
```

Check to see if `delta` is a key in the command dictionary. Then get the current position of the joint and add the delta as a new position value.

```python
elif 'position' in command:
    point.positions = [command['position']]
```

Check to see if `position` is a key in the command dictionary. Then extract the position value.

```python
point.velocities = [self.rot_vel]
trajectory_goal.trajectory.points = [point]
trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
trajectory_goal.trajectory.header.frame_id = 'base_link'
self.trajectory_client.send_goal(trajectory_goal)
self.trajectory_client.wait_for_result()
```

Then `trajectory_goal.trajectory.points` is defined by the positions set in `point`. Specify the coordinate frame that we want (*base_link*) and set the time to be now. Make the action call and send the goal. The last line of code waits for the result before it exits the python script.

```python
def find_tag(self, tag_name='docking_station'):
    """
    A function that actuates the camera to search for a defined ArUco tag
    marker. Then the function returns the pose
    :param self: The self reference.
    :param tag_name: A string value of the ArUco marker name.

    :returns transform: The docking station's TransformStamped message.
    """
    pan_command = {'joint': 'joint_head_pan', 'position': self.min_pan_position}
    self.send_command(pan_command)
    tilt_command = {'joint': 'joint_head_tilt', 'position': self.min_tilt_position}
    self.send_command(tilt_command)
```

Create a dictionary to get the head in its initial position for its search and send the commands with the `send_command()` function.

```python
for i in range(self.tilt_num_steps):
    for j in range(self.pan_num_steps):
        pan_command = {'joint': 'joint_head_pan', 'delta': self.pan_step_size}
        self.send_command(pan_command)
        rospy.sleep(0.5)
```

Utilize a nested for loop to sweep the pan and tilt in increments. Then update the `joint_head_pan` position by the `pan_step_size`. Use `rospy.sleep()` function to give time to the system to do a Transform lookup before the next step.

```python
try:
    transform = self.tf_buffer.lookup_transform('base_link',
                                                tag_name,
                                                rospy.Time())
    rospy.loginfo("Found Requested Tag: \n%s", transform)
    self.transform_pub.publish(transform)
    return transform
except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    continue
```

Use a try-except block to look up the transform between the *base_link* and the requested ArUco tag. Then publish and return the `TransformStamped` message.

```python
pan_command = {'joint': 'joint_head_pan', 'position': self.min_pan_position}
self.send_command(pan_command)
tilt_command = {'joint': 'joint_head_tilt', 'delta': self.tilt_step_size}
self.send_command(tilt_command)
rospy.sleep(.25)
```

Begin sweep with new tilt angle.

```python
def main(self):
    """
    Function that initiates the issue_command function.
    :param self: The self reference.
    """
    hm.HelloNode.main(self, 'aruco_tag_locator', 'aruco_tag_locator', wait_for_first_pointcloud=False)
```

Create a function, `main()`, to do the setup for the `hm.HelloNode` class and initialize the `aruco_tag_locator` node.

```python
self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
self.tf_buffer = tf2_ros.Buffer()
self.listener = tf2_ros.TransformListener(self.tf_buffer)
rospy.sleep(1.0)
```

Create a StaticTranformBoradcaster Node. Also, start a tf buffer that will store the tf information for a few seconds. Then set up a tf listener, which will subscribe to all of the relevant tf topics, and keep track of the information. Include `rospy.sleep(1.0)` to give the listener some time to accumulate transforms.

```python
rospy.loginfo('Searching for docking ArUco tag.')
pose = self.find_tag("docking_station")
```

Notice Stretch is searching for the ArUco tag with a `rospy.loginfo()` function. Then search for the ArUco marker for the docking station.

```python
if __name__ == '__main__':
    try:
        node = LocateArUcoTag()
        node.main()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')
```

Declare `LocateArUcoTag` object. Then run the `main()` method.
