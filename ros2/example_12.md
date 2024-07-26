# Example 12
For this example, we will send follow joint trajectory commands for the head camera to search and locate an ArUco tag. In this instance, a Stretch robot will try to locate the docking station's ArUco tag.

## Modifying Stretch Marker Dictionary YAML File
When defining the ArUco markers on Stretch, hello robot utilizes a YAML file, [stretch_marker_dict.yaml](https://github.com/hello-robot/stretch_ros2/blob/humble/stretch_core/config/stretch_marker_dict.yaml), that holds the information about the markers. A further breakdown of the YAML file can be found in our [Aruco Marker Detection](aruco_marker_detection.md) tutorial.

Below is what needs to be included in the [stretch_marker_dict.yaml](https://github.com/hello-robot/stretch_ros2/blob/humble/stretch_core/config/stretch_marker_dict.yaml) file so the [detect_aruco_markers](https://github.com/hello-robot/stretch_ros2/blob/humble/stretch_core/stretch_core/detect_aruco_markers.py) node can find the docking station's ArUco tag.

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
ros2 launch stretch_core stretch_driver.launch.py
```

To activate the RealSense camera and publish topics to be visualized, run the following launch file in a new terminal.

```{.bash .shell-prompt}
ros2 launch stretch_core d435i_high_resolution.launch.py
```

Next, run the stretch ArUco launch file which will bring up the [detect_aruco_markers](https://github.com/hello-robot/stretch_ros2/blob/humble/stretch_core/stretch_core/detect_aruco_markers.py) node. In a new terminal, execute:

```{.bash .shell-prompt}
ros2 launch stretch_core stretch_aruco.launch.py
```

Within this tutorial package, there is an [RViz config file](https://github.com/hello-robot/stretch_tutorials/blob/humble/rviz/aruco_detector_example.rviz) with the topics for the transform frames in the Display tree. You can visualize these topics and the robot model by running the command below in a new terminal.

```{.bash .shell-prompt}
ros2 run rviz2 rviz2 -d /home/hello-robot/ament_ws/src/stretch_tutorials/rviz/aruco_detector_example.rviz
```

Then run the [aruco_tag_locator.py](https://github.com/hello-robot/stretch_tutorials/blob/humble/stretch_ros_tutorials/aruco_tag_locator.py) node. In a new terminal, execute:

```{.bash .shell-prompt}
cd ament_ws/src/stretch_tutorials/stretch_ros_tutorials/
python3 aruco_tag_locator.py
```

<p align="center">
  <img src="https://raw.githubusercontent.com/hello-robot/stretch_tutorials/noetic/images/aruco_locator.gif"/>
</p>

### The Code

```python
#!/usr/bin/env python3

# Import modules
import rclpy
import time
import tf2_ros
from tf2_ros import TransformException
from rclpy.time import Time
from math import pi

# Import hello_misc script for handling trajectory goals with an action client
import hello_helpers.hello_misc as hm

# We're going to subscribe to a JointState message type, so we need to import
# the definition for it
from sensor_msgs.msg import JointState

# Import the FollowJointTrajectory from the control_msgs.action package to
# control the Stretch robot
from control_msgs.action import FollowJointTrajectory

# Import JointTrajectoryPoint from the trajectory_msgs package to define
# robot trajectories
from trajectory_msgs.msg import JointTrajectoryPoint

# Import TransformStamped from the geometry_msgs package for the publisher
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
        # Initialize the inhereted hm.Hellonode class
        hm.HelloNode.__init__(self)
        hm.HelloNode.main(self, 'aruco_tag_locator', 'aruco_tag_locator', wait_for_first_pointcloud=False)
        # Initialize subscriber
        self.joint_states_sub = self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 1)
        # Initialize publisher
        self.transform_pub = self.create_publisher(TransformStamped, 'ArUco_transform', 10)

        # Initialize the variable that will store the joint state positions
        self.joint_state = None

        # Provide the min and max joint positions for the head pan. These values
        # are needed for sweeping the head to search for the ArUco tag
        self.min_pan_position = -3.8
        self.max_pan_position =  1.50

        # Define the number of steps for the sweep, then create the step size for
        # the head pan joint
        self.pan_num_steps = 10
        self.pan_step_size = abs(self.min_pan_position - self.max_pan_position)/self.pan_num_steps

        # Define the min tilt position, number of steps, and step size
        self.min_tilt_position = -0.75
        self.tilt_num_steps = 3
        self.tilt_step_size = pi/16

        # Define the head actuation rotational velocity
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

            # Extract the string value from the `joint` key
            joint_name = command['joint']

            # Set trajectory_goal as a FollowJointTrajectory.Goal and define
            # the joint name
            trajectory_goal = FollowJointTrajectory.Goal()
            trajectory_goal.trajectory.joint_names = [joint_name]

            # Create a JointTrajectoryPoint message type
            point = JointTrajectoryPoint()

            # Check to see if `delta` is a key in the command dictionary
            if 'delta' in command:
                # Get the current position of the joint and add the delta as a
                # new position value
                joint_index = self.joint_state.name.index(joint_name)
                joint_value = self.joint_state.position[joint_index]
                delta = command['delta']
                new_value = joint_value + delta
                point.positions = [new_value]

            # Check to see if `position` is a key in the command dictionary
            elif 'position' in command:
                # extract the head position value from the `position` key
                point.positions = [command['position']]

            # Set the rotational velocity
            point.velocities = [self.rot_vel]

            # Assign goal position with updated point variable
            trajectory_goal.trajectory.points = [point]

            # Specify the coordinate frame that we want (base_link) and set the time to be now.
            trajectory_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
            trajectory_goal.trajectory.header.frame_id = 'base_link'
            
            # Make the action call and send the goal. The last line of code waits
            # for the result
            self.trajectory_client.send_goal(trajectory_goal)

    def find_tag(self, tag_name='docking_station'):
        """
        A function that actuates the camera to search for a defined ArUco tag
        marker. Then the function returns the pose.
        :param self: The self reference.
        :param tag_name: A string value of the ArUco marker name.

        :returns transform: The docking station's TransformStamped message.
        """
        # Create dictionaries to get the head in its initial position
        pan_command = {'joint': 'joint_head_pan', 'position': self.min_pan_position}
        self.send_command(pan_command)
        tilt_command = {'joint': 'joint_head_tilt', 'position': self.min_tilt_position}
        self.send_command(tilt_command)

        # Nested for loop to sweep the joint_head_pan and joint_head_tilt in increments
        for i in range(self.tilt_num_steps):
            for j in range(self.pan_num_steps):
                # Update the joint_head_pan position by the pan_step_size
                pan_command = {'joint': 'joint_head_pan', 'delta': self.pan_step_size}
                self.send_command(pan_command)

                # Give time for system to do a Transform lookup before next step
                time.sleep(0.2)

                # Use a try-except block
                try:
                    now = Time()
                    # Look up transform between the base_link and requested ArUco tag
                    transform = self.tf_buffer.lookup_transform('base_link',
                                                            tag_name,
                                                            now)
                    self.get_logger().info(f"Found Requested Tag: \n{transform}")

                    # Publish the transform
                    self.transform_pub.publish(transform)

                    # Return the transform
                    return transform
                except TransformException as ex:
                    continue

            # Begin sweep with new tilt angle
            pan_command = {'joint': 'joint_head_pan', 'position': self.min_pan_position}
            self.send_command(pan_command)
            tilt_command = {'joint': 'joint_head_tilt', 'delta': self.tilt_step_size}
            self.send_command(tilt_command)
            time.sleep(0.25)

        # Notify that the requested tag was not found
        self.get_logger().info("The requested tag '%s' was not found", tag_name)

    def main(self):
        """
        Function that initiates the issue_command function.
        :param self: The self reference.
        """
        # Create a StaticTranformBoradcaster Node. Also, start a Tf buffer that
        # will store the tf information for a few seconds.Then set up a tf listener, which
        # will subscribe to all of the relevant tf topics, and keep track of the information
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Give the listener some time to accumulate transforms
        time.sleep(1.0)

        # Notify Stretch is searching for the ArUco tag with `get_logger().info()`
        self.get_logger().info('Searching for docking ArUco tag')

        # Search for the ArUco marker for the docking station
        pose = self.find_tag("docking_station")

def main():
    try:
        # Instantiate the `LocateArUcoTag()` object
        node = LocateArUcoTag()
        # Run the `main()` method
        node.main()
        node.new_thread.join()
    except:
        node.get_logger().info('Interrupt received, so shutting down')
        node.destroy_node()
        rclpy.shutdown()


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
import time
import tf2_ros
from tf2_ros import TransformException
from rclpy.time import Time
from math import pi

import hello_helpers.hello_misc as hm
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import TransformStamped
```

You need to import `rclpy` if you are writing a ROS [Node](http://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html). Import other python modules needed for this node. Import the `FollowJointTrajectory` from the [control_msgs.action](http://wiki.ros.org/control_msgs) package to control the Stretch robot. Import `JointTrajectoryPoint` from the [trajectory_msgs](https://github.com/ros2/common_interfaces/tree/humble/trajectory_msgs) package to define robot trajectories. The [hello_helpers](https://github.com/hello-robot/stretch_ros2/tree/humble/hello_helpers) package consists of a module that provides various Python scripts used across [stretch_ros](https://github.com/hello-robot/stretch_ros2). In this instance, we are importing the `hello_misc` script.

```python
def __init__(self):
        # Initialize the inhereted hm.Hellonode class
        hm.HelloNode.__init__(self)
        hm.HelloNode.main(self, 'aruco_tag_locator', 'aruco_tag_locator', wait_for_first_pointcloud=False)
        # Initialize subscriber
        self.joint_states_sub = self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 1)
        # Initialize publisher
        self.transform_pub = self.create_publisher(TransformStamped, 'ArUco_transform', 10)

        # Initialize the variable that will store the joint state positions
        self.joint_state = None
```

The `LocateArUcoTag` class inherits the `HelloNode` class from `hm` and is instantiated.

Set up a subscriber with `self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 1)`.  We're going to subscribe to the topic `stretch/joint_states`, looking for `JointState` messages.  When a message comes in, ROS is going to pass it to the function `joint_states_callback()` automatically.

`self.create_publisher(TransformStamped, 'ArUco_transform', 10)` declares that your node is publishing to the `ArUco_transform` topic using the message type `TransformStamped`. The `10` argument limits the amount of queued messages if any subscriber is not receiving them fast enough.

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
    self.joint_state = msg
```

The `joint_states_callback()` function stores Stretch's joint states.

```python
def send_command(self, command):
    if (self.joint_state is not None) and (command is not None):
        joint_name = command['joint']
        trajectory_goal = FollowJointTrajectory.Goal()
        trajectory_goal.trajectory.joint_names = [joint_name]
        point = JointTrajectoryPoint()
```

Assign `trajectory_goal` as a `FollowJointTrajectory.Goal` message type. Then extract the string value from the `joint` key. Also, assign `point` as a `JointTrajectoryPoint` message type.

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
trajectory_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
trajectory_goal.trajectory.header.frame_id = 'base_link'
self.trajectory_client.send_goal(trajectory_goal)
```

Then `trajectory_goal.trajectory.points` is defined by the positions set in `point`. Specify the coordinate frame that we want (*base_link*) and set the time to be now. Make the action call and send the goal.

```python
def find_tag(self, tag_name='docking_station'):
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
        time.sleep(0.5)
```

Utilize a nested for loop to sweep the pan and tilt in increments. Then update the `joint_head_pan` position by the `pan_step_size`. Use `time.sleep()` function to give time to the system to do a Transform lookup before the next step.

```python
try:
    now = Time()
    transform = self.tf_buffer.lookup_transform('base_link',
                                                tag_name,
                                                now)
    self.get_logger().info("Found Requested Tag: \n%s", transform)
    self.transform_pub.publish(transform)
    return transform
except TransformException as ex:
    continue
```

Use a try-except block to look up the transform between the *base_link* and the requested ArUco tag. Then publish and return the `TransformStamped` message.

```python
pan_command = {'joint': 'joint_head_pan', 'position': self.min_pan_position}
self.send_command(pan_command)
tilt_command = {'joint': 'joint_head_tilt', 'delta': self.tilt_step_size}
self.send_command(tilt_command)
time.sleep(.25)
```

Begin sweep with new tilt angle.

```python
def main(self):
    self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
    self.tf_buffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
    time.sleep(1.0)
```

Create a StaticTranformBoradcaster Node. Also, start a tf buffer that will store the tf information for a few seconds. Then set up a tf listener, which will subscribe to all of the relevant tf topics, and keep track of the information. Include `time.sleep(1.0)` to give the listener some time to accumulate transforms.

```python
self.get_logger().info('Searching for docking ArUco tag')
pose = self.find_tag("docking_station")
```

Notice Stretch is searching for the ArUco tag with a `self.get_logger().info()` function. Then search for the ArUco marker for the docking station.

```python
def main():
    try:
        node = LocateArUcoTag()
        node.main()
        node.new_thread.join()
    except:
        node.get_logger().info('Interrupt received, so shutting down')
        node.destroy_node()
        rclpy.shutdown()
```
Instantiate the `LocateArUcoTag()` object and run the `main()` method.
