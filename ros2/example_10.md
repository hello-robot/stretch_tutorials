## Example 10
!!! note
    ROS 2 tutorials are still under active development. 

This tutorial provides you with an idea of what tf2 can do in the Python track. We will elaborate on how to create a tf2 static broadcaster and listener.

## tf2 Static Broadcaster

For the tf2 static broadcaster node, we will be publishing three child static frames in reference to the *link_mast*, *link_lift*, and *link_wrist_yaw* frames.

Begin by starting up the stretch driver launch file.

```{.bash .shell-prompt}
ros2 launch stretch_core stretch_driver.launch.py
```
Open RViz in another terminal and add the RobotModel and TF plugins in the left hand panel

```{.bash .shell-prompt}
ros2 run rviz2 rviz2
```
Then run the tf2 broadcaster node to visualize three static frames.

```{.bash .shell-prompt}
ros2 run stretch_ros_tutorials tf_broadcaster
```

The GIF below visualizes what happens when running the previous node.

<p align="center">
  <img src="https://raw.githubusercontent.com/hello-robot/stretch_tutorials/noetic/images/tf2_broadcaster.gif"/>
</p>

**OPTIONAL**: If you would like to see how the static frames update while the robot is in motion, run the stow command node and observe the tf frames in RViz.

```{.bash .shell-prompt}
ros2 run stretch_ros_tutorials stow_command
```
<p align="center">
  <img src="https://raw.githubusercontent.com/hello-robot/stretch_tutorials/noetic/images/tf2_broadcaster_with_stow.gif"/>
</p>

### The Code

```python
#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
import tf_transformations
from geometry_msgs.msg import TransformStamped

# This node publishes three child static frames in reference to their parent frames as below:
# parent -> link_mast            child -> fk_link_mast
# parent -> link_lift            child -> fk_link_lift
# parent -> link_wrist_yaw       child -> fk_link_wrist_yaw

class FixedFrameBroadcaster(Node):
    def __init__(self):
        super().__init__('stretch_tf_broadcaster')
        self.br = TransformBroadcaster(self)
        time_period = 0.1 # seconds
        self.timer = self.create_timer(time_period, self.broadcast_timer_callback)

        self.mast = TransformStamped()
        self.mast.header.frame_id = 'link_mast'
        self.mast.child_frame_id = 'fk_link_mast'
        self.mast.transform.translation.x = 0.0
        self.mast.transform.translation.y = 0.0
        self.mast.transform.translation.z = 0.0
        q = tf_transformations.quaternion_from_euler(1.5707, 0, -1.5707)
        self.mast.transform.rotation.x = q[0]
        self.mast.transform.rotation.y = q[1]
        self.mast.transform.rotation.z = q[2]
        self.mast.transform.rotation.w = q[3]

        self.lift = TransformStamped()
        self.lift.header.stamp = self.get_clock().now().to_msg()
        self.lift.header.frame_id = 'link_lift'
        self.lift.child_frame_id = 'fk_link_lift'
        self.lift.transform.translation.x = 0.0
        self.lift.transform.translation.y = 2.0
        self.lift.transform.translation.z = 0.0
        q = tf_transformations.quaternion_from_euler(1.5707, 0, -1.5707)
        self.lift.transform.rotation.x = q[0]
        self.lift.transform.rotation.y = q[1]
        self.lift.transform.rotation.z = q[2]
        self.lift.transform.rotation.w = q[3]
        self.br.sendTransform(self.lift)

        self.wrist = TransformStamped()
        self.wrist.header.stamp = self.get_clock().now().to_msg()
        self.wrist.header.frame_id = 'link_wrist_yaw'
        self.wrist.child_frame_id = 'fk_link_wrist_yaw'
        self.wrist.transform.translation.x = 0.0
        self.wrist.transform.translation.y = 2.0
        self.wrist.transform.translation.z = 0.0
        q = tf_transformations.quaternion_from_euler(1.5707, 0, -1.5707)
        self.wrist.transform.rotation.x = q[0]
        self.wrist.transform.rotation.y = q[1]
        self.wrist.transform.rotation.z = q[2]
        self.wrist.transform.rotation.w = q[3]
        self.br.sendTransform(self.wrist)

        self.get_logger().info("Publishing Tf frames. Use RViz to visualize.")

    def broadcast_timer_callback(self):
        self.mast.header.stamp = self.get_clock().now().to_msg()
        self.br.sendTransform(self.mast)

        self.lift.header.stamp = self.get_clock().now().to_msg()
        self.br.sendTransform(self.lift)

        self.wrist.header.stamp = self.get_clock().now().to_msg()
        self.br.sendTransform(self.wrist)


def main(args=None):
    rclpy.init(args=args)
    tf_broadcaster = FixedFrameBroadcaster()

    rclpy.spin(tf_broadcaster)

    tf_broadcaster.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### The Code Explained
Now let's break the code down.

```python
#!/usr/bin/env python3
```

Every Python ROS [Node](http://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html) will have this declaration at the top. The first line makes sure your script is executed as a Python script.

```python
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
import tf_transformations
from geometry_msgs.msg import TransformStamped
```

You need to import rclpy if you are writing a ROS 2 node. Import `tf_transformations` to get quaternion values from Euler angles. Import the `TransformStamped` from the `geometry_msgs.msg` package because we will be publishing static frames and it requires this message type. The `tf2_ros` package provides an implementation of a `TransformBroadcaster.` to help make the task of publishing transforms easier.

```python
class FixedFrameBroadcaster(Node):
    def __init__(self):
        super().__init__('stretch_tf_broadcaster')
        self.br = TransformBroadcaster(self)
```

Here we create a `TransformStamped` object which will be the message we will send over once populated.

```python
        self.lift = TransformStamped()
        self.lift.header.stamp = self.get_clock().now().to_msg()
        self.lift.header.frame_id = 'link_lift'
        self.lift.child_frame_id = 'fk_link_lift'
```

We need to give the transform being published a timestamp, we'll just stamp it with the current time, `self.get_clock().now().to_msg()`. Then, we need to set the name of the parent frame of the link we're creating, in this case *link_lift*. Finally, we need to set the name of the child frame of the link we're creating. In this instance, the child frame is *fk_link_lift*.

```python
        self.mast.transform.translation.x = 0.0
        self.mast.transform.translation.y = 0.0
        self.mast.transform.translation.z = 0.0
```

Set the translation values for the child frame.

```python
        q = tf_transformations.quaternion_from_euler(1.5707, 0, -1.5707)
        self.lift.transform.rotation.x = q[0]
        self.lift.transform.rotation.y = q[1]
        self.lift.transform.rotation.z = q[2]
        self.lift.transform.rotation.w = q[3]
```

The `quaternion_from_euler()` function takes in a Euler angle argument and returns a quaternion values. Then set the rotation values to the transformed quaternions.

This process will be completed for the *link_mast* and *link_wrist_yaw* as well.

```python
self.br.sendTransform(self.lift)
```

Send the three transforms using the `sendTransform()` function.

```python
def main(args=None):
    rclpy.init(args=args)
    tf_broadcaster = FixedFrameBroadcaster()
```

Instantiate the `FixedFrameBroadcaster()` class.

```python
rclpy.spin(tf_broadcaster)
```

Give control to ROS.  This will allow the callback to be called whenever new
messages come in.  If we don't put this line in, then the node will not work,
and ROS will not process any messages.


## tf2 Static Listener
In the previous section of the tutorial, we created a tf2 broadcaster to publish three static transform frames. In this section we will create a tf2 listener that will find the transform between *fk_link_lift* and *link_grasp_center*.

Begin by starting up the stretch driver launch file.

```{.bash .shell-prompt}
ros2 launch stretch_core stretch_driver.launch.py
```

Then run the tf2 broadcaster node to create the three static frames.

```{.bash .shell-prompt}
ros2 run stretch_ros_tutorials tf_broadcaster
```

Finally, run the tf2 listener node to print the transform between two links.

```{.bash .shell-prompt}
ros2 run stretch_ros_tutorials tf_listener
```

Within the terminal the transform will be printed every 1 second. Below is an example of what will be printed in the terminal. There is also an image for reference of the two frames.

```{.bash .no-copy}
[INFO] [1659551318.098168]: The pose of target frame link_grasp_center with reference from fk_link_lift is:
translation:
  x: 1.08415191335
  y: -0.176147838153
  z: 0.576720021135
rotation:
  x: -0.479004489528
  y: -0.508053545368
  z: -0.502884087254
  w: 0.509454501243
```

<p align="center">
  <img src="https://raw.githubusercontent.com/hello-robot/stretch_tutorials/noetic/images/tf2_listener.png"/>
</p>

### The Code

```python
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class FrameListener(Node):

    def __init__(self):
        super().__init__('stretch_tf_listener')

        self.declare_parameter('target_frame', 'link_grasp_center')
        self.target_frame = self.get_parameter(
            'target_frame').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        time_period = 1.0 # seconds
        self.timer = self.create_timer(time_period, self.on_timer)

    def on_timer(self):
        from_frame_rel = self.target_frame
        to_frame_rel = 'fk_link_mast'

        try:
            now = Time()
            trans = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                now)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        self.get_logger().info(
                        f'the pose of target frame {from_frame_rel} with reference to {to_frame_rel} is: {trans}')


def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### The Code Explained

```python
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
```

Here, we create a `TransformListener` object. Once the listener is created, it starts receiving tf2 transformations over the wire, and buffers them for up to 10 seconds.

```python
        from_frame_rel = self.target_frame
        to_frame_rel = 'fk_link_mast'
```

Store frame names in variables that will be used to compute transformations.

```python
        try:
            now = Time()
            trans = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                now)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
```

Try to look up the transform we want. Use a try-except block, since it may fail on any single call, due to internal timing issues in the transform publishers. Look up transform between *from_frame_rel* and *to_frame_rel* frames with the `lookup_transform()` function.
