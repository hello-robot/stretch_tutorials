## Example 4

![image](images/balloon.png)

Let's bringup stretch in RViz by using the following command.

```bash
ros2 launch stretch_core stretch_driver.launch.py
ros2 run rviz2 rviz2 -d `ros2 pkg prefix stretch_calibrtion`/rviz/stretch_simple_test.rviz
```
In a new terminal run the following commands to create a marker.

```bash
ros2 run stretch_ros_tutorials marker
```
The gif below demonstrates how to add a new *Marker* display type, and change the topic name from `visualization_marker` to `balloon`. A red sphere Marker should appear above the Stretch robot.

![image](images/balloon.gif)


### The Code
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

class Balloon(Node):
	def __init__(self):
		super().__init__('stretch_marker')

		self.publisher_ = self.create_publisher(Marker, 'balloon', 10)	
		
		self.marker = Marker()

		self.marker.header.frame_id = '/base_link'
		self.marker.header.stamp = self.get_clock().now().to_msg()
		self.marker.type = self.marker.SPHERE

		self.marker.id = 0

		self.marker.action = self.marker.ADD

		self.marker.scale.x = 0.5
		self.marker.scale.y = 0.5
		self.marker.scale.z = 0.5

		self.marker.color.r = 1.0
		self.marker.color.g = 0.0
		self.marker.color.b = 0.0

		self.marker.color.a = 1.0

		self.marker.pose.position.x = 0.0
		self.marker.pose.position.y = 0.0
		self.marker.pose.position.z = 2.0

		self.get_logger().info("Publishing the balloon topic. Use RViz to visualize.")

	def publish_marker(self):
		self.publisher_.publish(self.marker)

def main(args=None):
	rclpy.init(args=args)
	balloon = Balloon()
	while rclpy.ok():
		balloon.publish_marker()
	balloon.destroy_node()	
	rclpy.shutdown()

if __name__ == '__main__':
	main()
```


### The Code Explained
Now let's break the code down.

```python
#!/usr/bin/env python3
```
Every Python ROS [Node](http://wiki.ros.org/Nodes) will have this declaration at the top. The first line makes sure your script is executed as a Python script.


```python
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
```
You need to import rclpy if you are writing a ROS 2 Node. Import the `Marker` type from the visualization_msgs.msg package. This import is required to publish a Marker, which will be visualized in RViz.

```python
self.publisher_ = self.create_publisher(Marker, 'balloon', 10)	
```
This declares that your node is publishing to the */ballon* topic using the message type *Twist*.


```python
self.marker = Marker()
self.marker.header.frame_id = '/base_link'
self.marker.header.stamp = self.get_clock().now().to_msg()
self.marker.type = self.marker.SPHERE

```

<!-- TODO: Update links -->
Create a maker. Markers of all shapes share a common type. Set the frame ID and type. The frame ID is the frame in which the position of the marker is specified. The type is the shape of the marker. Further details on marker shapes can be found here: [RViz Markers](http://wiki.ros.org/rviz/DisplayTypes/Marker)

```python
self.marker.id = 0
```
Each marker has a unique ID number. If you have more than one marker that you want displayed at a given time, then each needs to have a unique ID number. If you publish a new marker with the same ID number of an existing marker, it will replace the existing marker with that ID number.

```python
self.marker.action = self.marker.ADD
```
This line of code sets the action. You can add, delete, or modify markers.

```python
self.marker.scale.x = 0.5
self.marker.scale.y = 0.5
self.marker.scale.z = 0.5
```
These are the size parameters for the marker. These will vary by marker type.

```python
self.marker.color.r = 1.0
self.marker.color.g = 0.0
self.marker.color.b = 0.0
```
Color of the object, specified as r/g/b/a, with values in the range of [0, 1].

```python
self.marker.color.a = 1.0
```
The alpha value is from 0 (invisible) to 1 (opaque). If you don't set this then it will automatically default to zero, making your marker invisible.

```python
self.marker.pose.position.x = 0.0
self.marker.pose.position.y = 0.0
self.marker.pose.position.z = 2.0
```

Specify the pose of the marker. Since spheres are rotationally invariant, we're only going to specify the positional elements. As usual, these are in the coordinate frame named in frame_id. In this case, the position will always be directly 2 meters above the frame_id (*base_link*), and will move with it.


```python
def publish_marker(self):
		self.publisher_.publish(self.marker)
```
Publish the Marker data structure to be visualized in RViz.

```python
def main(args=None):
	rclpy.init(args=args)
	balloon = Balloon()
```

The next line, rospy.init. In this case, your node will take on the name talker. NOTE: the name must be a base name, i.e. it cannot contain any slashes "/".

Setup Balloon class with `balloon = Balloon()`


```python
while rclpy.ok():
	balloon.publish_marker()
	balloon.destroy_node()	
	rclpy.shutdown()
```

This loop is a fairly standard rclpy construct: checking the rclpy.ok() flag and then doing work. You have to run this check to see if your program should exit (e.g. if there is a Ctrl-C or otherwise).
