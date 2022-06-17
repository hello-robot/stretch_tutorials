## Example 4

![image](images/balloon.png)

Let's bringup stretch in the willowgarage world from the [gazebo basics tutorial](gazebo_basics.md) and RViz by using the following command.

```bash
# Terminal 1
roslaunch stretch_gazebo gazebo.launch world:=worlds/willowgarage.world rviz:=true
```
the `rviz` flag will open an RViz window  to visualize a variety of ROS topics. In a new terminal run the following commands to create a marker.

```bash
# Terminal 2
cd catkin_ws/src/stretch_ros_tutorials/src/
python3 marker.py
```
The gif below demonstrates how to add a new *Marker* display type, and change the topic name from `visualization_marker` to `balloon`. A red sphere Marker should appear above the Stretch robot.

![image](images/balloon.gif)


### The Code
```python
#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker

class Balloon():
	"""
	A class that attaches a Sphere marker directly above the Stretch robot.
	"""
	def __init__(self):
		"""
		Function that initializes the markers features.
		:param self: The self reference
		"""
		self.publisher = rospy.Publisher('balloon', Marker, queue_size=10)
		self.marker = Marker()
		self.marker.header.frame_id = '/base_link'
		self.marker.header.stamp = rospy.Time()
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

	def publish_marker(self):
		"""
		Function that publishes the sphere marker
		:param self: The self reference

		:publishes self.marker: Marker message
		"""
		self.publisher.publish(self.marker)


if __name__ == '__main__':
	rospy.init_node('marker', argv=sys.argv)
	ballon = Balloon()
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		ballon.publish_marker()
		rate.sleep()		
```


### The Code Explained
Now let's break the code down.

```python
#!/usr/bin/env python
```
Every Python ROS [Node](http://wiki.ros.org/Nodes) will have this declaration at the top. The first line makes sure your script is executed as a Python script.


```python
import rospy
from visualization_msgs.msg import Marker
```
You need to import rospy if you are writing a ROS Node. Import the `Marker` type from the visualization_msgs.msg package. This import is required to publish a Marker, which will be visualized in RViz.

```python
self.pub = rospy.Publisher('balloon', Marker, queue_size=10)
```
This section of code defines the talker's interface to the rest of ROS. pub = rospy.Publisher("balloon", Twist, queue_size=1) declares that your node is publishing to the */ballon* topic using the message type *Twist*.


```python
self.marker = Marker()
self.marker.header.frame_id = '/base_link'
self.marker.header.stamp = rospy.Time()
self.marker.type = self.marker.SPHERE
```

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
		self.publisher.publish(self.marker)
```
Publish the Marker data structure to be visualized in RViz.

```python
rospy.init_node('marker', argv=sys.argv)
ballon = Balloon()
rate = rospy.Rate(10)
```

The next line, rospy.init_node(NAME, ...), is very important as it tells rospy the name of your node -- until rospy has this information, it cannot start communicating with the ROS Master. In this case, your node will take on the name talker. NOTE: the name must be a base name, i.e. it cannot contain any slashes "/".

Setup Balloon class with `Balloon()`

Give control to ROS with `rospy.spin()`. This will allow the callback to be called whenever new messages come in. If we don't put this line in, then the node will not work, and ROS will not process any messages.


```python
while not rospy.is_shutdown():
	ballon.publish_marker()
	rate.sleep()
```

This loop is a fairly standard rospy construct: checking the rospy.is_shutdown() flag and then doing work. You have to check is_shutdown() to check if your program should exit (e.g. if there is a Ctrl-C or otherwise). The loop calls rate.sleep(), which sleeps just long enough to maintain the desired rate through the loop.
