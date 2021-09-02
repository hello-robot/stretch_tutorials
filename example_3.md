## Example 3

The aim of example 3 is to combine the two previous examples and have Stretch utilize its laser scan data to avoid collision with objects as it drives forward.

Begin by running `roscore` in a terminal. Then set the ros parameter to *navigation* mode  by running the following commands in a new terminal.

```bash
rosparam set /stretch_driver/mode "navigation"
roslaunch stretch_core stretch_driver.launch
```
Then in a new terminal type the following to activate the LiDAR sensor.
```bash
roslaunch stretch_core rplidar.launch
```
To activate the avoider node, type the following in a new terminal.
```bash
cd catkin_ws/src/stretch_ros_turotials/src/
python3 avoider.py
```
To stop the node from sending twist messages, type **Ctrl** + **c**.

<p align="center">
  <img height=600 src="images/avoider.gif"/>
</p>

### The Code

```python
#!/usr/bin/env python

import rospy
from numpy import linspace, inf, tanh
from math import sin
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Avoider:
    def __init__(self):
        self.pub = rospy.Publisher('/stretch/cmd_vel', Twist, queue_size=1) #/stretch_diff_drive_controller/cmd_vel for gazebo
        self.sub = rospy.Subscriber('/scan', LaserScan, self.set_speed)

        self.width = 1
        self.extent = self.width / 2.0
        self.distance = 0.5

        self.twist = Twist()
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0

    def set_speed(self,msg):
        angles = linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        points = [r * sin(theta) if (theta < -2.5 or theta > 2.5) else inf for r,theta in zip(msg.ranges, angles)]
        new_ranges = [r if abs(y) < self.extent else inf for r,y in zip(msg.ranges, points)]
        error = min(new_ranges) - self.distance

        self.twist.linear.x = tanh(error) if (error > 0.05 or error < -0.05) else 0
        self.pub.publish(self.twist)		

if __name__ == '__main__':
    rospy.init_node('avoider')
    Avoider()
    rospy.spin()
```

### The Code Explained

Now let's break the code down.

```python
#!/usr/bin/env python
```
Every Python ROS [Node](http://wiki.ros.org/Nodes) will have this declaration at the top. The first line makes sure your script is executed as a Python script.


```python
import rospy
from numpy import linspace, inf, tanh
from math import sin
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
```
You need to import rospy if you are writing a ROS Node. There are functions from numpy and math that are required within this code, thus linspace, inf, tanh, and sin are imported. The sensor_msgs.msg import is so that we can subscribe to LaserScan messages. The geometry_msgs.msg import is so that we can send velocity commands to the robot.


```python
self.pub = rospy.Publisher('/stretch/cmd_vel', Twist, queue_size=1)#/stretch_diff_drive_controller/cmd_vel for gazebo
```
This section of code defines the talker's interface to the rest of ROS. pub = rospy.Publisher("/stretch/cmd_vel", Twist, queue_size=1) declares that your node is publishing to the /stretch/cmd_vel topic using the message type Twist.


```python
self.sub = rospy.Subscriber('/scan', LaserScan, self.set_speed)
```
Set up a subscriber.  We're going to subscribe to the topic "scan", looking for LaserScan messages.  When a message comes in, ROS is going to pass it to the function "set_speed" automatically.


```python
self.width = 1
self.extent = self.width / 2.0
self.distance = 0.5
```
*self.width* is the width of the laser scan we want in front of Stretch. Since Stretch's front is pointing in the x-axis, any points with y coordinates further than half of the defined width (*self.extent*) from the x-axis are not considered. *self.distance* deinfest the stopping distance from an object.


```python
self.twist = Twist()
self.twist.linear.x = 0.0
self.twist.linear.y = 0.0
self.twist.linear.z = 0.0
self.twist.angular.x = 0.0
self.twist.angular.y = 0.0
self.twist.angular.z = 0.0
```
Allocate a Twist to use, and set everything to zero.  We're going to do this when the class is initiating. Redefining this within the callback function, `set_speed()` can be more computationally taxing.

```python
angles = linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
points = [r * sin(theta) if (theta < -2.5 or theta > 2.5) else inf for r,theta in zip(msg.ranges, angles)]
new_ranges = [r if abs(y) < self.extent else inf for r,y in zip(msg.ranges, points)]
```
This line of code utilizes linspace to compute each angle of the subscribed scan. Here we are computing the y coordinates of the ranges that are below -2.5 and above 2.5 radians of the scan angles. These limits are sufficient for considering scan ranges in front of Stretch, but these values can be altered to your preference. If the absolute value of a point's y-coordinate is under self.extent then keep the range, otherwise use inf, which means "no return".


```python
error = min(new_ranges) - self.distance
```
Calculate the difference of the closest measured scan and where we want the robot to stop. We define this as *error*.


```python
self.twist.linear.x = tanh(error) if (error > 0.05 or error < -0.05) else 0
```
Set the speed according to a tanh function. This method gives a nice smooth mapping from distance to speed, and asymptotes at +/- 1


```python
self.pub.publish(self.twist)
```
Publish the Twist message.

```python
rospy.init_node('avoider')
Avoider()
rospy.spin()
```
The next line, rospy.init_node(NAME, ...), is very important as it tells rospy the name of your node -- until rospy has this information, it cannot start communicating with the ROS Master. In this case, your node will take on the name talker. NOTE: the name must be a base name, i.e. it cannot contain any slashes "/".

Setup Avoider class with `Avioder()`

Give control to ROS with `rospy.spin()`. This will allow the callback to be called whenever new messages come in. If we don't put this line in, then the node will not work, and ROS will not process any messages.

**Next Tutorial:** [Example 4](example_4.md)
