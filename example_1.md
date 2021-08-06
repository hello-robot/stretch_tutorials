## Example 1

![image](images/move.gif)


The goal of this example to give you an enhance understanding how to control the mobile base by sending `Twist` messages to a Stretch robot.

First, bringup [Stretch in the empty world simulation](getting_started). Open a new terminal and go into the *src* folder where the [move](src/move.py) node is located. To drive the robot forward, type the following in the terminal.

```
python move.py
```
This will drive the robot forward ... 


```python
#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist

class Move:
	def __init__(self):
		self.pub = rospy.Publisher('/stretch_diff_drive_controller/cmd_vel', Twist, queue_size=1)

	def move_forward(self):
		command = Twist()
		command.linear.x = 0.1
		command.linear.y = 0.0
		command.linear.z = 0.0
		command.angular.x = 0.0
		command.angular.y = 0.0
		command.angular.z = 0.0
		self.pub.publish(command)

if __name__ == '__main__':
	rospy.init_node('move')
	base_motion = Move()
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		base_motion.move_forward()
		rate.sleep()
```
