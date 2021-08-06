#!/usr/bin/env python

# Every python controller needs these lines
import rospy

# We're going to subscribe to a LaserScan message.
from sensor_msgs.msg import LaserScan

class Scan:
	def __init__(self):
		# Set up a subscriber.  We're going to subscribe to the topic "counter",
		# looking for Int64 messages.  When a message comes in, ROS is going to pass
		# it to the function "callback" automatically.
		self.sub = rospy.Subscriber('/scan', LaserScan, self.callback)

	def callback(self,msg):
		print('Minimum scan value = ' + str(len(msg.ranges)))
		rospy.sleep(1)

if __name__ == '__main__':
	# Initialize the node, and call it "move".
	rospy.init_node('scan')

	# Setup Move class to base_motion
	Scan()

	rospy.spin()
