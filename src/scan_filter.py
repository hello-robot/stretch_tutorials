#!/usr/bin/env python

# Every python controller needs these lines
import rospy
from numpy import linspace, inf
from math import sin
# We're going to subscribe to a LaserScan message.
from sensor_msgs.msg import LaserScan

class Scanfilter:
	def __init__(self):

		# We're going to assume that the robot is pointing up the x-axis, so that any points with y coordinates further
		# than half a robot width from the axis are not in front of the robot.
		self.extent = .5 / 2.0

		# Set up a subscriber.  We're going to subscribe to the topic "counter",
		# looking for Int64 messages.  When a message comes in, ROS is going to pass
		# it to the function "callback" automatically.
		self.sub = rospy.Subscriber('/scan', LaserScan, self.callback)
		self.pub = rospy.Publisher('filtered_scan', LaserScan, queue_size=10)


	def callback(self,msg):
		# print('Minimum scan value = ' + str(len(msg.ranges)))
		#
		# center_index = len(msg.ranges)/3
		# rospy.sleep(1)
		"""
		:param self: Self reference.
		:param msg: LaserScan message.
		"""

		# Figure out the angles of the scan.  We're going to do this each time, in case we're subscribing to more than one
		# laser, with different numbers of beams.
		angles = linspace(msg.angle_min, msg.angle_max, len(msg.ranges))

		# # Work out the y coordinates of the ranges.
		points = [r * sin(theta) if (theta < -2.5 or theta > 2.5) else inf for r,theta in zip(msg.ranges, angles)]
		#
		# If we're close to the x axis, keep the range, otherwise use inf, which means "no return".
		new_ranges = [r if abs(y) < self.extent else inf for r,y in zip(msg.ranges, points)]

		# Substitute in the new ranges in the original message, and republish it.
		msg.ranges = new_ranges
		self.pub.publish(msg)

if __name__ == '__main__':
	# Initialize the node, and call it "move".
	rospy.init_node('scan_filter')

	# Setup Move class to base_motion
	Scan()

	rospy.spin()
