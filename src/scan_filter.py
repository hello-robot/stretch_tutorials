#!/usr/bin/env python

# Every python controller needs these lines
import rospy
from numpy import linspace, inf
from math import sin
# We're going to subscribe to a LaserScan message
from sensor_msgs.msg import LaserScan

class ScanFilter:
	"""
	A class that implements a LaserScan filter that removes all of the points.
	that are not infront of the robot.
	"""
	def __init__(self):
		# We're going to assume that the robot is pointing up the x-axis, so that
		# any points with y coordinates further than half of the defined
		# width (1 meter) from the axis are not considered
		self.width = 1
		self.extent = self.width / 2.0

		# Set up a subscriber.  We're going to subscribe to the topic "scan",
		# looking for LaserScan messages.  When a message comes in, ROS is going
		# to pass it to the function "callback" automatically
		self.sub = rospy.Subscriber('/scan', LaserScan, self.callback)

		# Set up a publisher.  This will publish on a topic called "filtered_scan",
		# with a LaserScan message type
		self.pub = rospy.Publisher('filtered_scan', LaserScan, queue_size=10)

		# Create log message
		rospy.loginfo("Publishing the filtered_scan topic. Use RViz to visualize.")


	def callback(self,msg):
		"""
		Callback function to deal with incoming laserscan messages.
		:param self: The self reference.
		:param msg: The subscribed laserscan message.

		:publishes msg: updated laserscan message.
		"""
		# Figure out the angles of the scan.  We're going to do this each time, in case we're subscribing to more than one
		# laser, with different numbers of beams
		angles = linspace(msg.angle_min, msg.angle_max, len(msg.ranges))

		# Work out the y coordinates of the ranges
		points = [r * sin(theta) if (theta < -2.5 or theta > 2.5) else inf for r,theta in zip(msg.ranges, angles)]

		# If we're close to the x axis, keep the range, otherwise use inf, which means "no return"
		new_ranges = [r if abs(y) < self.extent else inf for r,y in zip(msg.ranges, points)]

		# Substitute in the new ranges in the original message, and republish it
		msg.ranges = new_ranges
		self.pub.publish(msg)

if __name__ == '__main__':
	# Initialize the node, and call it "scan_filter"
	rospy.init_node('scan_filter')

	# Instantiate the ScanFilter class
	Scanfilter()

	# Give control to ROS.  This will allow the callback to be called whenever new
	# messages come in.  If we don't put this line in, then the node will not work,
	# and ROS will not process any messages
	rospy.spin()
