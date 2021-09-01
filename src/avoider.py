#!/usr/bin/env python

# Every python controller needs these lines
import rospy
from numpy import linspace, inf, tanh
from math import sin

# The Twist message is used to send velocities to the robot.
from geometry_msgs.msg import Twist

# We're going to subscribe to a LaserScan message.
from sensor_msgs.msg import LaserScan

class Avoider:
    def __init__(self):
        # Set up a publisher and a subscriber.  We're going to call the subscriber
        # "scan", and filter the ranges similar to what we did in example 2.
        # For the publisher, we're going to use the topic name /stretch/cmd_vel.
        self.pub = rospy.Publisher('/stretch/cmd_vel', Twist, queue_size=1) #/stretch_diff_drive_controller/cmd_vel for gazebo
        self.sub = rospy.Subscriber('/scan', LaserScan, self.set_speed)

        # We're going to assume that the robot is pointing up the x-axis, so that

        # any points with y coordinates further than half of the defined
        # width (1 meter) from the axis are not considered.
        self.width = 1
        self.extent = self.width / 2.0


        self.distance = 0.5

        # Alocate a Twist to use, and set everything to zero.  We're going to do this here, to save some time in
        # the callback.
        self.twist = Twist()
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0

    # Called every time we get a LaserScan message from ROS.
    def set_speed(self,msg):
        # Figure out the angles of the scan.  We're going to do this each time, in case we're subscribing to more than one
        # laser, with different numbers of beams.
        angles = linspace(msg.angle_min, msg.angle_max, len(msg.ranges))

        # Work out the y coordinates of the ranges.
        points = [r * sin(theta) if (theta < -2.5 or theta > 2.5) else inf for r,theta in zip(msg.ranges, angles)]

        # If we're close to the x axis, keep the range, otherwise use inf, which means "no return".
        new_ranges = [r if abs(y) < self.extent else inf for r,y in zip(msg.ranges, points)]

        error = min(new_ranges) - self.distance

        # Using hyperbolic tanget for speed regulation
        self.twist.linear.x = tanh(error) if (error > 0.05 or error < -0.05) else 0
        self.pub.publish(self.twist)		# Publish the command using the global publisher

if __name__ == '__main__':
    rospy.init_node('avoider')
    Avoider()

    rospy.spin()
