#!/usr/bin/env python

# Import modules
import rospy
from numpy import linspace, inf, tanh
from math import sin

# The Twist message is used to send velocities to the robot
from geometry_msgs.msg import Twist

# We're going to subscribe to a LaserScan message
from sensor_msgs.msg import LaserScan

class Avoider:
    """
    A class that implements both a LaserScan filter and base velocity control
    for collision avoidance.
    """
    def __init__(self):
        """
        Function that initializes the subscriber, publisher, and marker features.
        :param self: The self reference.
        """
        # Set up a publisher and a subscriber.  We're going to call the subscriber
        # "scan", and filter the ranges similar to what we did in example 2.
        # For the publisher, we're going to use the topic name /stretch/cmd_vel
        self.pub = rospy.Publisher('/stretch/cmd_vel', Twist, queue_size=1) #/stretch_diff_drive_controller/cmd_vel for gazebo
        self.sub = rospy.Subscriber('/scan', LaserScan, self.set_speed)

        # We're going to assume that the robot is pointing at the x-axis, so that
        # any points with y coordinates further than half of the defined
        # width (1 meter) from the axis are not considered.
        self.width = 1
        self.extent = self.width / 2.0

        # We want the robot to drive foward or backwards until it is 0.5 m from
        # the closest obstacle measured in front of it
        self.distance = 0.5

        # Alocate a Twist to use, and set everything to zero.  We're going to do
        # this here, to save some time in the callback function, set_speed()
        self.twist = Twist()
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0

    def set_speed(self,msg):
        """
        Callback function to deal with incoming laserscan messages.
        :param self: The self reference.
        :param msg: The subscribed laserscan message.

        :publishes self.twist: Twist message.
        """
        # Figure out the angles of the scan.  We're going to do this each time,
        # in case we're subscribing to more than one laser, with different numbers of beams
        angles = linspace(msg.angle_min, msg.angle_max, len(msg.ranges))

        # Work out the y coordinates of the ranges
        points = [r * sin(theta) if (theta < -2.5 or theta > 2.5) else inf for r,theta in zip(msg.ranges, angles)]

        # If we're close to the x axis, keep the range, otherwise use inf, which means "no return"
        new_ranges = [r if abs(y) < self.extent else inf for r,y in zip(msg.ranges, points)]

        # Calculate the difference of the closest measured scan and where we want the robot to stop
        error = min(new_ranges) - self.distance

        # Using hyperbolic tanget for speed regulation, with a threshold to stop
        # and driving when it is close to the desired distance
        self.twist.linear.x = tanh(error) if (error > 0.05 or error < -0.05) else 0

        # Publish the command using the publisher
        self.pub.publish(self.twist)

if __name__ == '__main__':
    # Initialize the node, and call it "avoider"
    rospy.init_node('avoider')

	# Instantiate the Avoider class
    Avoider()

    # Give control to ROS.  This will allow the callback to be called whenever new
    # messages come in.  If we don't put this line in, then the node will not work,
    # and ROS will not process any messages
    rospy.spin()
