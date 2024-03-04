# Writing Nodes

TODO

You would publish [geometry_msgs/Twist](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html) messages to the [/stretch/cmd_vel topic](#TODO). Since Twist messages are generalized to robots that can move with velocity in any direction, only the `Twist.linear.x` (translational velocity) and `Twist.angular.z` (rotational velocity) fields apply for differential drive mobile bases.
