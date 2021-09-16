## FollowJointTrajectory Commands

Stretch ROS driver offers a [`FollowJointTrajectory`](http://docs.ros.org/en/api/control_msgs/html/action/FollowJointTrajectory.html) action service for its arm. Within this tutorial we will have a simple FollowJointTrajectory commands sent to a Stretch robot to execute.

## Stow Command Example
<p align="center">
  <img src="images/stow_command.gif"/>
</p>

Begin by running `roscore` in a terminal. Then set the ros parameter to *position* mode  by running the following commands in a new terminal.

```bash
rosparam set /stretch_driver/mode "position"
roslaunch stretch_core stretch_driver.launch
```

In a new terminal type the following commands.

```bash
cd catkin_ws/src/stretch_ros_turotials/src/
python3 stow_command.py
```

This will sent FollowJointTrajectory commands to stow Stretch's arm.
### The Code

```python
#!/usr/bin/env python3

import rospy
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import hello_helpers.hello_misc as hm
import time

class StowCommand(hm.HelloNode):

    def __init__(self):
        hm.HelloNode.__init__(self)

    def issue_stow_command(self):
        stow_point = JointTrajectoryPoint()
        stow_point.time_from_start = rospy.Duration(0.000)
        stow_point.positions = [0.2, 0.0, 3.4]

        trajectory_goal = FollowJointTrajectoryGoal()
        trajectory_goal.trajectory.joint_names = ['joint_lift', 'wrist_extension', 'joint_wrist_yaw']
        trajectory_goal.trajectory.points = [stow_point]
        trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
        trajectory_goal.trajectory.header.frame_id = 'base_link'

        self.trajectory_client.send_goal(trajectory_goal)
        rospy.loginfo('Sent stow goal = {0}'.format(trajectory_goal))
        self.trajectory_client.wait_for_result()

    def main(self):
        hm.HelloNode.main(self, 'stow_command', 'stow_command', wait_for_first_pointcloud=False)
        rospy.loginfo('stowing...')
        self.issue_stow_command()
        time.sleep(2)


if __name__ == '__main__':
    try:
        node = StowCommand()
        node.main()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')
```

### The Code Explained

Now let's break the code down.

```python
#!/usr/bin/env python3
```
Every Python ROS [Node](http://wiki.ros.org/Nodes) will have this declaration at the top. The first line makes sure your script is executed as a Python script.


```python
import rospy
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import hello_helpers.hello_misc as hm
import time
```
You need to import rospy if you are writing a ROS Node. Import the FollowJointTrajectoryGoal from the [control_msgs.msg](http://wiki.ros.org/control_msgs) package to control the Stretch robot. Import JointTrajectoryPoint from the [trajectory_msgs](http://wiki.ros.org/trajectory_msgs) package to define robot trajectories. The [hello_helpers](https://github.com/hello-robot/stretch_ros/tree/master/hello_helpers) package consists of a module the provides various Python scripts used across [stretch_ros](https://github.com/hello-robot/stretch_ros). In this instance we are importing the hello_misc script.

```python
class StowCommand(hm.HelloNode):

    def __init__(self):
        hm.HelloNode.__init__(self)
```
The `StowCommand ` class inherits the `HelloNode` class from `hm` and is initialized.

```python
def issue_stow_command(self):
    stow_point = JointTrajectoryPoint()
    stow_point.time_from_start = rospy.Duration(0.000)
    stow_point.positions = [0.2, 0.0, 3.4]
```
The `issue_stow_command()` is the name of the function that will stow Stretch's arm. Within the function, we set *stow_point* as a `JointTrajectoryPoint`and provide desired positions (in meters). These are the positions of the lift, wrist extension, and yaw of the wrist, respectively. These are defined in the next set of the code.

```python
    trajectory_goal = FollowJointTrajectoryGoal()
    trajectory_goal.trajectory.joint_names = ['joint_lift', 'wrist_extension', 'joint_wrist_yaw']
    trajectory_goal.trajectory.points = [stow_point]
    trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
    trajectory_goal.trajectory.header.frame_id = 'base_link'
```
Set *trajectory_goal* as a `FollowJointTrajectoryGoal` and define the joint names as a list. Then `trajectory_goal.trajectory.points` is defined by the positions set in *stow_point*. Specify the coordinate frame that we want (base_link) and set the time to be now.

```python
self.trajectory_client.send_goal(trajectory_goal)
rospy.loginfo('Sent stow goal = {0}'.format(trajectory_goal))
self.trajectory_client.wait_for_result()
```
Make the action call and send the goal. The last line of code waits for the result before it exits the python script.

```python
def main(self):
    hm.HelloNode.main(self, 'stow_command', 'stow_command', wait_for_first_pointcloud=False)
    rospy.loginfo('stowing...')
    self.issue_stow_command()
    time.sleep(2)
```
Create a funcion, `main()`, to do all of the setup the `hm.HelloNode` class and issue the stow command.

```python
if __name__ == '__main__':
    try:
        node = StowCommand()
        node.main()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')

```
Initialize the `StowCommand()` class and set it to *node* and run the `main()` function.
