## Example 6

<!-- In this example, we will review a Python script that prints out the positions of a selected group of Stretch's joints. This script is helpful if you need the joint positions after you teleoperated Stretch with the Xbox controller or physically moved the robot to the desired configuration after hitting the run stop button. -->

<!-- If you are looking for a continuous print of the joint states while Stretch is in action, then you can use the [rostopic command-line tool](http://wiki.ros.org/rostopic) shown in the [Internal State of Stretch Tutorial](internal_state_of_stretch.md). -->


![image](images/effort_sensing.gif)





Begin by running `roscore` in a terminal. Then set the ros parameter to *position* mode  by running the following commands in a new terminal.

```bash
# Terminal 2
rosparam set /stretch_driver/mode "position"
roslaunch stretch_core stretch_driver.launch
```

In a new terminal type the following commands.

```bash
# Terminal 3
cd catkin_ws/src/stretch_ros_tutorials/src/
python effort_sensing.py
```

This will send a `FollowJointTrajectory` command to move Stretch's arm or head.

<!-- It's important to note that the arm has 4 prismatic joints and the sum of these positions gives the wrist extension distance. The wrist extension is needed when sending [joint trajectory commands](follow_joint_trajectory.md) to the robot. Here is an image of the arm joints for reference: -->




### The Code
```python
#!/usr/bin/env python
import rospy
import time
import actionlib
import os
import csv

from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
import hello_helpers.hello_misc as hm
from datetime import datetime

class JointActuatorEffortSensor(hm.HelloNode):
    """
    A class that sends multiple joint trajectory goals to a single joint.
    """
    def __init__(self):
        """
        Function that initializes the subscriber,and other features.
        :param self: The self reference.
        """
        hm.HelloNode.__init__(self)
        self.sub = rospy.Subscriber('joint_states', JointState, self.callback)
        self.joints = ['joint_lift']
        self.joint_effort = []
        self.save_path = '/home/hello-robot/catkin_ws/src/stretch_ros_tutorials/stored_data'
        self.export_data = False

    def callback(self, msg):
        """
        Callback function to update and store JointState messages.
        :param self: The self reference.
        :param msg: The JointState message.
        """
        self.joint_states = msg

    def issue_command(self):
        """
        Function that makes an action call and sends joint trajectory goals
        to a single joint
        :param self: The self reference.
        """
        trajectory_goal = FollowJointTrajectoryGoal()
        trajectory_goal.trajectory.joint_names = self.joints

        point0 = JointTrajectoryPoint()
        point0.positions = [0.9]

        trajectory_goal.trajectory.points = [point0]
        trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
        trajectory_goal.trajectory.header.frame_id = 'base_link'
        self.trajectory_client.send_goal(trajectory_goal, feedback_cb=self.feedback_callback, done_cb=self.done_callback)
        rospy.loginfo('Sent stow goal = {0}'.format(trajectory_goal))
        self.trajectory_client.wait_for_result()

    def feedback_callback(self,feedback):
        """
        The feedback_callback function deals with the incoming feedback messages
        from the trajectory_client. Although, in this function, we do not use the
        feedback information.
        :param self: The self reference.
        :param feedback: FollowJointTrajectoryActionFeedback message.
        """
        if 'wrist_extension' in self.joints:
            self.joints.remove('wrist_extension')
            self.joints.append('joint_arm_l0')

        current_effort = []
        for joint in self.joints:
            index = self.joint_states.name.index(joint)
            current_effort.append(self.joint_states.effort[index])

        if not self.export_data:
            print("name: " + str(self.joints))
            print("effort: " + str(current_effort))
        else:
            self.joint_effort.append(current_effort)


    def done_callback(self, status, result):
        """
        The done_callback function will be called when the joint action is complete.
        Within this function we export the data to a .txt file in  the /stored_data directory.
        :param self: The self reference.
        :param status: status attribute from FollowJointTrajectoryActionResult message.
        :param result: result attribute from FollowJointTrajectoryActionResult message.
        """
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo('Suceeded')
        else:
            rospy.loginfo('Failed')

        if self.export_data:
            file_name = datetime.now().strftime("%Y-%m-%d_%I:%M:%S-%p")
            completeName = os.path.join(self.save_path, file_name)
            with open(completeName, "w") as f:
                writer = csv.writer(f)
                writer.writerow(self.joints)
                writer.writerows(self.joint_effort)

    def main(self):
        """
        Function that initiates the issue_command function.
        :param self: The self reference.
        """
        hm.HelloNode.main(self, 'issue_command', 'issue_command', wait_for_first_pointcloud=False)
        rospy.loginfo('issuing command...')
        self.issue_command()
        time.sleep(2)

if __name__ == '__main__':
	try:
		node = JointActuatorEffortSensor()
		node.main()
	except KeyboardInterrupt:
		rospy.loginfo('interrupt received, so shutting down')

```


### The Code Explained
Now let's break the code down.

```python
#!/usr/bin/env python
```
Every Python ROS [Node](http://wiki.ros.org/Nodes) will have this declaration at the top. The first line makes sure your script is executed as a Python script.


```python

```

**Previous Example** [Example 5](example_4.md)
