## Example 6

In this example, we will review a Python script that prints and stores the effort values from a specified joint. If you are looking for a continuous print of the joint state efforts while Stretch is in action, then you can use the [rostopic command-line tool](https://docs.ros.org/en/iron/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html) shown in the [Internal State of Stretch Tutorial](https://github.com/hello-robot/stretch_tutorials/blob/master/ros2/internal_state_of_stretch.md).

<p align="center">
  <img src="https://raw.githubusercontent.com/hello-robot/stretch_tutorials/noetic/images/effort_sensing.gif"/>
</p>

Begin by running the following command in a terminal.

```{.bash .shell-prompt}
ros2 launch stretch_core stretch_driver.launch.py
```

There's no need to switch to the position mode in comparison with ROS1 because the default mode of the launcher is this position mode. Then run the [effort_sensing.py](https://github.com/hello-robot/stretch_tutorials/blob/iron/stretch_ros_tutorials/effort_sensing.py) node. In a new terminal, execute:

```{.bash .shell-prompt}
cd ament_ws/src/stretch_tutorials/stretch_ros_tutorials/
python3 effort_sensing.py
```

This will send a `FollowJointTrajectory` command to move Stretch's arm or head while also printing the effort of the lift.

### The Code

```python
##!/usr/bin/env python3

import rclpy
import hello_helpers.hello_misc as hm
import os
import csv
import pandas as pd
import matplotlib.pyplot as plt
import time
from matplotlib import animation
from datetime import datetime
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class JointActuatorEffortSensor(hm.HelloNode):
    def __init__(self, export_data=True, animate=True):
        hm.HelloNode.__init__(self)
        hm.HelloNode.main(self, 'issue_command', 'issue_command', wait_for_first_pointcloud=False)
        self.joints = ['joint_lift']
        self.joint_effort = []
        self.save_path = '/home/hello-robot/ament_ws/src/stretch_tutorials/stored_data'
        self.export_data = export_data
        

    def issue_command(self):
        trajectory_goal = FollowJointTrajectory.Goal()
        trajectory_goal.trajectory.joint_names = self.joints

        point0 = JointTrajectoryPoint()
        point0.positions = [0.9]

        trajectory_goal.trajectory.points = [point0]
        trajectory_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory_goal.trajectory.header.frame_id = 'base_link'

        self._send_goal_future = self.trajectory_client.send_goal_async(trajectory_goal, self.feedback_callback)
        self.get_logger().info('Sent position goal = {0}'.format(trajectory_goal))
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Failed')
            return

        self.get_logger().info('Succeded')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Sent position goal = {0}'.format(result))

    def feedback_callback(self, feedback_msg):
        if 'wrist_extension' in self.joints:
            self.joints.remove('wrist_extension')
            self.joints.append('joint_arm_l0')

        current_effort = []
        for joint in self.joints:
            index = self.joint_state.name.index(joint)
            current_effort.append(self.joint_state.effort[index])

        if not self.export_data:
            print("name: " + str(self.joints))
            print("effort: " + str(current_effort))
        else:
            self.joint_effort.append(current_effort)

        if self.export_data:
            file_name = datetime.now().strftime("%Y-%m-%d_%I-%p")
            completeName = os.path.join(self.save_path, file_name)
            with open(completeName, "w") as f:
                writer = csv.writer(f)
                writer.writerow(self.joints)
                writer.writerows(self.joint_effort)
    
    def plot_data(self, animate = True):
        time.sleep(0.1)
        file_name = datetime.now().strftime("%Y-%m-%d_%I-%p")
        self.completeName = os.path.join(self.save_path, file_name)
        self.data = pd.read_csv(self.completeName)
        self.y_anim = []
        self.animate = animate

        for joint in self.data.columns:

            # Create figure, labels, and title
            fig = plt.figure()
            plt.title(joint + ' Effort Sensing')
            plt.ylabel('Effort')
            plt.xlabel('Data Points')

            # Conditional statement for animation plotting
            if self.animate:
                self.effort = self.data[joint]
                frames = len(self.effort)-1
                anim = animation.FuncAnimation(fig=fig,func=self.plot_animate, repeat=False,blit=False,frames=frames, interval =75)
                plt.show()

                ## If you want to save a video, make sure to comment out plt.show(),
                ## right before this comment.
                # save_path = str(self.completeName + '.mp4')
                # anim.save(save_path, writer=animation.FFMpegWriter(fps=10))

                # Reset y_anim for the next joint effort animation
                del self.y_anim[:]

            # Conditional statement for regular plotting (No animation)
            else:
                self.data[joint].plot(kind='line')
                # save_path = str(self.completeName + '.png')
                # plt.savefig(save_path, bbox_inches='tight')
                plt.show()
    
    def plot_animate(self,i):
        # Append self.effort values for given joint
        self.y_anim.append(self.effort.values[i])
        plt.plot(self.y_anim, color='blue')

    def main(self):
        self.get_logger().info('issuing command')
        self.issue_command()

def main():
    try:
        node = JointActuatorEffortSensor(export_data=True, animate=True)
        node.main()
        node.plot_data()
        node.new_thread.join()
        
        
    except KeyboardInterrupt:
        node.get_logger().info('interrupt received, so shutting down')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

```

### The Code Explained
This code is similar to that of the [multipoint_command](https://github.com/hello-robot/stretch_tutorials/blob/iron/stretch_ros_tutorials/multipoint_command.py) and [joint_state_printer](https://github.com/hello-robot/stretch_tutorials/blob/iron/stretch_ros_tutorials/joint_state_printer.py) node. Therefore, this example will highlight sections that are different from those tutorials. Now let's break the code down.

```python
#!/usr/bin/env python3
```

Every Python ROS [Node](https://docs.ros.org/en/iron/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html) will have this declaration at the top. The first line makes sure your script is executed as a Python3 script.

```python
import rclpy
import hello_helpers.hello_misc as hm
import os
import csv
import pandas as pd
import matplotlib.pyplot as plt
import time
from matplotlib import animation
from datetime import datetime
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
```

You need to import rclpy if you are writing a ROS [Node](https://docs.ros.org/en/iron/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html). Import the `FollowJointTrajectory` from the `control_msgs.action` package to control the Stretch robot. Import `JointTrajectoryPoint` from the `trajectory_msgs` package to define robot trajectories. The `hello_helpers` package consists of a module that provides various Python scripts used across [stretch_ros](https://github.com/hello-robot/stretch_ros2). In this instance, we are importing the `hello_misc` script.

```Python
class JointActuatorEffortSensor(hm.HelloNode):
    def __init__(self, export_data=False):
        hm.HelloNode.__init__(self)
        hm.HelloNode.main(self, 'issue_command', 'issue_command', wait_for_first_pointcloud=False)
```

The `JointActuatorEffortSensor` class inherits the `HelloNode` class from `hm` and is initialized also the HelloNode class already have the topic joint_states, thanks to this we don't need to create a subscriber.

```python
self.joints = ['joint_lift']
```

Create a list of the desired joints you want to print.

```Python
self.joint_effort = []
self.save_path = '/home/hello-robot/ament_ws/src/stretch_tutorials/stored_data'
self.export_data = export_data
```

Create an empty list to store the joint effort values. The `self.save_path` is the directory path where the .txt file of the effort values will be stored. You can change this path to a preferred directory. The `self.export_data` is a boolean and its default value is set to `True`. If set to `False`, then the joint values will be printed in the terminal, otherwise, it will be stored in a .txt file and that's what we want to see the plot graph.

```python
self._send_goal_future = self.trajectory_client.send_goal_async(trajectory_goal, self.feedback_callback)
self._send_goal_future.add_done_callback(self.goal_response_callback)
```

The [ActionClient.send_goal_async()](https://docs.ros2.org/latest/api/rclpy/api/actions.html#rclpy.action.client.ActionClient.send_goal_async) method returns a future to a goal handle. Include the goal and `feedback_callback` functions in the send goal function. Also we need to register a `goal_response_callback` for when the future is complete 

```python
def goal_response_callback(self,future):
    goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Failed')
            return

        self.get_logger().info('Succeded')

        
```
Looking at the `goal_response_callback` in more detail we can see if the future is complete with the messages that will appear.

```python
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
```
We need the goal_handle to request the result with the method get_result_async. With this we will get a future that will complete when the result is ready so we need a callback for this result.

```python
def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Sent position goal = {0}'.format(result))
```
In the result callback we log the result of our poistion goal 

```python
def feedback_callback(self,feedback_msg):
```

The feedback callback function takes in the `FollowJointTrajectoryActionFeedback` message as its argument.

```python
if 'wrist_extension' in self.joints:
    self.joints.remove('wrist_extension')
    self.joints.append('joint_arm_l0')
```

Use a conditional statement to replace `wrist_extenstion` with `joint_arm_l0`. This is because `joint_arm_l0` has the effort values that the `wrist_extension` is experiencing.

```python
current_effort = []
for joint in self.joints:
    index = self.joint_states.name.index(joint)
    current_effort.append(self.joint_states.effort[index])
```

Create an empty list to store the current effort values. Then use a for loop to parse the joint names and effort values.

```python
if not self.export_data:
    print("name: " + str(self.joints))
    print("effort: " + str(current_effort))
else:
    self.joint_effort.append(current_effort)
```

Use a conditional statement to print effort values in the terminal or store values into a list that will be used for exporting the data in a .txt file.

```python
if self.export_data:
            file_name = datetime.now().strftime("%Y-%m-%d_%I-%p")
            completeName = os.path.join(self.save_path, file_name)
            with open(completeName, "w") as f:
                writer = csv.writer(f)
                writer.writerow(self.joints)
                writer.writerows(self.joint_effort)
```
A conditional statement is used to export the data to a .txt file. The file's name is set to the date and time the node was executed.

```python
def plot_data(self, animate = True):
        time.sleep(0.1)
        file_name = datetime.now().strftime("%Y-%m-%d_%I-%p")
        self.completeName = os.path.join(self.save_path, file_name)
        self.data = pd.read_csv(self.completeName)
        self.y_anim = []
        self.animate = animate
```
This function will help us initialize some values to plot our data, the file is going to be the one we created and we need to create an empty list for the animation

```python
for joint in self.data.columns:

            # Create figure, labels, and title
            fig = plt.figure()
            plt.title(joint + ' Effort Sensing')
            plt.ylabel('Effort')
            plt.xlabel('Data Points')
```
Create a for loop to print each joint's effort writing the correct labels for x and y

```python
if self.animate:
    self.effort = self.data[joint]
    frames = len(self.effort)-1
    anim = animation.FuncAnimation(fig=fig,func=self.plot_animate, repeat=False,blit=False,frames=frames, interval =75)
    plt.show()
    del self.y_anim[:]

else:
    self.data[joint].plot(kind='line')
    # save_path = str(self.completeName + '.png')
    # plt.savefig(save_path, bbox_inches='tight')
    plt.show()
```
This is a conditional statement for the animation plotting

```python
def plot_animate(self,i):
        self.y_anim.append(self.effort.values[i])
        plt.plot(self.y_anim, color='blue')
```
We will create another function that will plot every increment in the data frame

<p align="center">
  <img src="https://raw.githubusercontent.com/hello-robot/stretch_tutorials/noetic/stored_data/2022-06-30_11:26:20-AM.png"/>
</p>