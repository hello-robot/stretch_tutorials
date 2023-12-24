## FollowJointTrajectory Commands
!!! note
    ROS 2 tutorials are still under active development. For this exercise you'll need to have Ubuntu 22.04 and ROS Iron for it to work completly.

Stretch driver offers a [`FollowJointTrajectory`](http://docs.ros.org/en/api/control_msgs/html/action/FollowJointTrajectory.html) action service for its arm. Within this tutorial, we will have a simple FollowJointTrajectory command sent to a Stretch robot to execute.

## Stow Command Example

<p align="center">
  <img src="https://raw.githubusercontent.com/hello-robot/stretch_tutorials/ROS2/images/stow_command.gif"/>
</p>

Begin by launching stretch_driver in a terminal.

```{.bash .shell-prompt}
ros2 launch stretch_core stretch_driver.launch.py mode:=trajectory
```

In a new terminal type the following commands.

```{.bash .shell-prompt}
ros2 run stretch_ros_tutorials stow_command
```

This will send a FollowJointTrajectory command to stow Stretch's arm.

### The Code

```python
#!/usr/bin/env python3
import rclpy
from rclpy.duration import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from hello_helpers.hello_misc import HelloNode
import time
class StowCommand(HelloNode):
    def __init__(self):
        HelloNode.__init__(self)
        HelloNode.main(self, 'stow_command', 'stow_command', wait_for_first_pointcloud=False)
    def issue_stow_command(self):
        while not self.joint_state.position:
            self.get_logger().info("Waiting for joint states message to arrive")
            time.sleep(0.1)
            continue
        self.get_logger().info('Stowing...')
        joint_state = self.joint_state
        stow_point1 = JointTrajectoryPoint()
        stow_point2 = JointTrajectoryPoint()
        duration1 = Duration(seconds=0.0)
        duration2 = Duration(seconds=4.0)
        stow_point1.time_from_start = duration1.to_msg()
        stow_point2.time_from_start = duration2.to_msg()
        lift_index = joint_state.name.index('joint_lift')
        arm_index = joint_state.name.index('wrist_extension')
        wrist_yaw_index = joint_state.name.index('joint_wrist_yaw')
        joint_value1 = joint_state.position[lift_index]
        joint_value2 = joint_state.position[arm_index]
        joint_value3 = joint_state.position[wrist_yaw_index]
        
        stow_point1.positions = [joint_value1, joint_value2, joint_value3]
        stow_point2.positions = [0.2, 0.0, 3.14]
        trajectory_goal = FollowJointTrajectory.Goal()
        trajectory_goal.trajectory.joint_names = ['joint_lift', 'wrist_extension', 'joint_wrist_yaw']
        trajectory_goal.trajectory.points = [stow_point1, stow_point2]
        self.trajectory_client.send_goal_async(trajectory_goal)
        self.get_logger().info("Goal sent")
    def main(self):
        self.issue_stow_command()
def main():
    try:
        node = StowCommand()
        node.main()
        node.new_thread.join()
    except:
        node.get_logger().info("Exiting")
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
```

### The Code Explained

Now let's break the code down.

```python
#!/usr/bin/env python3
```
Every Python ROS [Node](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html) will have this declaration at the top. The first line makes sure your script is executed as a Python script.


```python
import rclpy
from rclpy.duration import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from hello_helpers.hello_misc import HelloNode
import time
```

You need to import rclpy if you are writing a ROS 2 Node. Import the FollowJointTrajectory from the [control_msgs.action](https://github.com/ros-controls/control_msgs/tree/master/control_msgs) package to control the Stretch robot. Import JointTrajectoryPoint from the [trajectory_msgs](https://github.com/ros2/common_interfaces/tree/humble/trajectory_msgs/msg) package to define robot trajectories.

```python
class StowCommand(HelloNode):
    def __init__(self):
        HelloNode.__init__(self)
        HelloNode.main(self, 'stow_command', 'stow_command', wait_for_first_pointcloud=False)
```

The `StowCommand ` class inherits from the `HelloNode` class and is initialized with the main method in HelloNode by passing the arguments node_name as 'stow_command', node_namespace as 'stow_command' and wait_for_first_pointcloud as False. Refer to the [Introduction to HelloNode]() tutorial if you haven't already to understand how this works.

```python
def issue_stow_command(self):
```

The `issue_stow_command()` method will stow Stretch's arm. Within the function, we set *stow_point* as a `JointTrajectoryPoint`and provide desired positions (in meters). These are the positions of the lift, wrist extension, and yaw of the wrist, respectively. These are defined in the next set of the code.

```python
        while not self.joint_state.position:
            self.get_logger().info("Waiting for joint states message to arrive")
            time.sleep(0.1)
            continue
        self.get_logger().info('Stowing...')
        joint_state = self.joint_state
        stow_point1 = JointTrajectoryPoint()
        stow_point2 = JointTrajectoryPoint()
        duration1 = Duration(seconds=0.0)
        duration2 = Duration(seconds=4.0)
        stow_point1.time_from_start = duration1.to_msg()
        stow_point2.time_from_start = duration2.to_msg()
        lift_index = joint_state.name.index('joint_lift')
        arm_index = joint_state.name.index('wrist_extension')
        wrist_yaw_index = joint_state.name.index('joint_wrist_yaw')
        joint_value1 = joint_state.position[lift_index]
        joint_value2 = joint_state.position[arm_index]
        joint_value3 = joint_state.position[wrist_yaw_index]
        
        stow_point1.positions = [joint_value1, joint_value2, joint_value3]
        stow_point2.positions = [0.2, 0.0, 3.14]
        trajectory_goal = FollowJointTrajectory.Goal()
        trajectory_goal.trajectory.joint_names = ['joint_lift', 'wrist_extension', 'joint_wrist_yaw']
        trajectory_goal.trajectory.points = [stow_point1, stow_point2]
```

Set *trajectory_goal* as a `FollowJointTrajectory.Goal()` and define the joint names as a list. Then `trajectory_goal.trajectory.points` is defined by the positions set in *stow_point1* and *stow_point2*.

```python
        self.trajectory_client.send_goal_async(trajectory_goal)
        self.get_logger().info("Goal sent")
```
Make the action call and send the goal.

```python
def main(args=None):
    try:
        node = StowCommand()
        node.main()
        node.new_thread.join()
    except:
        node.get_logger().info("Exiting")
        node.destroy_node()
        rclpy.shutdown()
```

Create a funcion, `main()`, to do all of the setup in the class and issue the stow command. Initialize the `StowCommand()` class and set it to *node* and run the `main()` function.

```python
if __name__ == '__main__':
    main()
```

To make the script executable call the main() function like above.


## Multipoint Command Example

<p align="center">
  <img src="https://raw.githubusercontent.com/hello-robot/stretch_tutorials/ROS2/images/multipoint.gif"/>
</p>

If you have killed the above instance of stretch_driver relaunch it again through the terminal.

```{.bash .shell-prompt}
ros2 launch stretch_core stretch_driver.launch.py mode:=trajectory
```

In a new terminal type the following commands.

```{.bash .shell-prompt}
ros2 run stretch_ros_tutorials multipoint_command
```

This will send a list of JointTrajectoryPoint's to move Stretch's arm.

### The Code

```python
#!/usr/bin/env python3
import rclpy
from rclpy.duration import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from hello_helpers.hello_misc import HelloNode
import time
class MultiPointCommand(HelloNode):
    def __init__(self):
        HelloNode.__init__(self)
        HelloNode.main(self, 'multipoint_command', 'multipoint_command', wait_for_first_pointcloud=False)
    def issue_multipoint_command(self):
        while not self.joint_state.position:
            self.get_logger().info("Waiting for joint states message to arrive")
            time.sleep(0.1)
            continue
        
        self.get_logger().info('Issuing multipoint command...')
        joint_state = self.joint_state
        duration0 = Duration(seconds=0.0)
        duration1 = Duration(seconds=6.0)
        duration2 = Duration(seconds=9.0)
        duration3 = Duration(seconds=12.0)
        duration4 = Duration(seconds=16.0)
        duration5 = Duration(seconds=20.0)
        lift_index = joint_state.name.index('joint_lift')
        arm_index = joint_state.name.index('wrist_extension')
        wrist_yaw_index = joint_state.name.index('joint_wrist_yaw')
        gripper_index = joint_state.name.index('joint_gripper_finger_left')
        joint_value1 = joint_state.position[lift_index]
        joint_value2 = joint_state.position[arm_index]
        joint_value3 = joint_state.position[wrist_yaw_index]
        joint_value4 = joint_state.position[gripper_index]
        point0 = JointTrajectoryPoint()
        point0.positions = [joint_value1, joint_value2, joint_value3, joint_value4]
        point0.velocities = [0.0, 0.0, 0.0, 0.0]
        point0.time_from_start = duration0.to_msg()
        point1 = JointTrajectoryPoint()
        point1.positions = [0.9, 0.0, 0.0, 0.0] 
        point1.time_from_start = duration1.to_msg()
        point2 = JointTrajectoryPoint()
        point2.positions = [0.9, 0.2, 0.0, -0.3]
        point2.time_from_start = duration2.to_msg()
        point3 = JointTrajectoryPoint()
        point3.positions = [0.9, 0.4, 0.0, -0.3]
        point3.time_from_start = duration3.to_msg()
        point4 = JointTrajectoryPoint()
        point4.positions = [0.9, 0.4, 0.0, 0.0]
        point4.time_from_start = duration4.to_msg()
        point5 = JointTrajectoryPoint()
        point5.positions = [0.4, 0.0, 1.54, 0.0]
        point5.time_from_start = duration5.to_msg()
        trajectory_goal = FollowJointTrajectory.Goal()
        trajectory_goal.trajectory.joint_names = ['joint_lift', 'wrist_extension', 'joint_wrist_yaw', 'joint_gripper_finger_left']
        trajectory_goal.trajectory.points = [point0, point1, point2, point3, point4, point5]
        trajectory_goal.trajectory.header.frame_id = 'base_link'
        self.trajectory_client.send_goal_async(trajectory_goal)
        self.get_logger().info("Goal sent")
        
    def main(self):
        self.issue_multipoint_command()
def main():
    try:
        node = MultiPointCommand()
        node.main()
        node.new_thread.join()
    except KeyboardInterrupt:
        node.get_logger().info("Exiting")
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
```

### The Code Explained.
Seeing that there are similarities between the multipoint and stow command nodes, we will only breakdown the distinct components of the multipoint_command node.

```python
        point1 = JointTrajectoryPoint()
        point1.positions = [0.9, 0.0, 0.0, 0.0] 
        point1.time_from_start = duration1.to_msg()
```

Set *point1* as a `JointTrajectoryPoint`and provide desired positions (in meters). These are the positions of the lift, wrist_extension, wrist_yaw and gripper_aperture joints, respectively.

!!! note
    The lift and wrist extension can only go up to 0.2 m/s. If you do not provide any velocities or accelerations for the lift or wrist extension, then they go to their default values. However, the Velocity and Acceleration of the wrist yaw will stay the same from the previous value unless updated.

```python
        trajectory_goal = FollowJointTrajectory.Goal()
        trajectory_goal.trajectory.joint_names = ['joint_lift', 'wrist_extension', 'joint_wrist_yaw', 'joint_gripper_finger_left']
        trajectory_goal.trajectory.points = [point0, point1, point2, point3, point4, point5]
        trajectory_goal.trajectory.header.frame_id = 'base_link'
        self.trajectory_client.send_goal_async(trajectory_goal)
        self.get_logger().info("Goal sent")
```

Set *trajectory_goal* as a `FollowJointTrajectory.Goal()` and define the joint names as a list. Then `trajectory_goal.trajectory.points` is defined by a list of the 6 points.
