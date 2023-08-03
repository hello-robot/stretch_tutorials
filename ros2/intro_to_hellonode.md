# Introduction to HelloNode

HelloNode is a convenience class for creating a ROS 2 node for Stretch. The most common way to use this class is to extend it. In your extending class, the main funcion would call `HelloNode`'s main function. This would look like:

```python
import hello_helpers.hello_misc as hm

class MyNode(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)

    def main(self):
        hm.HelloNode.main(self, 'my_node', 'my_node', wait_for_first_pointcloud=False)
        # my_node's main logic goes here

node = MyNode()
node.main()
```

There is also a one-liner class method for instantiating a `HelloNode` for easy prototyping. One example where this is handy in sending pose commands from iPython:

```python
# roslaunch the stretch launch file beforehand

import hello_helpers.hello_misc as hm
temp = hm.HelloNode.quick_create('temp')
temp.move_to_pose({'joint_lift': 0.4})
```

#### Attributes

##### `dryrun`

This attribute allows you to control whether the robot actually moves when calling `move_to_pose()`, `home_the_robot()`, `stow_the_robot()`, or other motion methods in this class. When `dryrun` is set to True, these motion methods return immediately. This attribute is helpful when you want to run just the perception/planning part of your node without actually moving the robot. For example, you could replace the following verbose snippet:

```python
# launch the stretch driver launch file beforehand
import hello_helpers.hello_misc as hm
temp = hm.HelloNode.quick_create('temp')
actually_move = False
[...]
if actually_move:
    temp.move_to_pose({'translate_mobile_base': 1.0})
```

to be more consise:

```python
# launch the stretch driver launch file beforehand
import hello_helpers.hello_misc as hm
temp = hm.HelloNode.quick_create('temp')
[...]
temp.dryrun = True
temp.move_to_pose({'translate_mobile_base': 1.0})
```

#### Methods

##### `move_to_pose(pose, blocking=False, custom_contact_thresholds=False, duration=2.0)`

This method takes in a dictionary that describes a desired pose for the robot and communicates with [stretch_driver](../stretch_core/README.md#stretchdrivernodesstretchdriver) to execute it. The basic format of this dictionary is string/number key/value pairs, where the keys are joint names and the values are desired position goals. For example, `{'joint_lift': 0.5}` would put the lift at 0.5m in its joint range. A full list of command-able joints is published to the `/stretch/joint_states` topic. Used within a node extending `HelloNode`, calling this method would look like:

```python
self.move_to_pose({'joint_lift': 0.5})
```

Internally, this dictionary is converted into a [JointTrajectory](https://docs.ros2.org/latest/api/trajectory_msgs/msg/JointTrajectory.html) message that is sent to a [FollowJointTrajectory action](http://docs.ros.org/en/noetic/api/control_msgs/html/action/FollowJointTrajectory.html) server in stretch_driver. This method waits by default for the server to report that the goal has completed executing. However, you can return before the goal has completed by setting the `blocking` argument to False. This can be useful for preempting goals.

When the robot is in `position` mode, if you set `custom_contact_thresholds` to True, this method expects a different format dictionary: string/tuple key/value pairs, where the keys are still joint names, but the values are `(position_goal, effort_threshold)`. The addition of a effort threshold enables you to detect when a joint has made contact with something in the environment, which is useful for manipulation or safe movements. For example, `{'joint_arm': (0.5, 20)}` commands the telescoping arm fully out (the arm is nearly fully extended at 0.5 meters) but with a low enough effort threshold (20% of the arm motor's max effort) that the motor will stop when the end of arm has made contact with something. Again, in a node, this would look like:

```python
self.move_to_pose({'joint_arm': (0.5, 40)}, custom_contact_thresholds=True)
```

When the robot is in `trajectory` mode, if you set argument `duration` as `ts`, this method will ensure that the target joint positions are achieved over `ts` seconds. For example, the below would put the lift at 0.5m from its current position in `5.0` seconds:

```python
self.move_to_pose({'joint_lift': 0.5}, duration=5.0)
```

##### `home_the_robot()`

This is a convenience method to interact with the driver's [`/home_the_robot` service](../stretch_core/README.md#home_the_robot-std_srvstrigger).

##### `stow_the_robot()`

This is a convenience method to interact with the driver's [`/stow_the_robot` service](../stretch_core/README.md#stow_the_robot-std_srvstrigger).

##### `stop_the_robot()`

This is a convenience method to interact with the driver's [`/stop_the_robot` service](../stretch_core/README.md#stop_the_robot-std_srvstrigger).

##### `get_tf(from_frame, to_frame)`

Use this method to get the transform ([geometry_msgs/TransformStamped](https://docs.ros2.org/latest/api/geometry_msgs/msg/TransformStamped.html)) between two frames. This method is blocking. For example, this method can do forward kinematics from the base_link to the link between the gripper fingers, link_grasp_center, using:

```python
# launch the stretch driver launch file beforehand

import hello_helpers.hello_misc as hm
temp = hm.HelloNode.quick_create('temp')
t = temp.get_tf('base_link', 'link_grasp_center')
print(t.transform.translation)
```

##### `get_robot_floor_pose_xya(floor_frame='odom')`

Returns the current estimated x, y position and angle of the robot on the floor. This is typically called with respect to the odom frame or the map frame. x and y are in meters and the angle is in radians.

```python
# launch the stretch driver launch file beforehand

import hello_helpers.hello_misc as hm
temp = hm.HelloNode.quick_create('temp')
t = temp.get_robot_floor_pose_xya(floor_frame='odom')
print(t)
```

##### `main(node_name, node_topic_namespace, wait_for_first_pointcloud=True)`

When extending the `HelloNode` class, call this method at the very beginning of your `main()` method. This method handles setting up a few ROS components, including registering the node with the ROS server, creating a TF listener, creating a [FollowJointTrajectory](http://docs.ros.org/en/noetic/api/control_msgs/html/action/FollowJointTrajectory.html) client for the [`move_to_pose()`](#movetoposepose-returnbeforedonefalse-customcontactthresholdsfalse-customfullgoalfalse) method, subscribing to depth camera point cloud topic, and connecting to the quick-stop service.

Since it takes up to 30 seconds for the head camera to start streaming data, the `wait_for_first_pointcloud` argument will get the node to wait until it has seen camera data, which is helpful if your node is processing camera data.

##### `quick_create(name, wait_for_first_pointcloud=False)`

A class level method for quick testing. This allows you to avoid having to extend `HelloNode` to use it.

```python
# launch the stretch driver launch file beforehand

import hello_helpers.hello_misc as hm
temp = hm.HelloNode.quick_create('temp')
temp.move_to_pose({'joint_lift': 0.4})
```

#### Subscribed Topics

##### /camera/depth/color/points ([sensor_msgs/PointCloud2](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud2.html))

Provides a point cloud as currently seen by the Realsense depth camera in Stretch's head. Accessible from the `self.point_cloud` attribute.

```python
# launch the stretch driver launch file beforehand

import hello_helpers.hello_misc as hm
temp = hm.HelloNode.quick_create('temp', wait_for_first_pointcloud=True)
print(temp.point_cloud)
```

##### /stretch/joint_states ([sensor_msgs/JointState](https://docs.ros2.org/latest/api/sensor_msgs/msg/JointState.html))

Provides the current state of robot joints that includes joint names, positions, velocities, efforts. Accessible from the `self.joint_state` attribute.
```python
print(temp.joint_state)
```

##### /mode ([std_msgs/String](https://docs.ros2.org/latest/api/std_msgs/msg/String.html))

Provides the mode the stretch driver is currently in. Possible values include `position`, `trajectory`, `navigation`, `homing`, `stowing`.
```python
print(temp.mode)
```

##### /tool ([std_msgs/String](https://docs.ros2.org/latest/api/std_msgs/msg/String.html))

Provides the end of arm tool attached to the robot.
```python
print(temp.tool)
```

#### Subscribed Services

##### /stop_the_robot ([std_srvs/Trigger](https://docs.ros2.org/latest/api/std_srvs/srv/Trigger.html))

Provides a service to quickly stop any motion currently executing on the robot.

```python
# launch the stretch driver launch file beforehand

from std_srvs.srv import TriggerRequest
import hello_helpers.hello_misc as hm
temp = hm.HelloNode.quick_create('temp')
temp.stop_the_robot_service(TriggerRequest())
```

##### /stow_the_robot ([std_srvs/Trigger](https://docs.ros2.org/latest/api/std_srvs/srv/Trigger.html))

Provides a service to stow the robot arm.

```python
# launch the stretch driver launch file beforehand

from std_srvs.srv import TriggerRequest
import hello_helpers.hello_misc as hm
temp = hm.HelloNode.quick_create('temp')
temp.stow_the_robot_service(TriggerRequest())
```

##### /home_the_robot ([std_srvs/Trigger](https://docs.ros2.org/latest/api/std_srvs/srv/Trigger.html))

Provides a service to home the robot joints.

```python
# launch the stretch driver launch file beforehand

from std_srvs.srv import TriggerRequest
import hello_helpers.hello_misc as hm
temp = hm.HelloNode.quick_create('temp')
temp.home_the_robot_service(TriggerRequest())
```

##### /switch_to_trajectory_mode ([std_srvs/Trigger](https://docs.ros2.org/latest/api/std_srvs/srv/Trigger.html))

Provides a service to quickly stop any motion currently executing on the robot.

```python
# launch the stretch driver launch file beforehand

from std_srvs.srv import TriggerRequest
import hello_helpers.hello_misc as hm
temp = hm.HelloNode.quick_create('temp')
temp.switch_to_trajectory_mode_service(TriggerRequest())
```

##### /switch_to_position_mode ([std_srvs/Trigger](https://docs.ros2.org/latest/api/std_srvs/srv/Trigger.html))

Provides a service to quickly stop any motion currently executing on the robot.

```python
# launch the stretch driver launch file beforehand

from std_srvs.srv import TriggerRequest
import hello_helpers.hello_misc as hm
temp = hm.HelloNode.quick_create('temp')
temp.switch_to_position_mode_service(TriggerRequest())
```

##### /switch_to_navigation_mode ([std_srvs/Trigger](https://docs.ros2.org/latest/api/std_srvs/srv/Trigger.html))

Provides a service to quickly stop any motion currently executing on the robot.

```python
# launch the stretch driver launch file beforehand

from std_srvs.srv import TriggerRequest
import hello_helpers.hello_misc as hm
temp = hm.HelloNode.quick_create('temp')
temp.switch_to_navigation_mode_service(TriggerRequest())
```