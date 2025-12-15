# Motion Commands in ROS 2

## Quickstart

Sending motion commands is as easy as:

 1. Launch the ROS 2 driver in a terminal:
    ```{.bash .shell-prompt .copy}
    ros2 launch stretch_core stretch_driver.launch.py
    ```
 2. Open iPython and type the following code, one line at a time:
    ```python
    import hello_helpers.hello_misc as hm
    node = hm.HelloNode.quick_create('temp')
    node.move_to_pose({'joint_lift': 0.4}, blocking=True)
    node.move_to_pose({'joint_wrist_yaw': 0.0, 'joint_wrist_roll': 0.0}, blocking=True)
    ```

## Writing a node

You can also write a ROS 2 node to send motion commands:

```python
import hello_helpers.hello_misc as hm

class MyNode(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)

    def main(self):
        hm.HelloNode.main(self, 'my_node', 'my_node', wait_for_first_pointcloud=False)

        # my_node's main logic goes here
        self.move_to_pose({'joint_lift': 0.6}, blocking=True)
        self.move_to_pose({'joint_wrist_yaw': -1.0, 'joint_wrist_pitch': -1.0}, blocking=True)

node = MyNode()
node.main()
```

Copy the above into a file called "example.py" and run it using:

```{.bash .shell-prompt .copy}
python3 example.py
```

## Retrieving joint limits

In a terminal, echo the `/joint_limits` topic:

```{.bash .shell-prompt .copy}
ros2 topic echo /joint_limits
```

In a second terminal, request the driver publish the joint limits:

```{.bash .shell-prompt .copy}
ros2 service call /get_joint_states std_srvs/srv/Trigger {}
```

In the first terminal, you'll see a single message get published. It'll look like this:

```yaml
header:
  stamp:
    sec: 1725388967
    nanosec: 818893747
  frame_id: ''
name:
- joint_head_tilt
- joint_wrist_pitch
- joint_wrist_roll
- joint_wrist_yaw
- joint_head_pan
- joint_lift
- joint_arm
- gripper_aperture
- joint_gripper_finger_left
- joint_gripper_finger_right
position:
- -2.0171847360696185
- -1.5707963267948966
- -2.9114955354069467
- -1.3933658823294575
- -4.035903452927122
- 0.0
- 0.0
- -0.1285204486235414
- -0.3757907854489514
- -0.3757907854489514
velocity:
- 0.4908738521234052
- 0.45099035163837853
- 2.9176314585584895
- 4.416586351787409
- 1.7303303287350031
- 1.0966833704348709
- 0.5197662863936018
- 0.34289112948906764
- 1.0026056417808995
- 1.0026056417808995
effort: []
```

We're misusing the [sensor_msgs/JointState](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html) message to publish the joint limits. The `name` array lists out each ranged joint. The `position` array lists the lower bound for each joint. The `velocity` array lists the upper bound. The length of these 3 arrays will be equal, because the index of the joint in the `name` array determines which index the corresponding limits will be in the other two arrays.

The revolute joints will have their limits published in radians, and the prismatic joints will have them published in meters. See the [Hardware Overview](../../getting_started/stretch_hardware_overview/) to see the ranges represented visually.

## Translating and rotating the base 

You can also write a ROS 2 node to send motion commands to the base:

```python
import hello_helpers.hello_misc as hm

class MyNode(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)

    def main(self):
        hm.HelloNode.main(self, 'my_node', 'my_node', wait_for_first_pointcloud=False)

        # translate the base
        self.move_to_pose({'translate_mobile_base': 0.2}, blocking=True)

        # rotate the base
        self.move_to_pose({'rotate_mobile_base': 0.2}, blocking=True)

node = MyNode()
node.main()
```

Copy the above into a file called "example.py" and run it using:

```{.bash .shell-prompt .copy}
python3 example.py
```
