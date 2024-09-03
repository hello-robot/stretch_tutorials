# Motion Commands in ROS2

## Quickstart

Sending motion commands is as easy as:

 1. Launch the ROS2 driver in a terminal:
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

You can also write a ROS2 node to send motion commands:

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

Copy the above into a file called "example.py" and run it using:

```{.bash .shell-prompt .copy}
python3 example.py
```
