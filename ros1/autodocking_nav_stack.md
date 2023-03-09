# Autodocking with Nav Stack
Wouldn't it be awesome if after a hard day's work, Stretch would just go charge itself without you having to worry about it? In this tutorial we will explore an experimental code that allows Stretch to locate a charging station and charge itself autonomously. This demo will build on top of some of the tutorials that we have explored earlier like [ArUco detection](https://docs.hello-robot.com/0.2/stretch-tutorials/ros1/aruco_marker_detection/), [base teleoperation](https://docs.hello-robot.com/0.2/stretch-tutorials/ros1/example_1/) and using the [Nav Stack](https://docs.hello-robot.com/0.2/stretch-tutorials/ros1/navigation_stack/). Be sure to check them out.

## Docking Station
The [Stretch Docking Station](https://github.com/hello-robot/stretch_tool_share/tree/master/tool_share/stretch_docking_station) is a Stretch accessory that allows one to autonomously charge the robot. The top part of the docking station has an ArUco marker number 245 from the 6x6, 250 dictionary. To understand why this is important, refer to [this](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html) handy guide provided by OpenCV. This marker enables Stretch to accurately locate the docking station in its environment. The docking station also has a Y-shaped recess in its base plate to guide the caster wheel of the robot towards the charging port in case of minor misalignments while backing up. Overall, it's a minimal yet effective way to allow Stretch to charge itself.

## Behaviour Trees
Traditionally, high level task planning has been achieved using Finite State Machines or FSMs which break down each functional element of the task into states that have to be traversed in a cyclic manner to accomplish the major task. This approach has recently gone out of vogue in favour of [Behavior Trees](https://robohub.org/introduction-to-behavior-trees/) or BTs. BTs, also known as Directed Acyclic Graphs, have been popularized through their use in the gaming industry to impart complex behaviors to NPCs. BTs organize behaviors in a tree representation where the control flow is achieved not through cyclic state transitions but in a tree traversal fashion. Additionally, conditions and actions form distinct leafs in the tree which results in better modularity. This ensures that augmenting behaviors by way of additional leafs in the tree does not require a restructuring of previous and following leafs but only the final tree graph. This also means that actions in complex behaviors can be developed independently resulting in higher scalability.

## Py-trees
We decided to implement this demo using the open-source behvaior trees library called [py-trees](https://py-trees.readthedocs.io/en/devel/introduction.html) because of its following features:

- Open-source
- Pythonic for quicker adoption and turnaround
- Well-documented to enable users to self-learn
- Scalable to work equally well on all levels of our software stack, including ROS 2
- Well-supported to allow us to continue to build on top for the forseebale future

## Prerequisites

1. Since this demo uses the ROS Navigation Stack to navigate to the docking station, it requires a pregenerated map that can be utilized to localize the robot. To know how to generate the map, refer to the [Nav Stack]() tutorial.
2. To understand the underlying implementation, it is important to review the concept of Behavior Trees. Although this demo does not use some of its more useful features such as a blackboard or tree visualization, a preliminary read on the concept should be sufficient to understand what's happening under the hood.
3. This demo requires the Behavior Tree library called py-trees to be installed. To do this, execute the following command:

```{.bash .shell-prompt}
sudo apt install ros-noetic-py-trees-ros ros-noetic-rqt-py-trees
```

Once you have the above covered, we are ready to setup the demo.

## Setup and Launch
The demo requires the docking station to be rested against a wall with the charger connected to the back. It is also necessary for the robot to be close to the origin of the map for the robot to have the correct pose estimate at startup. If not, the pose estimate will have to be supplied manually using the `2D Pose Estimate` button in RViz as soon as the demo starts.

To launch the demo, execute the following command:
```{.bash .shell-prompt}
roslaunch stretch_demos autodocking.launch
```

Add GIF of autodocking in action

## Theory
Add image of the autodocking BT

Add image of simple controller setup

## Code Breakdown
Let's jump into the code to see how things work under the hood. Follow along [here]() to have a look at the entire script.

We start off by importing the dependencies. The ones of interest are those relating to py-trees and the various behaviour classes in autodocking.autodocking_behaviours, namely, MoveBaseActionClient, CheckTF and VisualServoing. We also created custom ROS action messages for the ArucoHeadScan action defined in the action directory of stretch_demos package.
```python
import py_trees
import py_trees_ros
import py_trees.console as console
import rospy
import sys
import functools
from autodocking.autodocking_behaviours import MoveBaseActionClient
from autodocking.autodocking_behaviours import CheckTF
from autodocking.autodocking_behaviours import VisualServoing
from stretch_core.msg import ArucoHeadScanAction, ArucoHeadScanGoal
from geometry_msgs.msg import Pose
from sensor_msgs.msg import BatteryState
import hello_helpers.hello_misc as hm
```

The main class of this script is the AutodockingBT class which is a subclass of HelloNode.
```python
class AutodockingBT(hm.HelloNode):
```

The create_root() method is where we construct the autodocking behavior tree. As seen in the figure above, the root node of the behavior tree is a sequence node called `autodocking_seq_root`. This sequence node executes its child nodes sequentially until either all of them succeed or one of them fails. It begins by executing its first child node called `dock_found_fb`. 

The `dock_found_fb` node is a fallback node which starts executing from the left-most child node and only executes the following child node if the child node preceeding it fails. This is useful for executing recovery behaviors in case a required condition is not met. Similarly, `at_predock_fb` and `charging_fb` are also fallback nodes.
```python
    def create_root(self):
        # behaviours
        
        autodocking_seq_root = py_trees.composites.Sequence("autodocking")
        dock_found_fb = py_trees.composites.Selector("dock_found_fb")
        at_predock_fb = py_trees.composites.Selector("at_predock_fb")
        charging_fb = py_trees.composites.Selector("charging_fb")
```

The node `predock_found_sub` is a behavior node which is a child of the `dock_found_fb` fallback node. This node subscribes to the `/predock_pose` topic to check for incoming messages. It returns SUCCESS when a predock pose is being published. At the start of the demo, since the robot likely does not have the docking station in its view, no messages are received on this topic. The fallback to this condition would be to scan the area using the head camera. The `head_scan_action` action node sends a goal to the `ArucoHeadScan` server to look for the marker number 245 at a camera tilt angle of -0.68 rads through ArucoHeadScanGoal(). If this action returns SUCCESS, we start receiving the predock_pose.
```python
        predock_found_sub = py_trees_ros.subscribers.CheckData(
            name="predock_found_sub?",
            topic_name='/predock_pose',
            expected_value=None,
            topic_type=Pose,
            fail_if_no_data=True,fail_if_bad_comparison=False)

        aruco_goal = ArucoHeadScanGoal()
        aruco_goal.aruco_id = 245
        aruco_goal.tilt_angle = -0.68
        aruco_goal.publish_to_map = True
        aruco_goal.fill_in_blindspot_with_second_scan = False
        aruco_goal.fast_scan = False
        head_scan_action = py_trees_ros.actions.ActionClient( # Publishes predock pose to /predock_pose topic and tf frame called /predock_pose
            name="ArucoHeadScan",
            action_namespace="ArucoHeadScan",
            action_spec=ArucoHeadScanAction,
            action_goal=aruco_goal,
            override_feedback_message_on_running="rotating"
        )
```

Next, we want to move to the predock_pose. We do this by passing the predock pose as a goal to the Move Base action server using the `predock_action`. This is followed by the `dock_action` action node which uses a mock visual servoing controller to back up into the docking station. This action uses the predock pose to align the robot to the docking station. Internally, it publishes Twist messages on the /stretch/cmd_vel topic after computing the linear and angular velocities based on the postional and angular errors as defined by the simple controller in the image above.
```python
        predock_action = MoveBaseActionClient(
            self.tf2_buffer,
            name="predock_action",
            override_feedback_message_on_running="moving"
        )
        invert_predock = py_trees.decorators.SuccessIsFailure(name='invert_predock', child=predock_action)

        dock_action = VisualServoing(
            name='dock_action',
            source_frame='docking_station',
            target_frame='charging_port',
            override_feedback_message_on_running="docking"
        )
```

Finally, we define the `is_charging_sub` behavior node which, like the `predock_found_sub`, subscribes to the `\battery` topic and checks for the `present` attribute of the BatteryState message to turn True. If this behavior node returns SUCCEES, the root node returns SUCCEESS as well.
```python
        is_charging_sub = py_trees_ros.subscribers.CheckData(
            name="battery_charging?",
            topic_name='/battery',
            variable_name='present',
            expected_value=True,
            topic_type=BatteryState,
            fail_if_no_data=True,fail_if_bad_comparison=True)
```

Once we have defined the behavior nodes, the behavior tree can be constructed using the add_child() or add_children() methods. The root node is then returned to the caller.
```python
        autodocking_seq_root.add_children([dock_found_fb, at_predock_fb, dock_action, charging_fb])
        dock_found_fb.add_children([predock_found_sub, head_scan_action])
        at_predock_fb.add_children([predock_action])
        charging_fb.add_children([is_charging_sub, invert_predock])
        return autodocking_seq_root
```

The main() method is where the behavior tree is ticked. First, we create an instance of the BehaviorTree class using the root of the tree we created in the create_root() method. The tick_tock() method then ticks the behavior nodes in order until the root either returns a SUCCESS or a FAILURE.
```python
    def main(self):
        """
        Entry point for the demo script.
        """
        hm.HelloNode.main(self, 'autodocking', 'autodocking')

        root = self.create_root()
        self.behaviour_tree = py_trees_ros.trees.BehaviourTree(root)
        rospy.on_shutdown(functools.partial(self.shutdown, self.behaviour_tree))
        if not self.behaviour_tree.setup(timeout=15):
            console.logerror("failed to setup the tree, aborting.")
            sys.exit(1)
        
        def print_tree(tree):
            print(py_trees.display.unicode_tree(root=tree.root, show_status=True))

        try:
            self.behaviour_tree.tick_tock(
                500
                # period_ms=500,
                # number_of_iterations=py_trees.trees.CONTINUOUS_TICK_TOCK,
                # pre_tick_handler=None,
                # post_tick_handler=print_tree
            )
        except KeyboardInterrupt:
            self.behaviour_tree.interrupt()
```
