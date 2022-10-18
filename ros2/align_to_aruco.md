# Align to ArUco
ArUco markers are a type of fiducials that are used extensively in robotics for identification and pose estimation. In this tutorial we will learn how to identify ArUco markers with the ArUco detection node and enable Stretch to navigate and align itself with respect to the marker.

## ArUco Detection
Stretch uses the OpenCV ArUco detection library and is configured to identify a specific set of ArUco markers belonging to the 6x6, 250 dictionary. To understand why this is important, please refer to this handy guide provided by OpenCV.

Stretch comes preconfigured to identify ArUco markers. The ROS node that enables this is the detect_aruco_markers node in the stretch_core package. Thanks to this node, identifying and estimating the pose of a marker is as easy as pointing the camera at the marker and running the detection node. It is also possible and quite convenient to visualize the detections with RViz.

## Computing Transformations
If you have not already done so, now might be a good time to review the tf_tranformation tutorial. Go on, we can wait…
Now that we know how to program stretch to return the transform between known reference frames, we can use this knowledge to compute the transform between the detected marker and the robot base_link.. Enter TF transformations! From its current pose, for Stretch to align itself in front of the marker, we need to command it to reach there. But even before that, we need to program Stretch to know the goal pose. We define the goal pose to be 0.5 metre outward from the marker in the marker negative y-axis (Green axis). This is easier to visualize through the figure below.

<p align="center">
    <img src="https://user-images.githubusercontent.com/97639181/196329130-986c3af9-6dc9-4d9b-8a74-626d12b1d82c.png" width="400">
</p>

<!-- Add images to show alignment and tranformations -->

By monitoring the /aruco/marker_array and /aruco/axes topics, we can visualize the markers in RViz. The detection node also publishes the tf pose of the detected markers. This can be visualized by using the TF plugin and selecting the detected marker to inspect the pose. Next, we will use exactly that to compute the transform between the detected marker and the base_link of the robot.

Now, we can compute the transformation from the robot base_link frame to the goal pose and pass this is an SE3 pose to the mobile base.

Since we want Stretch to align with respect to the marker we define a 0.5m offset in the marker y-axis where Stretch would come to a stop. At the same time, we also want Stretch to point the arm towards the marker so as to make the subsequent manipulation tasks easier to accomplish. This would result in the end pose of the base_link as shown below. Sweet! The next task is to plan a trajectory for the mobile base to reach this end pose. We do this in three steps:
1. Turn theta degrees towards the goal position
2. Travel straight to the goal position
3. Turn phi degrees to attain the goal orientation

Luckily, we know how to command Stretch to execute a trajectory using the joint trajectory server. If you are just starting, have a look at the tutorial to know how to command Stretch using the Joint trajectory Server.

## Warnings
Since we won't be using the arm for this demo, it's safer to stow Stretch's arm in. Execute the command:
```bash
stretch_robot_stow.py
```

## See It In Action
First, we need to point the camera towards the marker. To do this, you could use the keyboard teleop node.
When you are ready, execute the following command:
```bash
ros2 launch stretch_core align_to_aruco.launch.py
```

<p align="center">
    <img src="https://user-images.githubusercontent.com/97639181/196327520-7a3b6743-8e2c-4ec0-8603-ba9baff7aa34.gif" width="400">
</p>

## Code Breakdown
Let's jump into the code to see how things work under the hood. Follow along [here](https://github.com/hello-robot/stretch_ros2/blob/galactic/stretch_core/stretch_core/align_to_aruco.py) to have a look at the entire script.

We make use of two separate Python classes for this demo. The FrameListener() class is derived from the Node class and is the place where we compute the TF transformations. For an explantion of this class, you can refer to the TF listener tutorial.
```python
class FrameListener(Node):
```

The AlignToAruco() class is where we command Stretch to the pose goal. This class is derived from the FrameListener class so that they can both share the node instance.
```python
class AlignToAruco(FrameListener):
```

The constructor initializes the Joint trajectory action client. It also initialized the attribute called offset that determines the end distance between the marker and the robot.
```python
    def __init__(self, node, offset=0.75):
        self.trans_base = TransformStamped()
        self.trans_camera = TransformStamped()
        self.joint_state = JointState()
        self.offset = offset
        self.node = node

        self.trajectory_client = ActionClient(self.node, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory')
        server_reached = self.trajectory_client.wait_for_server(timeout_sec=60.0)
        if not server_reached:
            self.node.get_logger().error('Unable to connect to arm action server. Timeout exceeded.')
            sys.exit()
```

The joint_states_callback is the callback method that receives the most recent joint state messages published on the /stretch/joint_states topic.
```python
    def joint_states_callback(self, joint_state):
        self.joint_state = joint_state
```

The copute_difference() method is where we call the get_transform() method from the FrameListener class to compute the difference between the base_link and base_right frame with an offset of 0.5 m in the negative y-axis.
```python
    def compute_difference(self):
        self.trans_base, self.trans_camera = self.node.get_transforms()
```

To compute the (x, y) coordinates of the SE3 pose goal, we compute the transformation as explained above here.
```python
        R = quaternion_matrix((x, y, z, w))
        P_dash = np.array([[0], [-self.offset], [0], [1]])
        P = np.array([[self.trans_base.transform.translation.x], [self.trans_base.transform.translation.y], [0], [1]])

        X = np.matmul(R, P_dash)
        P_base = X + P

        base_position_x = P_base[0, 0]
        base_position_y = P_base[1, 0]
```

From this, it is relatively straightforward to compute the angle phi and the euclidean distance dist. We then compute the angle z_rot_base to perform the last angle correction.
```python
        phi = atan2(base_position_y, base_position_x)
        
        dist = sqrt(pow(base_position_x, 2) + pow(base_position_y, 2))

        x_rot_base, y_rot_base, z_rot_base = euler_from_quaternion([x, y, z, w])
        z_rot_base = -phi + z_rot_base + 3.14159
```

The align_to_marker() method is where we command Stretch to the pose goal in three steps using the Joint Trajectory action server. For an explanation on how to form the trajectory goal, you can refer to the Follow Joint Trajectory tutorial.
```python
    def align_to_marker(self):
```

The End! If you want to work with a different ArUco marker than the one we used in this tutorial, you can do so by changing line 44 in the [code](https://github.com/hello-robot/stretch_ros2/blob/galactic/stretch_core/stretch_core/align_to_aruco.py#L44) to the one you wish to detect. Also, don't forget to add the marker in the ArUco marker dictionary.
