# Align to ArUco
ArUco markers are a type of fiducials that are used extensively in robotics for identification and pose estimation. In this tutorial we will learn how to identify ArUco markers with the ArUco detection node and enable Stretch to navigate and align itself with respect to the marker.

There are three main parameters to creating an ArUco tag:
- Dictionary Size (Use 6x6_250 when using Stretch tutorials or ROS2 nodes, this is explained more [below](./#aruco-detection))
- ID (A unique identifier for the tag, see [Create a New ArUco Marker](./#create-a-new-aruco-marker) for reserved ID's)
- Marker Size (The physical dimensions of the tag in mm, this can be customized freely.)

## The ArUco Marker Dictionary
When defining the ArUco markers on Stretch, hello robot utilizes a YAML file, [stretch_marker_dict.yaml](https://github.com/hello-robot/stretch_ros/blob/master/stretch_core/config/stretch_marker_dict.yaml), that holds the information about the markers.

If [detect_aruco_markers](https://github.com/hello-robot/stretch_ros/blob/master/stretch_core/nodes/detect_aruco_markers) node doesn’t find an entry in [stretch_marker_dict.yaml](https://github.com/hello-robot/stretch_ros/blob/master/stretch_core/config/stretch_marker_dict.yaml) for a particular ArUco marker ID number, it uses the default entry. For example, most robots have shipped with the following default entry:

```yaml
'default':
  'length_mm': 24
  'use_rgb_only': False
  'name': 'unknown'
  'link': None
```

and the following entry for the ArUco marker on the top of the wrist

```yaml
'133':
  'length_mm': 23.5
  'use_rgb_only': False
  'name': 'wrist_top'
  'link': 'link_aruco_top_wrist'
```


**Dictionary Breakdown**

```yaml
'133':
```

 The dictionary key for each entry is the ArUco marker’s ID number or `default`. For example, the entry shown above for the ArUco marker on the top of the wrist assumes that the marker’s ID number is `133`.

```yaml
'length_mm': 23.5
```

The `length_mm` value used by [detect_aruco_markers](https://github.com/hello-robot/stretch_ros/blob/master/stretch_core/nodes/detect_aruco_markers) is important for estimating the pose of an ArUco marker.

!!! note
    If the actual width and height of the marker do not match this value, then pose estimation will be poor. Thus, carefully measure custom Aruco markers.

```yaml
'use_rgb_only': False
```

If `use_rgb_only` is `True`, [detect_aruco_markers](https://github.com/hello-robot/stretch_ros/blob/master/stretch_core/nodes/detect_aruco_markers) will ignore depth images from the [Intel RealSense D435i depth camera](https://www.intelrealsense.com/depth-camera-d435i/) when estimating the pose of the marker and will instead only use RGB images from the D435i.

```yaml
'name': 'wrist_top'
```

`name` is used for the text string of the ArUco marker’s [ROS Marker](http://docs.ros.org/en/melodic/api/visualization_msgs/html/msg/Marker.html) in the [ROS MarkerArray](http://docs.ros.org/en/melodic/api/visualization_msgs/html/msg/MarkerArray.html) Message published by the [detect_aruco_markers](https://github.com/hello-robot/stretch_ros/blob/master/stretch_core/nodes/detect_aruco_markers) ROS node.

```yaml
'link': 'link_aruco_top_wrist'
```

`link` is currently used by [stretch_calibration](https://github.com/hello-robot/stretch_ros/blob/master/stretch_calibration/nodes/collect_head_calibration_data). It is the name of the link associated with a body-mounted ArUco marker in the [robot’s URDF](https://github.com/hello-robot/stretch_ros/blob/master/stretch_description/urdf/stretch_aruco.xacro).

It’s good practice to add an entry to [stretch_marker_dict.yaml](https://github.com/hello-robot/stretch_ros/blob/master/stretch_core/config/stretch_marker_dict.yaml) for each ArUco marker you use.

### Create a New ArUco Marker
At Hello Robot, we’ve used the following guide when generating new ArUco markers.

We generate ArUco markers using a 6x6-bit grid (36 bits) with 250 unique codes. This corresponds with[ DICT_6X6_250 defined in OpenCV](https://docs.opencv.org/3.4/d9/d6a/group__aruco.html). We generate markers using this [online ArUco marker generator](https://chev.me/arucogen/) by setting the Dictionary entry to 6x6 and then setting the Marker ID and Marker size, mm as appropriate for the specific application. We strongly recommend measuring the actual marker by hand before adding an entry for it to [stretch_marker_dict.yaml](https://github.com/hello-robot/stretch_ros/blob/master/stretch_core/config/stretch_marker_dict.yaml).

We select marker ID numbers using the following ranges.

* 0 - 99: reserved for users
  * 100 - 249: reserved for official use by Hello Robot Inc.
  * 100 - 199: reserved for robots with distinct sets of body-mounted markers
    * Allows different robots near each other to use distinct sets of body-mounted markers to avoid confusion. This could be valuable for various uses of body-mounted markers, including calibration, visual servoing, visual motion capture, and multi-robot tasks.
    * 5 markers per robot = 2 on the mobile base + 2 on the wrist + 1 on the shoulder
    * 20 distinct sets = 100 available ID numbers / 5 ID numbers per robot
  * 200 - 249: reserved for official accessories
    * 245 for the prototype docking station
    * 246-249 for large floor markers

When coming up with this guide, we expected the following:

* Body-mounted accessories with the same ID numbers mounted to different robots could be disambiguated using the expected range of 3D locations of the ArUco markers on the calibrated body.
* Accessories in the environment with the same ID numbers could be disambiguated using a map or nearby observable features of the environment.

> Note: This section of the tutorial is borrowed from its [ROS1 predecessor](https://github.com/hello-robot/stretch_tutorials/blob/master/ros1/aruco_marker_detection.md).

## ArUco Detection
Stretch uses the OpenCV ArUco detection library and is configured to identify a specific set of ArUco markers belonging to the 6x6, 250 dictionary. To understand why this is important, please refer to [this](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html) handy guide provided by OpenCV.

Stretch comes preconfigured to identify ArUco markers. The ROS node that enables this is the [detect_aruco_markers node](https://github.com/hello-robot/stretch_ros2/blob/humble/stretch_core/stretch_core/detect_aruco_markers.py) in the stretch_core package. Thanks to this node, identifying and estimating the pose of a marker is as easy as pointing the camera at the marker and running the detection node. It is also possible and quite convenient to visualize the detections with RViz.

## Computing Transformations
If you have not already done so, now might be a good time to review the [tf listener](https://docs.hello-robot.com/latest/ros2/example_10/) tutorial. Go on, we can wait…
Now that we know how to program stretch to return the transform between known reference frames, we can use this knowledge to compute the transform between the detected marker and the robot base_link. From its current pose, for Stretch to align itself in front of the marker, we need to command it to reach there. But even before that, we need to program Stretch to know the goal pose. We define the goal pose to be 0.5 metre outward from the marker in the marker negative y-axis (Green axis). This is easier to visualize through the figure below.

<p align="center">
    <img src="https://user-images.githubusercontent.com/97639181/196329130-986c3af9-6dc9-4d9b-8a74-626d12b1d82c.png" width="400">
</p>

<!-- Add images to show alignment and tranformations -->

By monitoring the /aruco/marker_array and /aruco/axes topics, we can visualize the markers in RViz. The detection node also publishes the tf pose of the detected markers. This can be visualized by using the TF plugin and selecting the detected marker to inspect the pose. Next, we will use exactly that to compute the transform between the detected marker and the base_link of the robot.

Now, we can compute the transformation from the robot `base_link` frame to the `goal` pose and pass this as a 2D special Euclidian (SE2) pose to the mobile base.

Since we want Stretch to stop at a fixed distance with respect to the marker, we define a 0.75m offset in the marker y-axis where Stretch would come to a stop. 

At the same time, we also want Stretch to align its orientation to point its arm towards the marker so as to make the subsequent manipulation tasks easier to accomplish. This would result in the end pose of the base_link as shown in the above figure. Sweet! 

The next task is to generate a simple motion plan for the mobile base to reach this end pose. We do this in three steps:
1. Turn theta degrees towards the goal position. This would be the angle formed between the robot x-axis and the line connecting the start and the goal positions.
2. Travel straight to the goal position. This would be the euclidean distance between the start and the goal positions.
3. Turn phi degrees to attain the goal orientation. This would be the correction angle necessary to align the robot y-axis with the marker x-axis.

Luckily, we know how to command Stretch to execute a trajectory using the joint trajectory server. If you are just starting, have a look at the [Follow Joint Trajectory Commands](https://docs.hello-robot.com/0.2/stretch-tutorials/ros2/follow_joint_trajectory/) tutorial to know how to command Stretch using the Joint trajectory Server.

!!! warning
    Since we won't be using the arm for this demo, it's safer to stow Stretch's arm in. Run `stretch_robot_stow.py` from anywhere on a terminal.

```{.bash .shell-prompt}
stretch_robot_stow.py
```

## See It In Action
First, we need to point the camera towards the marker. To do this, you could use the keyboard teleop node. To do this, run:
```{.bash .shell-prompt}
ros2 launch stretch_core keyboard_teleop.launch.py
```

When you are ready, execute the following command:
```{.bash .shell-prompt}
ros2 launch stretch_core align_to_aruco.launch.py
```
<p align="center">
    <img src="https://user-images.githubusercontent.com/97639181/196327520-7a3b6743-8e2c-4ec0-8603-ba9baff7aa34.gif" width="400">
</p>


#### Special Considerations
1. Since we won't be using the arm for this demo, it's safer to stow Stretch's arm in. Run `stretch_robot_stow.py` from anywhere on a terminal.
2. the ArUco tag needs to be laid flat and rotated to match the image in [Computing Transformations](https://docs.hello-robot.com/latest/ros2/align_to_aruco/#computing-transformations) above.
3. The robot will move 0.75m away from the marker. This means that if the robot is within 0.75m, then it will rotate and move forward until it is at 0.75m. If the robot is too close while rotating, it may collide with its surroundings.
4. If you wish to use a custom ArUco tag, 1. you should [create a tag](./#create-a-new-aruco-marker). 2. print it so you can place it in your environment. 3. Duplicate the `131` `base_right` marker in [stretch_marker_dict.yml](https://github.com/hello-robot/stretch_ros/blob/master/stretch_core/config/stretch_marker_dict.yaml) and replace `131` with your marker's ID, `base_right` with a new name, and the `marker_size`, if it is different. 4. Lastly, you will need to pass the `aruco_tag_name` rosparam to the `stretch_aruco.py` node. You can do this either by editting the `align_to_aruco.launch.py` file or following the [Manual Launch](./#manual-launch) steps below.


### Manual Launch

Instead of using the launch file, you can also run the following commands. This allows you to specify a custom ArUco tag name to the `align_to_aruco.py` node. The custom tag name must be defined in [stretch_marker_dict.yml](https://github.com/hello-robot/stretch_ros/blob/master/stretch_core/config/stretch_marker_dict.yaml):

```
roslaunch stretch_core stretch_driver.launch
roslaunch stretch_core d435i_low_resolution.launch
roslaunch stretch_core stretch_aruco.launch --ros-args -p aruco_tag_name:=base_right
rosrun rviz rviz -d /home/hello-robot/catkin_ws/src/stretch_tutorials/rviz/aruco_detector_example.rviz
```

## Code Breakdown
Let's jump into the code to see how things work under the hood. Follow along [here](https://github.com/hello-robot/stretch_ros2/blob/humble/stretch_core/stretch_core/align_to_aruco.py) to have a look at the entire script.

We make use of two separate Python classes for this demo. The FrameListener class is derived from the Node class and is the place where we compute the TF transformations. For an explantion of this class, you can refer to the [TF listener](https://docs.hello-robot.com/0.2/stretch-tutorials/ros2/example_10/) tutorial.
```python
class FrameListener(Node):
```

The AlignToAruco class is where we command Stretch to the pose goal. This class is derived from the FrameListener class so that they can both share the node instance.
```python
class AlignToAruco(FrameListener):
```

The constructor initializes the Joint trajectory action client. It also initializes the attribute called offset that determines the end distance between the marker and the robot.
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

The compute_difference() method is where we call the get_transform() method from the FrameListener class to compute the difference between the base_link and base_right frame with an offset of 0.5 m in the negative y-axis.
```python
    def compute_difference(self):
        self.trans_base, self.trans_camera = self.node.get_transforms()
```

To compute the (x, y) coordinates of the SE2 pose goal, we compute the transformation here.
```python
        R = quaternion_matrix((x, y, z, w))
        P_dash = np.array([[0], [-self.offset], [0], [1]])
        P = np.array([[self.trans_base.transform.translation.x], [self.trans_base.transform.translation.y], [0], [1]])

        X = np.matmul(R, P_dash)
        P_base = X + P

        base_position_x = P_base[0, 0]
        base_position_y = P_base[1, 0]
```

From this, it is relatively straightforward to compute the angle **phi** and the euclidean distance **dist**. We then compute the angle z_rot_base to perform the last angle correction.
```python
        phi = atan2(base_position_y, base_position_x)
        
        dist = sqrt(pow(base_position_x, 2) + pow(base_position_y, 2))

        x_rot_base, y_rot_base, z_rot_base = euler_from_quaternion([x, y, z, w])
        z_rot_base = -phi + z_rot_base + 3.14159
```

The align_to_marker() method is where we command Stretch to the pose goal in three steps using the Joint Trajectory action server. For an explanation on how to form the trajectory goal, you can refer to the [Follow Joint Trajectory Commands](https://docs.hello-robot.com/0.2/stretch-tutorials/ros2/follow_joint_trajectory/) tutorial.
```python
    def align_to_marker(self):
```

If you want to work with a different ArUco marker than the one we used in this tutorial, you can do so by changing line 44 in the [code](https://github.com/hello-robot/stretch_ros2/blob/humble/stretch_core/stretch_core/align_to_aruco.py#L130) to the one you wish to detect or pass in that rosparam. Also, don't forget to add the marker in the [stretch_marker_dict.yaml](https://github.com/hello-robot/stretch_ros2/blob/humble/stretch_core/config/stretch_marker_dict.yaml) ArUco marker dictionary.
