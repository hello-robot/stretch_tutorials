# Surface Cleaning Demo

FUNMAP is a hardware-targeted perception, planning, and navigation framework developed by Hello Robot for ROS developers and researchers. Some of the key features provided by FUNMAP include cliff detection, closed-loop navigation, and mapping. In this tutorial, we will explore a surface cleaning demo using FUNMAP.

## Motivation

Through this demo, we demonstrate flat surface detection using the head camera, and demonstrate surface cleaning with navigation using FUNMAP. The robot is teleoperated to have a flat surface at a reasonable height in the view of the robot’s camera. The arm of the robot is fully retracted to get a good and entire view of the surface of interest. We have observed reliable depth inference and object detection whenever the surface to be cleaned is placed in front of a dark background.

## Workspace Setup

Ideally, this demo requires the surface to be present at half the height of a Stretch. Teleoperate your robot so as to get a good view of it. The end-effector of the robot is aligned with the target object. The arm is fully retracted to get a good view of the scene and reliable depth computation. The lift position is slightly below the height of the target surface. The demo works best in dim lighting conditions. Finally, ensure that there is no flat vertical surface, such as a wall, close behind the surface to be cleaned. You can use RViz to visualize the pointcloud data being captured by the camera.

## How-to-run

After building and sourcing the workspace, home the robot:

```bash
stretch_robot_home.py
```

This ensures that the underlying stretch_body package knows the exact joint limits and provides the user with a good starting joint configuration.

After homing, launch the surface cleaning demo:

```bash
ros2 launch stretch_demos clean_surface.launch.py
```

This command will launch stretch_driver, stretch_funmap, keyboard_teleop, and the clean_surface nodes. 

In a new terminal, launch keyboard teleoperation:
```bash
ros2 run stretch_core keyboard_teleop --ros-args -p clean_surface_on:=true
```

You will be presented with a keyboard teleoperation menu in a new terminal window. Use key commands to get the Stretch configured as per the above workspace setup guidelines. Once the robot is ready, press ‘/’ or ‘?’ to trigger surface cleaning.

## Code Explained

The clean_surface node uses the joint_trajectory_server inside stretch_core to send out target joint positions.

```python
self.trajectory_client = ActionClient(self,
    FollowJointTrajectory,
    '/stretch_controller/follow_joint_trajectory',
    callback_group=self.callback_group
)
server_reached = self.trajectory_client.wait_for_server(timeout_sec=60.0)
if not server_reached:
    self.get_logger().error('Unable to connect to joint_trajectory_server. Timeout exceeded.')
    sys.exit()
```

Additionally, the node also subscribes to RealSense's depth cloud, and Stretch’s joint state topics.

```python
self.joint_states_subscriber = self.create_subscription(JointState,
    '/stretch/joint_states',
    callback=self.joint_states_callback,
    qos_profile=10,
    callback_group=self.callback_group
)

self.point_cloud_subscriber = self.create_subscription(PointCloud2,
    '/camera/depth/color/points',
    callback=self.point_cloud_callback,
    qos_profile=10, 
    callback_group=self.callback_group
)
```

Whenever the user triggers the clean surface service, Stretch scans the field of view for candidate flat surfaces and generates a surface wiping plan represented by a list of joint position goals. The wiping plan is generated using the ManipulationView class:

```python
self.log.info("Cleaning initiating!")

tool_width_m = 0.08
tool_length_m = 0.08
step_size_m = 0.04
min_extension_m = 0.01
max_extension_m = 0.5
   	 
self.look_at_surface()
strokes, simple_plan, lift_to_surface_m = self.manipulation_view.get_surface_wiping_plan(self.tf2_buffer,
    tool_width_m,
    tool_length_m,
    step_size_m
)
self.log.info("Plan:" + str(simple_plan))

self.log.info('********* lift_to_surface_m = {0} **************'.format(lift_to_surface_m))
```

The ManipulationView class holds a max-height image as an internal representation of the surroundings. It creates a segmentation mask to generate a set of linear forward-backward strokes that cover the entire detected surface. See `stretch_funmap/segment_max_height_image.py` to understand how a flat surface is detected. Obstacles, if present on the surface, are inflated using dilation and erosion to create a margin of safety. These strokes, in the image frame, are then transformed into the base_link frame using tf2. This is how we obtain the wiping plan.

After generating the wiping plan, the node validates this plan by asserting that the required strokes are greater than zero and the surface is reachable by adjusting the lift and wrist positions:

```python
if True and (strokes is not None) and (len(strokes) > 0):
    if (self.lift_position is not None) and (self.wrist_position is not None):
        above_surface_m = 0.1
        lift_above_surface_m = self.lift_position + lift_to_surface_m + above_surface_m
```

After validation, the node then proceeds to execute the wiping plan in a sequential manner.

## Results and Expectations

This demo serves as an experimental setup to explore surface cleaning with Stretch. Please be advised that this code is not expected to work perfectly. Some of the shortcomings of the demo include:

- The area of surface being cleaned is dependent on the fill rate of the RealSense point cloud. If the segemented cleaning surface has too many holes, Stretch might only attempt to clean a fraction of the total area.

- As each Stretch is unique in its contacts thresholds after calibration, your robot might attempt too aggressively to press down the cloth upon the surface. Additonal tuning might be necessary.

Users are encouraged to try this demo and submit improvements.
