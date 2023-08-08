# Object Grasping

FUNMAP is a hardware-targeted perception, planning, and navigation framework developed by Hello Robot for ROS developers and researchers. Some of the key features provided by FUNMAP include cliff detection, closed-loop navigation, and mapping. In this tutorial, we will explore an object-grasping demo using FUNMAP.

## Motivation

Through this demo, we demonstrate grasp candidate computation, grasp planning, and navigation using FUNMAP. An object is placed in front of the Stretch at a reasonable distance and height. The arm of the robot is fully retracted to get a good view of the object. We have observed reliable depth inference and object detection whenever the grasp candidate is placed in front of a dark background.

## Workspace Setup

Ideally, this demo requires the object to be placed at half the height of a Stretch. The end-effector of the robot is aligned with the target object. The arm is fully retracted to get a good view of the scene and reliable depth computation. The lift position is slightly below the height of the target surface. The demo works best in dim lighting conditions. Finally, ensure that there is no flat vertical surface, such as a wall, close behind the object of interest.

## How-to-run

After building and sourcing the workspace, home the robot:

```bash
stretch_robot_home.py
```

This ensures that the underlying stretch_body package knows the exact joint limits and provides the user with a good starting joint configuration.

After homing, launch the object grasping demo:

```bash
ros2 launch stretch_demos grasp_object.launch.py
```

This command will launch stretch_driver, stretch_funmap, and the grasp_object nodes. 

In a new terminal window, start keyboard teleoperation:

```bash
ros2 run stretch_core keyboard_teleop --ros-args -p grasp_object_on:=true
```

You will be presented with a keyboard teleoperation menu in a new terminal window. Use key commands to get the Stretch configured as per the above workspace setup guidelines. Once the robot is ready, press ‘\’ to trigger object grasping.

## Code Explained

The grasp_object node uses the joint_trajectory_server inside stretch_core to send out target joint positions. 

```python
self.trajectory_client = ActionClient(self,
    FollowJointTrajectory,
    '/stretch_controller/follow_joint_trajectory',
    callback_group=self.callback_group
)

server_reached = self.trajectory_client.wait_for_server(
    timeout_sec=60.0
)

if not server_reached:
    self.get_logger().error('Unable to connect to joint_trajectory_server. Timeout exceeded.')
    sys.exit()
```

Additionally, the node also subscribes to RealSense's depth cloud, tool type, and Stretch’s joint state topics.

```python
self.joint_states_subscriber = self.create_subscription(JointState,
    '/stretch/joint_states',
    callback=self.joint_states_callback,
    qos_profile=1,
    callback_group=self.callback_group
)

self.point_cloud_subscriber = self.create_subscription(PointCloud2,
    '/camera/depth/color/points',
    callback=self.point_cloud_callback,
    qos_profile=1,
    callback_group=self.callback_group
)

self.tool_subscriber = self.create_subscription(String,
    '/tool',
    callback=self.tool_callback,
    qos_profile=1,
    callback_group=self.callback_group
)
```

Whenever the user triggers the grasp object service, Stretch scans the field of view for potential grasp candidates.

```python
self.logger.info('Stow the arm.')
self.stow_the_robot()

# 1. Scan surface and find grasp target
self.look_at_surface(scan_time_s = 4.0)
grasp_target = self.manipulation_view.get_grasp_target(self.tf2_buffer)
```

After determining a suitable grasp candidate, the node generates a pregrasp base and end-effector pose.

```python
# 2. Move to pregrasp pose
pregrasp_lift_m = self.manipulation_view.get_pregrasp_lift(grasp_target,
    self.tf2_buffer
)

if self.tool == "tool_stretch_dex_wrist":
    pregrasp_lift_m += 0.02
if (self.lift_position is None):
    return Trigger.Response(
        success=False,
        message='lift position unavailable'
    )

self.logger.info('Raise tool to pregrasp height.')
lift_to_pregrasp_m = max(self.lift_position + pregrasp_lift_m, 0.1)
lift_to_pregrasp_m = min(lift_to_pregrasp_m, max_lift_m)

pose = {'joint_lift': lift_to_pregrasp_m}
self.move_to_pose(pose)

if self.tool == "tool_stretch_dex_wrist":
    self.logger.info('Rotate pitch/roll for grasping.')
    pose = {'joint_wrist_pitch': -0.3, 'joint_wrist_roll': 0.0}
    self.move_to_pose(pose)

pregrasp_yaw = self.manipulation_view.get_pregrasp_yaw(grasp_target,    
    self.tf2_buffer
)

self.logger.info('pregrasp_yaw = {0:.2f} rad'.format(pregrasp_yaw))
self.logger.info('pregrasp_yaw = {0:.2f} deg'.format(pregrasp_yaw * (180.0/np.pi)))
self.logger.info('Rotate the gripper for grasping.')

pose = {'joint_wrist_yaw': pregrasp_yaw}
self.move_to_pose(pose)

self.logger.info('Open the gripper.')
pose = {'gripper_aperture': 0.125}
self.move_to_pose(pose)

pregrasp_mobile_base_m, pregrasp_wrist_extension_m = self.manipulation_view.get_pregrasp_planar_translation(grasp_target,
   self.tf2_buffer
)
self.logger.info('pregrasp_mobile_base_m = {0:.3f} m'.format(pregrasp_mobile_base_m))
self.logger.info('pregrasp_wrist_extension_m = {0:.3f} m'.format(pregrasp_wrist_extension_m))
self.logger.info('Drive to pregrasp location.')
self.drive(pregrasp_mobile_base_m)

if pregrasp_wrist_extension_m > 0.0:
    extension_m = max(self.wrist_position + pregrasp_wrist_extension_m, min_extension_m)
    extension_m = min(extension_m, max_extension_m)
    self.logger.info('Extend tool above surface.')
    pose = {'wrist_extension': extension_m}
    self.move_to_pose(pose)
else:
    self.logger.info('negative wrist extension for pregrasp, so not extending or retracting.')
```

A plan to grasp the object is generated using FUNMAP’s ManipulationView class. This class requires the pregrasp pose parameters to generate the final grasp plan.

```python
# 3. Grasp the object and lift it
grasp_mobile_base_m, grasp_lift_m, grasp_wrist_extension_m = self.manipulation_view.get_grasp_from_pregrasp(grasp_target, self.tf2_buffer)
self.logger.info('grasp_mobile_base_m = {0:3f} m, grasp_lift_m = {1:3f} m, grasp_wrist_extension_m = {2:3f} m'.format(grasp_mobile_base_m, grasp_lift_m, grasp_wrist_extension_m))
self.logger.info('Move the grasp pose from the pregrasp pose.')

lift_m = max(self.lift_position + grasp_lift_m, 0.1)
lift_m = min(lift_m, max_lift_m)
extension_m = max(self.wrist_position + grasp_wrist_extension_m, min_extension_m)
extension_m = min(extension_m, max_extension_m)
pose = {'translate_mobile_base': grasp_mobile_base_m,
        'joint_lift': lift_m,
        'wrist_extension': extension_m}

```

The node then proceeds to grasp the object by sending our goal joint positions.


