# Object Handover Demo

FUNMAP is a hardware-targeted perception, planning, and navigation framework developed by Hello Robot for ROS developers and researchers. Some of the key features provided by FUNMAP include cliff detection, closed-loop navigation, and mapping. In this tutorial, we will explore an object-handover demo using FUNMAP.

## Motivation
Through this demo, we demonstrate human mouth detection using the stretch_deep_perception package, and demonstrate object delivery with navigation using FUNMAP. The robot is teleoperated to have a person in the view of its camera. The person requesting the object must face the robot. We use OpenVINO to perform facial recognition.

## Workspace Setup
Ideally, this demo requires the person requesting the object to be facing the robot’s camera. Use keyboard teleop to place the object in the robot’s gripper.

## How-to-run
After building and sourcing the workspace, home the robot:

```bash
stretch_robot_home.py
```

This ensures that the underlying stretch_body package knows the exact joint limits and provides the user with a good starting joint configuration.

After homing, launch the object handover demo:

```bash
ros2 launch stretch_demos handover_object.launch.py
```

This command will launch stretch_driver, stretch_funmap, and the handover_object nodes. 

In a new terminal, launch keyboard teleoperation:

```bash
ros2 run stretch_core keyboard_teleop --ros-args -p handover_object_on:=true
```

You will be presented with a keyboard teleoperation menu in a new terminal window. Use key commands to get the Stretch configured as per the above workspace setup guidelines. Once the robot is ready, press ‘y’ or ‘Y’ to trigger object handover.

## Code Explained
The object_handover node uses the joint_trajectory_server inside stretch_core to send out target joint positions.

```python
self.trajectory_client = ActionClient(self, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory', callback_group=self.callback_group)
server_reached = self.trajectory_client.wait_for_server(timeout_sec=60.0)
if not server_reached:
    self.get_logger().error('Unable to connect to joint_trajectory_server. Timeout exceeded.')
    sys.exit()
```

Additionally, the node also subscribes to mouth positions detected by stretch_deep_perception, and Stretch’s joint state topics.

```python
self.joint_states_subscriber = self.create_subscription(JointState, '/stretch/joint_states', qos_profile=1, callback=self.joint_states_callback, callback_group=self.callback_group)
   	 
self.mouth_position_subscriber = self.create_subscription(MarkerArray, '/nearest_mouth/marker_array', qos_profile=1, callback=self.mouth_position_callback, callback_group=self.callback_group)
```

Whenever the node receives a mouth position message, it computes a handoff XYZ coordinate depending upon the current wrist and mouth positions:

```python
def mouth_position_callback(self, marker_array):
    with self.move_lock:

        for marker in marker_array.markers:
            if marker.type == self.mouth_marker_type:
                mouth_position = marker.pose.position
                self.mouth_point = PointStamped()
                self.mouth_point.point = mouth_position
                header = self.mouth_point.header
                header.stamp = marker.header.stamp
                header.frame_id = marker.header.frame_id
                # header.seq = marker.header.seq
                self.logger.info('******* new mouth point received *******')

                lookup_time = Time(seconds=0) # return most recent transform
                timeout_ros = Duration(seconds=0.1)

                old_frame_id = self.mouth_point.header.frame_id
                new_frame_id = 'base_link'
                stamped_transform = self.tf2_buffer.lookup_transform(new_frame_id, old_frame_id, lookup_time, timeout_ros)
                points_in_old_frame_to_new_frame_mat = rn.numpify(stamped_transform.transform)
                camera_to_base_mat = points_in_old_frame_to_new_frame_mat

                grasp_center_frame_id = 'link_grasp_center'
                stamped_transform = self.tf2_buffer.lookup_transform(new_frame_id,
                    grasp_center_frame_id,
                    lookup_time,
                    timeout_ros
                )
                grasp_center_to_base_mat = rn.numpify(stamped_transform.transform)

                mouth_camera_xyz = np.array([0.0, 0.0, 0.0, 1.0])
                mouth_camera_xyz[:3] = rn.numpify(self.mouth_point.point)[:3]

                mouth_xyz = np.matmul(camera_to_base_mat, mouth_camera_xyz)[:3]
                fingers_xyz = grasp_center_to_base_mat[:,3][:3]

                handoff_object = True

                if handoff_object:
                    # attempt to handoff the object at a location below
                    # the mouth with respect to the world frame (i.e.,
                    # gravity)
                    target_offset_xyz = np.array([0.0, 0.0, -0.2])
                else:
                    object_height_m = 0.1
                    target_offset_xyz = np.array([0.0, 0.0, -object_height_m])
                target_xyz = mouth_xyz + target_offset_xyz

                fingers_error = target_xyz - fingers_xyz
                self.logger.info(f'fingers_error = {str(fingers_error)}')

                delta_forward_m = fingers_error[0]
                delta_extension_m = -fingers_error[1]
                delta_lift_m = fingers_error[2]

                max_lift_m = 1.0
                lift_goal_m = self.lift_position + delta_lift_m
                lift_goal_m = min(max_lift_m, lift_goal_m)
                self.lift_goal_m = lift_goal_m

                self.mobile_base_forward_m = delta_forward_m

                max_wrist_extension_m = 0.5
                wrist_goal_m = self.wrist_position + delta_extension_m

                if handoff_object:
                    # attempt to handoff the object by keeping distance
                    # between the object and the mouth distance
                    wrist_goal_m = wrist_goal_m - 0.25 # 25cm from the mouth
                    wrist_goal_m = max(0.0, wrist_goal_m)

                self.wrist_goal_m = min(max_wrist_extension_m, wrist_goal_m)

                self.handover_goal_ready = True
```

The delta between the wrist XYZ and mouth XYZ is used to calculate the lift position, base forward translation, and wrist extension.

Once the user triggers the handover object service, the node sends out joint goal positions for the base, lift, and the wrist, to deliver the object near the person’s mouth:

```python
self.logger.info("Starting object handover!")
with self.move_lock:
    # First, retract the wrist in preparation for handing out an object.
    pose = {'wrist_extension': 0.005}
    self.move_to_pose(pose)

    if self.handover_goal_ready:
        pose = {'joint_lift': self.lift_goal_m}
        self.move_to_pose(pose)
        tolerance_distance_m = 0.01
        at_goal = self.move_base.forward(self.mobile_base_forward_m,
            detect_obstacles=False,
            tolerance_distance_m=tolerance_distance_m
        )
        pose = {'wrist_extension': self.wrist_goal_m}
        self.move_to_pose(pose)
        self.handover_goal_ready = False
```

## Results and Expectations

This demo serves as an experimental setup to explore object delivery with Stretch. Please be advised that this code is not expected to work perfectly. Some of the shortcomings of the demo include:

- The node requires the target user's face to be in the camera view while triggering the demo. As it stands, it does not keep any past face detections in its memory.

- Facial landmarks detection might not work well for some faces and is highly variable to the deviation from the faces that the algorithm was originally trained on.

Users are encouraged to try this demo and submit improvements.
