# Hello World Demo

FUNMAP is a hardware-targeted perception, planning, and navigation framework developed by Hello Robot for ROS developers and researchers. Some of the key features provided by FUNMAP include cliff detection, closed-loop navigation, and mapping. In this tutorial, we will explore a hello world writing demo using FUNMAP.

## Motivation

Stretch is a contacts-sensitive robot, making it compliant in human spaces. This contact sensitivity can be leveraged for surface and obstacles detection. Through this demo we demonstrate one specific application of contacts detection - surface writing. In this demo, Stretch writes out the letters 'H', 'E', 'L', 'L', and 'O' in a sequential manner.

## Workspace Setup

Things that you will need
 - Whiteboard
 - Marker

Ideally, this demo requires a whiteboard that is placed at the same height as the Stretch. The writing surface must be flat and reachable by the robot's arm.

## How-to-run

After building and sourcing the workspace, home the robot:

```bash
stretch_robot_home.py
```

This ensures that the underlying stretch_body package knows the exact joint limits and provides the user with a good starting joint configuration.

After homing, launch the hello world demo:

```bash
ros2 launch stretch_demos hello_world.launch.py
```


This command will launch stretch_driver, stretch_funmap, and the hello_world nodes. 

Next, in a separate terminal, run:

```bash
ros2 run stretch_core keyboard_teleop --ros-args -p hello_world_on:=true
```

You will be presented with a keyboard teleoperation menu. Teleoperate the robot so that the arm is perpendicular to the writing surface, about 30 cm away. Place a marker pointing outward in the robot's gripper. Once the robot is ready, press ‘`’ or '~’ to trigger hello world writing.

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

Additionally, the node also subscribes to Stretch’s joint states topic.

```python
self.joint_states_subscriber = self.create_subscription(JointState,
    '/stretch/joint_states',
    callback=self.joint_states_callback,
    qos_profile=10,
    callback_group=self.callback_group
)
```

Whenever the user triggers the hello world write service, the robot moves to an initial joint configuration:

```python
def move_to_initial_configuration(self):
    initial_pose = {'wrist_extension': 0.01,
                    'joint_lift': self.letter_top_lift_m,
                    'joint_wrist_yaw': 0.0}

    self.logger.info('Move to initial arm pose for writing.')
    self.move_to_pose(initial_pose)
```

Thereafter, the node uses FUNMAP's `align_to_nearest_cliff` service to align the Stretch w.r.t the whiteboard.

```python
def align_to_surface(self):
    self.logger.info('align_to_surface')
    trigger_request = Trigger.Request() 
    trigger_result = self.trigger_align_with_nearest_cliff_service.call_async(trigger_request)
    self.logger.info('trigger_result = {0}'.format(trigger_result))
```

After aligning with the whiteboard, the node then proceeds to execute joint position goals to draw out each letter of the word "hello":

```python
self.letter_h()
self.space()
self.letter_e()
self.space()
self.letter_l()
self.space()
self.letter_l()
self.space()
self.letter_o()
self.space()

return Trigger.Response(
    success=True,
    message='Completed writing hello!'
)
```

## Results and Expectations

This demo serves as an experimental setup to explore whiteboard writing with Stretch. Please be advised that this code is not expected to work perfectly. Some of the shortcomings of the demo include:

- Each Stretch has unique calibration thresholds. The contacts thresholds defined in the stock demo code might not work for your Stretch. Additional tuning might be necessary.

- The quality of the written text structure is also highly dependent on mobile base movements. Currently, navigation is open-loop and does not account for accumulated error terms.

Users are encouraged to try this demo and submit improvements.
