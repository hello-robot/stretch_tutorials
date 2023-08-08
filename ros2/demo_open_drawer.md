# Drawer Opening Demo

FUNMAP is a hardware-targeted perception, planning, and navigation framework developed by Hello Robot for ROS developers and researchers. Some of the key features provided by FUNMAP include cliff detection, closed-loop navigation, and mapping. In this tutorial, we will explore an drawer-opening demo using FUNMAP.

## Motivation

Through this demo, we demonstrate opening of common drawers that have handles, and contact sensing with navigation using FUNMAP. FUNMAP provides APIs that allow users to extend the arm or adjust the lift until a contact is detected. We leverage this contact detection API in the demo. Users can trigger the drawer opening demo through an upward or downward motion of the hook attached to the end-effector, explained below.

## Workspace Setup

The robot is teleoperated so as to keep its arm perpendicular to the drawer’s frontal surface, slightly above or below the handle. The workspace must be clear of any obstacles between the end-effector and the drawer. Finally, ensure that the wrist can reach the drawer through extension.

## How-to-run

After building and sourcing the workspace, home the robot:

```bash
stretch_robot_home.py
```

This ensures that the underlying stretch_body package knows the exact joint limits and provides the user with a good starting joint configuration.

After homing, launch the drawer opening demo:

```bash
ros2 launch stretch_demos open_drawer.launch.py
```

This command will launch stretch_driver, stretch_funmap, and the open_drawer nodes. 

In a new terminal, launch keyboard teleoperation:

```bash
ros2 run stretch_core keyboard_teleop --ros-args -p open_drawer_on:=true
```

You will be presented with a keyboard teleoperation menu in a new terminal window. Use key commands to get the Stretch configured as per the above workspace setup guidelines. Once the robot is ready, press ‘.’ or ‘>’ to trigger the drawer opening with an upward motion. If you want to run the demo with a downward motion, press ‘z’ or ‘Z’.

## Code Explained

The open_drawer node uses the joint_trajectory_server inside stretch_core to send out target joint positions. 

```python
self.trajectory_client = ActionClient(self, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory', callback_group=self.callback_group)
    	server_reached = self.trajectory_client.wait_for_server(timeout_sec=60.0)
    	if not server_reached:
        	self.get_logger().error('Unable to connect to joint_trajectory_server. Timeout exceeded.')
        	sys.exit()
```

The node then proceeds to connect to the contact detection services provided by FUNMAP:

```python
self.trigger_open_drawer_service = self.create_service(Trigger, '/open_drawer/trigger_open_drawer_down',                                                self.trigger_open_drawer_down_callback, callback_group=self.callback_group)

self.trigger_open_drawer_service = self.create_service(Trigger, '/open_drawer/trigger_open_drawer_up',                                                self.trigger_open_drawer_up_callback, callback_group=self.callback_group)

self.trigger_align_with_nearest_cliff_service = self.create_client(Trigger, '/funmap/trigger_align_with_nearest_cliff', callback_group=self.callback_group)

self.trigger_align_with_nearest_cliff_service.wait_for_service()
self.logger.info('Node ' + self.get_name() + ' connected to /funmap/trigger_align_with_nearest_cliff.')

self.trigger_reach_until_contact_service = self.create_client(Trigger, '/funmap/trigger_reach_until_contact', callback_group=self.callback_group)
self.trigger_reach_until_contact_service.wait_for_service()
self.logger.info('Node ' + self.get_name() + ' connected to /funmap/trigger_reach_until_contact.')

self.trigger_lower_until_contact_service = self.create_client(Trigger, '/funmap/trigger_lower_until_contact', callback_group=self.callback_group)
self.trigger_lower_until_contact_service.wait_for_service()
self.logger.info('Node ' + self.get_name() + ' connected to /funmap/trigger_lower_until_contact.')
```

Once the user triggers the drawer opening demo, the robot is commanded to move to an initial configuration:

```python
initial_pose = {'wrist_extension': 0.01,
                'joint_wrist_yaw': 1.570796327,
                'gripper_aperture': 0.0}
self.logger.info('Move to the initial configuration for drawer opening.')
self.move_to_pose(initial_pose)
```

The robot then proceeds to extend its wrist until it detects a contact, hopefully caused by a drawer surface.

```python
self.extend_hook_until_contact()
```

It then backs off from the surface by about 5 cm so as to not touch it. 

```python
success = self.backoff_from_surface()
if not success:
    return Trigger.Response(
        success=False,
        message='Failed to backoff from the surface.'
    )
```

Depending upon the user’s choice, the robot will either raise or lower its lift so as to hook the handle. 

```python
if direction == 'down':
    self.lower_hook_until_contact()
elif direction == 'up':
    self.raise_hook_until_contact()
```

Finally, the robot will pull open the drawer.

```python
success = self.pull_open()
if not success:
     return Trigger.Response(
         success=False,
         message='Failed to pull open the drawer.'
     )

push_drawer_closed = False
if push_drawer_closed:
    time.sleep(3.0)
    self.push_closed()

return Trigger.Response(
    success=True,
    message='Completed opening the drawer!'
)
```
