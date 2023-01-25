# Nav2 Stack Using Simple Commander Python API
In this tutorial, we will work with Stretch to explore the Simple Commander Python API to enable autonomous navigation programatically. We will also demonstrate a security patrol routine for Stretch developed using this API. If you just landed here, it might be a good idea to first review the previous tutorial which covered mapping and navigation using RViz as an interface.

## The Simple Commander Python API
To develop complex behaviors with Stretch where navigation is just one aspect of the autonomy stack, we need to be able to plan and execute navigation routines as part of a bigger program. Luckily, the Nav2 stack exposes a Python API that abstracts the ROS layer and the Behavior Tree framework (more on that later!) from the user through a pre-configured library called the [robot navigator](https://github.com/hello-robot/stretch_ros2/blob/galactic/stretch_navigation/stretch_navigation/robot_navigator.py). This library defines a class called BasicNavigator which wraps the planner, controller and recovery action servers and exposes methods such as `goToPose()`, `goToPoses()` and `followWaypoints()` to execute navigation behaviors.

Let's first see the demo in action and then explore the code to understand how this works!

!!! warning
    We will not be using the arm for this demo. We recommend stowing the arm to avoid inadvertently bumping it into walls while the robot is navigating. 
    
Execute:
```bash
stretch_robot_stow.py
```

## Setup
Let's set the patrol route up before you can execute this demo in your map. This requires reading the position of the robot at various locations in the map and entering the co-ordinates in the array called `security_route` in the [simple_commander_demo.py](https://github.com/hello-robot/stretch_ros2/blob/galactic/stretch_navigation/stretch_navigation/simple_commander_demo.py#L30) file. 

First, execute the following command while passing the correct map YAML. Then, press the 'Startup' button:
```bash
ros2 launch stretch_navigation navigation.launch.py map:=${HELLO_ROBOT_FLEET}/maps/<map_name>.yaml
```

Since we expect the first point in the patrol route to be at the origin of the map, the first co-ordinates should be (0.0, 0.0). Next, to define the route, the easiest way to define the waypoints in the `security_route` array is by setting the robot at random locations in the map using the '2D Pose Estimate' button in RViz as shown below. For each location, note the x, y co-ordinates in the position field of the base_footprint frame and add it to the `security_route` array in [simple_commander_demo.py](https://github.com/hello-robot/stretch_ros2/blob/galactic/stretch_navigation/stretch_navigation/simple_commander_demo.py#L30).

<p align="center">
  <img height=500 src="https://user-images.githubusercontent.com/97639181/206782270-e84b33c4-e155-468d-8a46-d926b88ba428.gif"/>
</p>

Finally, Press Ctrl+C to exit out of navigation and save the simple_commander_demo.py file. Now, build the workspace to make the updated file available for the next launch command. 

```bash
cd ~/ament_ws/
colcon build
```

## See It In Action
Go ahead and execute the following command to run the demo and visualize the result in RViz. Be sure to pass the correct path to the map YAML:
Terminal 1:
```bash
ros2 launch stretch_navigation demo_security.launch.py map:=${HELLO_ROBOT_FLEET}/maps/<map_name>.yaml
```

<p align="center">
  <img height=500 src="https://user-images.githubusercontent.com/97639181/214690474-25d2bbf5-74b2-4789-af7e-ef6b208b2275.gif"/>
</p>

## Code Breakdown
Now, let's jump into the code to see how things work under the hood. Follow along in the [code](https://github.com/hello-robot/stretch_ros2/blob/galactic/stretch_navigation/stretch_navigation/simple_commander_demo.py) to have a look at the entire script.

First, we import the `BasicNavigator` class from the robot_navigator library which comes standard with the Nav2 stack. This class wraps around the planner, controller and recovery action servers. 
```python
from stretch_navigation.robot_navigator import BasicNavigator, TaskResult
```

In the main method, we initialize the node and create an instance of the BasicNavigator class called navigator.
```python
def main():
    rclpy.init()

    navigator = BasicNavigator()
```

Then, we set up a path for Stretch to patrol consisting of the co-ordinates in the map reference frame. These co-ordinates are specific to the map generated for this tutorial and would not be suitable for your robot. To define co-ordinates that work with your robot, first command the robot to at least three random locations in the map you have generated of your environment, then read the base_link x and y co-ordinates for each of them from the RViz TF plugin. Plug them in the `security_route` list. Keep in mind that for this demo, the robot is starting from the [0.0, 0.0] which is the origin of the map. This might not be the case for you.
```python
    security_route = [
        [0.0, 0.0],
        [1.057, 1.3551],
        [1.5828, 5.0823],
        [-0.5390, 5.6623],
        [0.8975, 9.7033]]
```

<p align="center">
  <img height=500 src="https://user-images.githubusercontent.com/97639181/206782439-775074c5-2401-4019-ade8-bf6c920ecc61.gif"/>
</p>

Next, we set an initial pose for the robot which would help AMCL localize the robot by providing an initial estimate of the robot's location. For this, we pass a PoseStamped message in the map reference frame with the robot's pose to the `setInitialPose()` method. The Nav2 stack recommends this before starting the lifecycle nodes using the "Startup" button in RViz. The `waitUntilNav2Active()` method waits until precisely this event.
```python
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)
    
    navigator.waitUntilNav2Active()
```

Once the nodes are active, the navigator is ready to receive pose goals either through the `goToPose()`, `goToPoses()` or `followWaypoints()` methods. For this demo, we will be using the `followWaypoints()` method which takes a list of poses as an argument. Since we intend for the robot to patrol the route indefinitely or until the node is killed (or the robot runs out of battery!), we wrap the method in an infinite while loop with `rclpy.ok()`. Then, we generate pose goals with the `security_route` list and append them to a new list called `route_poses` which is passed to the `followWaypoints()` method.
```python
    while rclpy.ok():
        
        route_poses = []
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.orientation.w = 1.0
        for pt in security_route[1:]:
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            route_poses.append(deepcopy(pose))
        
        nav_start = navigator.get_clock().now()
        navigator.followWaypoints(route_poses)
```

Since we are utilizing an action server built into Nav2, it's possible to seek feedback on this long running task through the action interface. The `isTaskComplete()` method returns a boolean depending on whether the patrolling task is complete. For the follow waypoints action server, the feedback message tells us which waypoint is currently being executed through the `feedback.current_waypoint` attribute. It is possible to cancel a goal using the `cancelTask()` method if the robot gets stuck. For this demo, we have set the timeout at 600 seconds to allow sufficient time for the robot to succeed. However, if you wish to see it in action, you can reduce the timeout to 30 seconds.
```python
        i = 0
        while not navigator.isTaskComplete():
            i += 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                navigator.get_logger().info('Executing current waypoint: ' +
                    str(feedback.current_waypoint + 1) + '/' + str(len(route_poses)))
                now = navigator.get_clock().now()

                if now - nav_start > Duration(seconds=600.0):
                    navigator.cancelTask()
```

Once the robot reaches the end of the route, we reverse the `security_route` list to generate the goal pose list that would be used by the `followWaypoints()` method in the next iteration of this loop.
```python
        security_route.reverse()
```

Finally, after a leg of the patrol route is executed, we call the `getResult()` method to know whether the task succeeded, canceled or failed to log a message.
```python
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            navigator.get_logger().info('Route complete! Restarting...')
        elif result == TaskResult.CANCELED:
            navigator.get_logger().info('Security route was canceled, exiting.')
            rclpy.shutdown()
        elif result == TaskResult.FAILED:
            navigator.get_logger().info('Security route failed! Restarting from other side...')
```

That's it! Using the Simple Commander API is as simple as that. Be sure to follow more examples in the [nav2_simple_commander](https://github.com/ros-planning/navigation2/tree/main/nav2_simple_commander) package if you wish to work with other useful methods exposed by the library.
