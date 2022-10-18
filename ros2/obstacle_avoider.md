# Obstacle Avoider
In this tutorial we will work with Stretch to detect and avoid obstacles using the onboard RPlidar A1 laser scanner and learn how to filter laser scan data. If you want to know more about the laser scanner setup on Stretch and how to get it up and running, we recommend visiting the previous tutorials on filtering laser scans and mobile base collision avoidance.

A major drawback of using any ToF (Time of Flight) sensor is the inherent inaccuracies as a result of occlusions and weird reflection and diffraction phenomena the light pulses are subject to in an unstructured environment. This results in unexpected and undesired noise that can get in the way of an otherwise extremely useful sensor. Fortunately, it is easy to account for and eliminate these inaccuracies to a great extent by filering out the noise. We will do this with a ROS package called laser_filters that comes prebuilt with some pretty handy laser scan message filters.

By the end of this tutorial, you will be able to tweak them for your particular use case and publish and visualize them on the /scan_filtered topic using RViz. So let’s jump in! We will look at three filters from this package that have been tuned to work well with Stretch in an array of scenarios.

## LaserScan Filtering
LaserScanAngularBoundsFilterInPlace - This filter removes laser scans belonging to an angular range. For Stretch, we use this filter to discount points that are occluded by the mast because it is a part of Stretch’s body and not really an object we need to account for as an obstacle while navigating the mobile base.

LaserScanSpeckleFilter - We use this filter to remove phantom detections in the middle of empty space that are a result of reflections around corners. These disjoint speckles can be detected as false positives and result in jerky motion of the base through empty space. Removing them returns a relatively noise-free scan.

LaserScanBoxFilter - Stretch is prone to returning false detections right over the mobile base. While navigating, since it’s safe to assume that Stretch is not standing right above an obstacle, we filter out any detections that are in a box shape over the mobile base.

However, beware that filtering laser scans comes at the cost of a sparser scan that might not be ideal for all applications. If you want to tweak the values for your end application, you could do so by changing the values in the laser_filter_params.yaml file and by following the laser_filters package wiki. Also, if you are feeling zany and want to use the raw unfiltered scans from the laser scanner, simply subscribe to the /scan topic instead of the /scan_filtered topic.

![laser_filtering](https://user-images.githubusercontent.com/97639181/196327251-c39f3cbb-c898-48c8-ae28-2683564061d9.gif)

## Avoidance logic
Now, let’s use what we have learned so far to upgrade the collision avoidance demo in a way that Stretch is able to scan an entire room autonomously without bumping into things or people. To account for dynamic obstacles getting too close to the robot, we will define a keepout distance of 0.4 m - detections below this value stop the robot. To keep Stretch from getting too close to static obstacles, we will define another variable called turning distance of 0.75 m - frontal detections below this value make Stretch turn to the left until it sees a clear path ahead.

Building up on the teleoperation using velocity commands tutorial, let's implement a simple logic for obstacle avoidance. The logic can be broken down into three steps:
1. If the minimum value from the frontal scans is greater than 0.75 m then continue to move forward
2. If the minimum value from the frontal scans is less than 0.75 m then turn to the right until this is no longer true
3. If the minimum value from the overall scans is less than 0.4 m then stop the robot

## Warnings
If you see Stretch try to run over your lazy cat or headbutt a wall, just press the bright runstop button on Stretch's head to calm it down. For pure navigation tasks, it's also safer to stow Stretch's arm in. Execute the command:
```bash
stretch_robot_stow.py
```

## See It In Action
Alright, let's see it in action! Execute the following command to run the scripts:
```bash
ros2 launch stretch_core rplidar_keepout.launch.py
```

![avoidance](https://user-images.githubusercontent.com/97639181/196327294-1b2dde5e-2fdc-4a67-a188-ae6b1f5e6a06.gif)

## Code Breakdown:
Let's jump into the code to see how things work under the hood. Follow along here to have a look at the entire script.

The turning distance is defined by the distance attribute and the keepout distance is defined by the keepout attribute.
```python
        self.distance = 0.75 # robot turns at this distance
        self.keepout = 0.4 # robot stops at this distance
```

To pass velocity commands to the mobile base we publish the translational and rotational velocities to the /stretch/cmd_vel topic. To subscribe to the filtered laser scans from the laser scanner, we subscribe to the /scan_filtered topic. While you are at it, go ahead and check the behavior by switching to the /scan topic instead. You're welcome!
```python
        self.publisher_ = self.create_publisher(Twist, '/stretch/cmd_vel', 1) #/stretch_diff_drive_controller/cmd_vel for gazebo
        self.subscriber_ = self.create_subscription(LaserScan, '/scan_filtered', self.lidar_callback, 10)
```

This is the callback function for the laser scanner that gets called every time a new message is received.
```python
def lidar_callback(self, msg):
```

When the scan message is filtered, all the ranges that are filtered out are assigned the nan (not a number) value. This can get in the way of computing the minimum. Therefore, we reassign these values to inf (infinity). 
```python
        all_points = [r if (not isnan(r)) else inf for r in msg.ranges]
```

Next, we compute the two minimums that are neccessary for the avoidance logic to work - the overall minimum and the frontal minimum named min_all and min_front respectively.
```python
        front_points = [r * sin(theta) if (theta < -2.5 or theta > 2.5) else inf for r,theta in zip(msg.ranges, angles)]
        front_ranges = [r if abs(y) < self.extent else inf for r,y in zip(msg.ranges, front_points)]

        min_front = min(front_ranges)
        min_all = min(all_points)
```

Finally, we check the minimum values against the distance and keepout attributes to set the rotational and linear velocities of the mobile base with the set_speed() method.
```python
        if(min_all < self.keepout):
            lin_vel = 0.0
            rot_vel = 0.0
        elif(min_front < self.distance):
            lin_vel = 0.0
            rot_vel = 0.25
        else:
            lin_vel = 0.5
            rot_vel = 0.0

        self.set_speed(lin_vel, rot_vel)
```

That wasn't too hard, was it? Feel free to play with this code and change the attributes to see how it affects Stretch's behavior.
