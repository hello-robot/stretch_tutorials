In this tutorial we will work with Stretch to detect and avoid obstacles using the onboard RPlidar A1 laser scanner and talk a bit about filtering laser scan data. If you want to know more about the laser scanner setup on Stretch and how to get it up and running, we recommend visiting the previous tutorials on filtering laser scans and mobile base collision avoidance.

A major drawback of using any ToF (Time of Flight) sensor is the inherent inaccuracies as a result of occlusions and weird reflection and diffraction phenomena the light pulses are subject to in an unstructured environment. This results in unexpected and undesired noise that can get in the way of an otherwise extremely useful sensor. Fortunately, it is easy to account for and eliminate these inaccuracies with a ROS package called laser_filters that comes prebuilt with some pretty handy scan message filters.

We will look at three filters from this package that have been tuned to work well with Stretch in an array of scenarios. By the end of this tutorial, you will be able to tweak them for your particular use case and publish and visualize them on the /scan_filtered topic using RViz. Let’s jump in!

LaserScanAngularBoundsFilterInPlace - This filter removes laser scans belonging to an angular range. For Stretch we use this filter to discount points that are occluded by the mast because the mast being a part of Stretch’s body is not an object we need to account for as an obstacle while navigating the mobile base.

LaserScanSpeckleFilter - We use this filter to remove phantom detections in the middle of empty space that are a result of reflections around corners. These disjoint speckles can be detected as false positives and result in jerky motion of the base through empty space. Removing them returns a noise-free scan.

LaserScanBoxFilter - Stretch is prone to returning false detections right over the mobile base. While navigating, since it’s safe to assume that Stretch is not standing right above an obstacle, we filter out any detections that are in a box shape over the mobile base.

If you want to tweak the values for your end application, you could do so by changing the values in the laser_filter_params.yaml file. Also, if you want to use the unfiltered scans from the laser scanner, simply subscribe to the /scan topic instead of the /scan_filtered topic.

Now, let’s use what we have learned so far to upgrade the collision avoidance demo in a way that Stretch is able to scan an entire room autonomously without bumping into things or people. To account for people getting too close to the robot, we will define a keepout distance of 0.4 m and to keep Stretch from getting too close to static obstacles we will define another variable called turning distance. We set this as 0.75 m - it enables Stretch to start turning if a static obstacle is less than 0.75 m away.

Building up on the teleoperation tutorial that enables Stretch’s mobile base to be controlled using velocity commands, we implement a simple logic for obstacle avoidance. The logic can be broken down into three steps:
If the minimum value from the frontal scans is greater than 0.75 m then continue to move forward
If the minimum value from the frontal scans is less than 0.75 m then turn to the right until this is no longer true
If the minimum value from the overall scans is less than 0.4 m then stop the robot

This simple algorithm is sufficient to account for both static and dynamic obstacles
