ArUco markers are a type of fiducials that are used extensively in robotics for identification and pose estimation. In this tutorial we will learn how to identify ArUco markers with the ArUco detection node and enable Stretch to navigate and align itself with respect to the marker.

Stretch uses the OpenCV ArUco detection library and is configured to identify a specific set of ArUco markers belonging to the 6x6, 250 dictionary. To understand why this is important please refer to this handy guide provided by OpenCV.

Stretch comes preconfigured to identify ArUco markers and the ROS node that enables this is the detect_aruco_markers node in the stretch_core package. Thanks to this, identifying and estimating the pose of a marker is as easy as pointing the camera to the marker and running the detection node. It is also possible and quite convenient to visualize the detections with RViz.

To do this, simply point the camera towards a marker and execute the following commands:
Terminal 1:
```
ros2 run stretch_core detect_aruco_markers
```
Terminal 2:
```
ros2 rviz2 rviz2 -d `ros2 pkg prefix –share stretch_core`/rviz/stretch_simple_test.rviz
```

By monitoring the /aruco/marker_array and /aruco/axes topics we can visualize the markers. The detection node also publishes the tf pose of the detected markers. This can be visualized by using the TF plugin and selecting the detected marker to inspect the pose. Next, we will use exactly that to compute the transform between the detected marker and the base_link of the robot.

If you have not already done so, now might be a good time to review the tf_tranformation tutorial. Go on, we can wait…
Now that we know how to program stretch to return the transform between known reference frames, we can use this knowledge to compute the transform between the detected marker and the robot base_link.

Since we want Stretch to align with respect to the marker we define a 0.5m offset in the marker y-axis where Stretch would come to a stop. At the same time, we also want Stretch to point the arm towards the marker so as to make the subsequent manipulation tasks easier to accomplish. This would result in the end pose of the base_link as shown below. Sweet! The next task is to plan a trajectory for the mobile base to reach this end pose. We do this in three steps:
Turn theta degrees towards the goal position
Travel straight to the goal position
Turn phi degrees to attain the goal orientation

Luckily, we know how to command Stretch to execute a trajectory using the joint trajectory server. If not, have a look at this tutorial to know how.