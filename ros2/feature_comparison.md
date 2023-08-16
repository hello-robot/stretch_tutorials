# ROS 1 v/s ROS 2 Feature Comparison

This document will help you better understand our progress of bringing parity between our ROS 1 and ROS 2 software offerings. Additional comments indicate changes in the API and/or behavior for a component.

| Package | Node/Launch/Component | Item | ROS 1 | ROS 2 | Comments |
|-|-|-|-|-|-|
| Stretch Install | Humble | Installation scripts |  |  |  |
| Stretch Install | Humble | Mesh and URDF files |  |  |  |
| Stretch Install | Iron | Installation scripts |  |  |  |
| Stretch Install | Iron | Mesh and URDF files |  |  |  |
| Stretch Core | stretch_driver | JointTrajectoryAction Server, Position mode |  |  |
| Stretch Core | stretch_driver | JointTrajectoryAction Server, Navigation mode |  |  |
| Stretch Core | stretch_driver | JointTrajectoryAction Server, Trajectory mode |  |  |
| Stretch Core | stretch_driver | Mode topic |  |  |
| Stretch Core | stretch_driver | Mode services |  |  |
| Stretch Core | stretch_driver | Home/stop/stow services |  |  |
| Stretch Core | stretch_driver | Magnetometer and BatterState topic |  |  |
| Stretch Core | stretch_driver | Joint states, joint limits |  |  |
| Stretch Core | stretch_driver | TF2 |  |  |
| Stretch Core | keyboard_teleop | Teleoperation |  |  |  |
| Stretch Core | keyboard_teleop | Service triggers for demos |  |  |  |
| Stretch Core | detect_aruco_markers | Aruco marker detection |  |  |  |
| Stretch Core | rplidar | Driver |  |  |  |
| Stretch Core | rplidar | LaserScan filtering |  |  |  |
| Stretch Core | d435i | Driver |  |  |  |
| Stretch Core | d435i | High/Low resolution launch |  |  |  |
| Stretch Core | d435i | IMU |  |  |  |
| Stretch Core | d435i | Aligned depth |  |  |  |
| Stretch Core | respeaker | Driver |  |  |  |
| Hello Helpers | hello_misc | HelloNode |  |  |  |
| Hello Helpers | hello_misc | Preprocess trajectories |  |  |  |
| Hello Helpers | hello_ros_viz | Create markers |  |  |  |
| Hello Helpers | simple_command_groups | SimpleCommandGroup |  |  |  |
| Hello Helpers | gripper_conversion | GripperConversion |  |  |  |
| Hello Helpers | fit_plane | Fit plane helper functions |  |  |  |
| Hello Helpers | configure_wrist | Switch between standard and dex wrist |  |  |  |
| Hello Helpers | hello_misc | HelloNode API |  |  |  |
| Hello Helpers | API Docs | Documentation |  |  |  |
| Stretch Description | stretch_description | Stretch URDF |  |  |  |
| Stretch Description | stretch_description | Stretch URDF |  |  |  |
| Stretch Description | stretch_description | Standard Gripper xacro |  |  |  |
| Stretch Description | stretch_description | Dex wrist xacro |  |  |  |
| Stretch Description | stretch_description | Batch-specific mesh files |  |  |  |
| Stretch Calibration | collect_head_calibration_data | CollectHeadCalibrationDataNode |  |  |  |
| Stretch Calibration | process_head_calibration_data | HeadCalibrator, ProcessHeadCalibrationNode |  |  |  |
| Stretch Calibration | check_head_calibration | check_head_calibration |  |  |  |
| Stretch Calibration | revert_to_previous_calibration | revert_to_previous_calibration |  |  |  |
| Stretch Calibration | update_uncalibrated_urdf | update_uncalibrated_urdf |  |  |  |
| Stretch Calibration | update_urdf_after_xacro_change | update_urdf_after_xacro_change |  |  |  |
| Stretch Calibration | update_with_most_recent_calibration | update_with_most_recent_calibration |  |  |  |
| Stretch Calibration | visualize_most_recent_head_calibration | visualize_most_recent_head_calibration |  |  |  |
| Stretch Calibration | Revert to position mode | Revert to position mode |  |  |  |
| Stretch Tutorials | Getting started | Instructions |  |  |  |
| Stretch Tutorials | Modes tutorial | Instructions |  |  |  |
| Stretch Tutorials | ROS 2 with rclpy | Instructions |  |  |  |
| Stretch Tutorials | Race conditions and deadlocks in ROS 2 | Instructions |  |  |  |
| Stretch Tutorials | HelloNode Tutorial | Instructions |  |  |  |
| Stretch Tutorials | HFollow Joint Trajectory Commands | Trajectory mode |  |  |  |
| Stretch Tutorials | HFollow Joint Trajectory Commands | Joint Trajectory Server |  |  |  |
| Stretch Tutorials | Internal State of Stretch | Command line and RQT graph |  |  |  |
| Stretch Tutorials | RViz basics | Robot visualizationa and TF tree |  |  |  |
| Stretch Tutorials | Teleoperation | Velocity command of mobile base |  |  |  |
| Stretch Tutorials | Tf2 broadcaster and listener | TF2 static broadcaster and listener |  |  |  |
| Stretch Tutorials | Respeaker voice to text | Instructions |  |  |  |
| Stretch Tutorials | API docs | Instructions |  |  |  |
| Stretch Tutorials | Migration guides | From ROS1 Noetic & from ROS2 Galactic |  |  |  |
| Stretch FUNMAP | manipulation_planning | plan_surface_coverage, detect_cliff |  |  |  |
| Stretch FUNMAP | manipulation_planning | PlanarRobotModel, ManipulationPlanner |  |  |  |
| Stretch FUNMAP | manipulation_planning | ManipulationView |  |  |  |
| Stretch FUNMAP | mapping | robot stowing, scanning, & localizing methods |  |  |  |
| Stretch FUNMAP | mapping | HeadScan |  |  |  |
| Stretch FUNMAP | navigate | ForwardMotionObstacleDetector |  |  |  |
| Stretch FUNMAP | navigate | FastSingleViewPlanner |  |  |  |
| Stretch FUNMAP | navigate | MoveBase |  |  |  |
| Stretch FUNMAP | ros_max_height_image | ROSVolumeOfInterest |  |  |  |
| Stretch FUNMAP | ros_max_height_image | ROSMaxHeightImage |  |  |  |
| Stretch FUNMAP | funmap | ContactDetector, FunMapNode, services |  |  |  |
| Stretch FUNMAP | API Docs | Instructions |  |  |  |
| Stretch Deep Perception | detection_node | DetectionNode |  |  |  |
| Stretch Deep Perception | detect_faces | OpenVINO Face Detection |  |  |  |
| Stretch Deep Perception | detect_objects | PyTorch YOLO object detection |  |  |  |
| Stretch Deep Perception | detect_nearest_mouth | OpenVINO Mouth Detection |  |  |  |
| Stretch Deep Perception | detect_nearest_mouth | OpenVINO Body Landmark Detection |  |  |  |
| Stretch Deep Perception | API Docs | Instructions |  |  |  |
| Stretch Demos | hello_world | HelloWorldNode, services |  |  |  |
| Stretch Demos | clean_surface | CleanSurfaceNode, services |  |  |  |
| Stretch Demos | grasp_object | GraspObjectNode, services |  |  |  |
| Stretch Demos | handover_object | HandoverObjectNode, services |  |  |  |
| Stretch Demos | open_drawer | OpenDrawerNode, services |  |  |  |
| Stretch Demos | HelloNode API | Switch demos to HelloNode API |  |  |  |
| Stretch Demos | autodocking_behaviors | MoveBaseActionClient, CheckTF |  |  |  |
| Stretch Demos | autodocking_behaviors | VisualServoing |  |  |  |
| Stretch Demos | API docs | Instructions |  |  |  |
| Stretch Nav2 | offline_mapping | SLAM |  |  |  |
| Stretch Nav2 | navigation | Pose, Waypoints, Obstacle avoidance |  |  |  |
| Stretch Nav2 | Patrolling Demo | Python API, Autonomous waypoint nav |  |  |  |
| Stretch Nav2 | API Docs | Instructions |  |  |  |
| Stretch Tutorials | Nav2 stack | Mapping |  |  |  |
| Stretch Tutorials | Nav2 stack | Navigation |  |  |  |'
| Stretch Tutorials | Filter laser scans | RPLidar and laserscan filtering |  |  |  |
| Stretch Tutorials | Deep perception | Objects and faces detection |  |  |  |
| Stretch Tutorials | FUNMAP demos | Instructions |  |  |  |
| Stretch Tutorials | Align to ArUco | ArUco detection, tf transforms and trajectory server |  |  |  |
| Stretch Tutorials | Obstacle avoider | RPLidar based sensing and avoidance |  |  |  |
| Stretch Tutorials | Mobile base collision avoidance | RPLidar based sensing and avoidance |  |  |  |
| Stretch MoveIt 2 | MoveIt Config Files | YAML/XML/SRDF/URDF files |  |  |  |
| Stretch MoveIt 2 | moveit_py | Joint Space Goals |  |  |  |
| Stretch MoveIt 2 | moveit_py | Pose Goals |  |  |  |
| Stretch MoveIt 2 | moveit_py | Multiplanning pipeline |  |  |  |
| Stretch MoveIt 2 | moveit_py | Octomap plugin |  |  |  |
| Stretch MoveIt 2 | moveit | RViz plugin |  |  |  |
| Stretch MoveIt 2 | moveit | End effector pose goals |  |  |  |
| Stretch MoveIt 2 | moveit | Hybrid planning |  |  |  |
| Web-based Teleoperation | web_interface | operator |  |  |  |
| Web-based Teleoperation | web_interface | robot |  |  |  |
