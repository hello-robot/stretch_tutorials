# Stretch Simulation Tutorial

This tutorial will guide you through setting up and using the Stretch robot simulation in ROS2.

## What You'll Learn

By the end of this tutorial, you'll be able to:

- Install and configure the Stretch simulation environment

- Launch the simulated robot in various environments

- Control the robot using keyboard teleop and web interface

- Use the simulation for navigation and mapping

- Integrate the simulation with your own ROS2 applications

## Prerequisites

Before starting this tutorial, you should have:

- Basic familiarity with Linux command line

- Understanding of ROS2 concepts (nodes, topics, services). If you're new to ROS2 concepts, we recommend first reading through the [Introduction to ROS2](intro_to_ros2.md) tutorial.

- A computer meeting the system requirements below

## System Requirements

**Operating System**: Ubuntu 22.04 (recommended) or WSL2 with GPU acceleration

**Hardware Requirements**:
- **Minimum**: 16GB RAM, Nvidia graphics card

- **Recommended**: 32GB RAM, dedicated GPU

**Why these requirements?** The simulation uses Mujoco physics engine with 3D rendering, which requires significant computational resources and GPU acceleration for smooth performance.

## Installation Guide

### Step 1: Install ROS2 Humble (10 minutes)

> **Note**: Skip this step if you're running on a Stretch robot, as ROS2 is already installed.

```bash
# Add universe repository for additional packages
sudo apt install software-properties-common
sudo add-apt-repository universe

# Install curl for downloading ROS2 keys
sudo apt update && sudo apt install curl -y

# Add ROS2 official repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package list and install ROS2
sudo apt update
sudo apt install ros-humble-desktop ros-dev-tools rviz python3-pip

# Source ROS2 environment (you'll need to do this in every new terminal)
source /opt/ros/humble/setup.bash
```

**Expected output**: After installation, you should be able to run `ros2 --help` and see the ROS2 command options.

### Step 2: Install Node.js and npm (5 minutes)

Node.js is required for the web teleop interface that allows you to control the robot through a web browser.

```bash
# Install Node.js 22.x (latest LTS version)
curl -sL https://deb.nodesource.com/setup_22.x | sudo -E bash -
sudo apt install -y nodejs

# Verify installation
node --version  # Should show v22.x.x
npm --version   # Should show npm version
```

### Step 3: Set up the ROS2 Workspace (1 hour)

> **Note**: Skip this step if you're running on a Stretch robot.

A ROS2 workspace is a directory containing ROS2 packages. We'll create an `ament_ws` workspace with all the necessary Stretch packages.

**⚠️ Warning**: This will delete any existing `~/ament_ws` directory. Back up important files first.

```bash
# Download and run the workspace setup script
curl -sL https://raw.githubusercontent.com/hello-robot/stretch_ros2/refs/heads/humble/stretch_simulation/stretch_create_ament_workspace.sh > /tmp/stretch_create_ament_workspace.sh && sudo bash /tmp/stretch_create_ament_workspace.sh

# Add ROS2 environment to your shell profile (optional but recommended)
echo 'source ~/ament_ws/install/setup.bash' >> ~/.bashrc
```

**Troubleshooting**: If you encounter a numpy header error during compilation:
```bash
sudo ln -s ~/.local/lib/python3.10/site-packages/numpy/core/ /usr/include/numpy
```

**Expected output**: After successful setup, you should see these packages:
```bash
$ ls ~/ament_ws/src
audio_common  realsense-ros  respeaker_ros2  ros2_numpy  rosbridge_suite  sllidar_ros2  stretch_ros2  stretch_tutorials  stretch_web_teleop  tf2_web_republisher_py
```

### Step 4: Set up Robot Description (URDF) (15 minutes)

URDF (Unified Robot Description Format) files describe the robot's physical structure, joints, and sensors. This step downloads the 3D models and configurations for the Stretch robot.

```bash
# Source the ROS2 environment
source ~/ament_ws/install/setup.bash

# Install Stretch URDF tools
python3 -m pip install -U hello-robot-stretch-urdf

# Download URDF repository
git clone https://github.com/hello-robot/stretch_urdf.git --depth 1 /tmp/stretch_urdf

# Install Stretch body
python3 -m pip install hello-robot-stretch-body

# Update URDF files for ROS2
python3 /tmp/stretch_urdf/tools/stretch_urdf_ros_update.py
python3 /tmp/stretch_urdf/tools/stretch_urdf_ros_update.py --ros2_rebuild
```

**Expected output**: You should see URDF files in the stretch_description package:
```bash
$ ls ~/ament_ws/src/stretch_ros2/stretch_description/urdf/
d405  d435i  stretch_description.xacro  stretch_main.xacro  # ... and many more files
```

### Step 5: Set up Mujoco Simulation (15 minutes)


```bash
# Upgrade pip (important for dependency installation)
pip3 install --upgrade pip

# Source ROS2 environment
source ~/ament_ws/install/setup.bash

# Run interactive setup script (will ask about robocasa environments)
sh ~/ament_ws/src/stretch_ros2/stretch_simulation/stretch_mujoco_driver/setup.sh

# Fix OpenGL compatibility issue
pip install PyOpenGL==3.1.4

# Build the workspace
cd ~/ament_ws
source ./install/setup.bash
colcon build
```

**What is robocasa?** When prompted, robocasa provides realistic kitchen environments for testing household robotics tasks. Choose 'yes' if you want these additional environments.

## Running Your First Simulation

Now let's test the installation by launching the simulation:

```bash
# Source the environment
source ~/ament_ws/install/setup.bash

# Launch the simulation in navigation mode
ros2 launch stretch_simulation stretch_mujoco_driver.launch.py mode:=navigation
```

**Expected output**:
- A Mujoco viewer window should open showing the Stretch robot in a kitchen environment
- ROS2 nodes should start without errors
- You should see log messages indicating successful initialization

**Troubleshooting**:
- If you see GPU-related errors, try: `export MUJOCO_GL=egl` before launching
- If the viewer doesn't open, ensure you have proper graphics drivers installed

## Understanding Simulation Modes

The Stretch simulation supports different control modes:

### Position Mode
Direct joint position control - good for precise movements and testing specific poses.
```bash
ros2 launch stretch_simulation stretch_mujoco_driver.launch.py mode:=position
```

### Navigation Mode
Enables autonomous navigation with obstacle avoidance - used for mapping and path planning.
```bash
ros2 launch stretch_simulation stretch_mujoco_driver.launch.py mode:=navigation
```

### Trajectory Mode
Smooth trajectory execution - useful for complex manipulation tasks.
```bash
ros2 launch stretch_simulation stretch_mujoco_driver.launch.py mode:=trajectory
```

## Basic Robot Control

### Keyboard Teleop

Control the robot using keyboard commands:

```bash
# Terminal 1: Launch simulation
ros2 launch stretch_simulation stretch_mujoco_driver.launch.py mode:=navigation

# Terminal 2: Switch to navigation mode and start teleop
ros2 service call /switch_to_navigation_mode std_srvs/srv/Trigger
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/stretch/cmd_vel
```

**Controls**:
- `i`: Move forward
- `k`: Stop
- `j`: Turn left
- `l`: Turn right
- `u`: Move forward and turn left
- `o`: Move forward and turn right

### Web Interface Control

The web interface provides a graphical way to control the robot:

```bash
# Set up web teleop (one-time setup)
cd ~/ament_ws/src/stretch_ros2/stretch_description/urdf
cp ./stretch_uncalibrated.urdf stretch.urdf

sudo apt install rpl
./export_urdf.sh

mkdir -p $HELLO_FLEET_PATH/$HELLO_FLEET_ID/exported_urdf
cp -r ./exported_urdf/* $HELLO_FLEET_PATH/$HELLO_FLEET_ID/exported_urdf
```

Launch web teleop:
```bash
# Terminal 1: Simulation with cameras
MUJOCO_GL=egl ros2 launch stretch_simulation stretch_mujoco_driver.launch.py use_mujoco_viewer:=false mode:=position use_cameras:=true

# Terminal 2: Web interface
ros2 launch stretch_simulation stretch_simulation_web_interface.launch.py

# Terminal 3: Local storage server
cd ~/ament_ws/src/stretch_web_teleop
npm run localstorage

# Terminal 4: Main web server (requires sudo for port 80)
cd ~/ament_ws/src/stretch_web_teleop
sudo node ./server.js

# Terminal 5: Robot browser interface
cd ~/ament_ws/src/stretch_web_teleop
node start_robot_browser.js
```

Access the web interface at `http://localhost` in your browser.

## Camera and Sensor Data

Enable cameras to get visual feedback from the robot:

```bash
ros2 launch stretch_simulation stretch_mujoco_driver.launch.py use_cameras:=true mode:=navigation
```

**Available camera topics**:
- `/stretch/camera/color/image_raw`: Head camera RGB
- `/stretch/camera/depth/image_raw`: Head camera depth
- `/stretch/gripper_camera/color/image_raw`: Gripper camera RGB
- `/stretch/gripper_camera/depth/image_raw`: Gripper camera depth
- `/stretch/camera/pointcloud`: Head camera point cloud
- `/stretch/gripper_camera/pointcloud`: Gripper camera point cloud

View camera feeds:
```bash
# View RGB image from head camera
ros2 run rqt_image_view rqt_image_view /stretch/camera/color/image_raw

# View point cloud in RViz
rviz2 -d ~/ament_ws/src/stretch_ros2/stretch_core/rviz/stretch_simple_test.rviz
```

## Navigation and Mapping

### Creating a Map

SLAM (Simultaneous Localization and Mapping) allows the robot to build a map while navigating:

```bash
# Terminal 1: SLAM Toolbox
ros2 launch stretch_nav2 online_async_launch.py use_sim_time:=true

# Terminal 2: Simulation
export MUJOCO_GL=egl
ros2 launch stretch_simulation stretch_mujoco_driver.launch.py use_mujoco_viewer:=true mode:=navigation

# Terminal 3: Enable navigation and teleop
ros2 service call /switch_to_navigation_mode std_srvs/srv/Trigger
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/stretch/cmd_vel
```

Drive the robot around to build a map, then save it:
```bash
# Create maps directory
mkdir -p ${HELLO_FLEET_PATH}/maps

# Save the map
ros2 run nav2_map_server map_saver_cli -f ${HELLO_FLEET_PATH}/maps/my_kitchen_map
```

### Autonomous Navigation

Use a previously created map for autonomous navigation:

```bash
# Terminal 1: Simulation
ros2 launch stretch_simulation stretch_mujoco_driver.launch.py use_mujoco_viewer:=true mode:=navigation

# Terminal 2: Navigation stack
ros2 service call /switch_to_navigation_mode std_srvs/srv/Trigger
ros2 launch stretch_nav2 navigation.launch.py map:=${HELLO_FLEET_PATH}/maps/my_kitchen_map.yaml use_sim_time:=true use_rviz:=true teleop_type:=none
```

**Using RViz for navigation**:
1. Click "2D Pose Estimate" and click/drag on the map to set robot's initial position
2. Click "2D Nav Goal" and click on the map to send the robot to that location
3. Watch the robot plan and execute a path to the goal

### Pre-mapped Environments

The simulation includes pre-made maps for quick testing:

```bash
# Terminal 1: Launch specific environment
ros2 launch stretch_simulation stretch_mujoco_driver.launch.py use_mujoco_viewer:=true mode:=navigation robocasa_layout:='G-shaped' robocasa_style:=Modern_1

# Terminal 2: Use pre-made map
ros2 launch stretch_nav2 navigation.launch.py map:=~/ament_ws/src/stretch_ros2/stretch_simulation/maps/gshaped_modern1_robocasa.yaml use_sim_time:=true use_rviz:=true teleop_type:=none

# Terminal 3: Configure robot
ros2 service call /stow_the_robot std_srvs/srv/Trigger
ros2 param set /global_costmap/global_costmap inflation_layer.inflation_radius 0.20
ros2 param set /local_costmap/local_costmap inflation_layer.inflation_radius 0.20
```

## ROS2 Integration

### Key Topics

Monitor robot state and sensor data:

```bash
# List all available topics
ros2 topic list

# Key topics for robot control:
ros2 topic echo /stretch/cmd_vel          # Velocity commands
ros2 topic echo /stretch/joint_states     # Joint positions and velocities
ros2 topic echo /stretch/odom            # Robot odometry (position/orientation)

# Sensor topics:
ros2 topic echo /stretch/scan            # Laser scan data
ros2 topic echo /stretch/camera/color/image_raw  # Camera images
```

### Services

Control robot behavior through services:

```bash
# Switch control modes
ros2 service call /switch_to_navigation_mode std_srvs/srv/Trigger
ros2 service call /switch_to_position_mode std_srvs/srv/Trigger

# Robot poses
ros2 service call /stow_the_robot std_srvs/srv/Trigger
ros2 service call /home_the_robot std_srvs/srv/Trigger
```

### Action Servers

For complex movements, use action servers:

```bash
# List available actions
ros2 action list

# Send trajectory commands
ros2 action send_goal /stretch_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: ['joint_lift'],
    points: [{
      positions: [0.5],
      time_from_start: {sec: 2}
    }]
  }
}"
```

## Programming with the Simulation

### Python Example

Create a simple Python script to control the robot:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class StretchController(Node):
    def __init__(self):
        super().__init__('stretch_controller')

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/stretch/cmd_vel', 10)

        # Subscriber for laser scan data
        self.scan_sub = self.create_subscription(
            LaserScan, '/stretch/scan', self.scan_callback, 10)

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        self.obstacle_detected = False

    def scan_callback(self, msg):
        # Check for obstacles in front (center 60 degrees)
        center_start = len(msg.ranges) // 2 - 30
        center_end = len(msg.ranges) // 2 + 30
        center_ranges = msg.ranges[center_start:center_end]

        # Obstacle if anything closer than 1 meter
        self.obstacle_detected = any(r < 1.0 for r in center_ranges if r > 0)

    def control_loop(self):
        twist = Twist()

        if self.obstacle_detected:
            # Turn if obstacle detected
            twist.angular.z = 0.5
            self.get_logger().info('Obstacle detected, turning...')
        else:
            # Move forward if clear
            twist.linear.x = 0.2
            self.get_logger().info('Moving forward...')

        self.cmd_vel_pub.publish(twist)

def main():
    rclpy.init()
    controller = StretchController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Save as `stretch_simple_controller.py` and run:
```bash
python3 stretch_simple_controller.py
```

## Customization Options

### Environment Selection

Choose different kitchen layouts and styles:

```bash
# Available layouts: 'Random', 'One wall', 'L-shaped', 'U-shaped', 'G-shaped', etc.
# Available styles: 'Random', 'Modern_1', 'Traditional_1', 'Scandanavian', etc.

ros2 launch stretch_simulation stretch_mujoco_driver.launch.py \
  robocasa_layout:='U-shaped' \
  robocasa_style:='Traditional_1' \
  mode:=navigation
```

### Launch Parameters

View all available options:
```bash
ros2 launch stretch_simulation stretch_mujoco_driver.launch.py --show-args
```

Common parameters:

- `use_mujoco_viewer:=true/false`: Show/hide 3D viewer

- `use_cameras:=true/false`: Enable/disable camera sensors

- `use_rviz:=true/false`: Launch RViz visualization

- `mode:=position/navigation/trajectory`: Control mode

- `broadcast_odom_tf:=true/false`: Publish odometry transforms

## Troubleshooting

### Common Issues

**1. "MUJOCO_GL" errors**
```bash
export MUJOCO_GL=egl  # For hardware acceleration
```

**2. Slow performance**
- Reduce visual quality: `use_mujoco_viewer:=false`

- Use `export MUJOCO_GL=egl` for hardware acceleration

- Close unnecessary applications

- Check GPU drivers are properly installed

**3. ROS2 nodes not communicating**
```bash
# Check if nodes are running
ros2 node list

# Check topic connections
ros2 topic info /stretch/cmd_vel

# Restart simulation if needed
```

**4. Camera topics not publishing**
- Ensure `use_cameras:=true` is set

- Check camera topics: `ros2 topic list | grep camera`

**5. Navigation not working**

- Verify map file exists and path is correct

- Check if robot is properly localized in RViz

- Ensure `use_sim_time:=true` is set for navigation nodes

The simulation provides a safe environment to test your code before deploying to the real robot. Happy coding!