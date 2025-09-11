# Writing ROS2 Nodes

## Introduction

ROS2 nodes are the fundamental building blocks of any ROS2 application. A node is an executable that uses ROS2 to communicate with other nodes. Nodes can publish messages to topics, subscribe to topics to receive messages, provide services, or call services provided by other nodes. In this tutorial, you'll learn how to create your own ROS2 Python package and write a simple node that subscribes to Stretch's joint states.

Think of nodes as individual programs that each have a specific responsibility - one node might handle camera data, another might control the robot's movement, and yet another might process sensor information. By breaking functionality into separate nodes, ROS2 applications become modular, easier to debug, and more maintainable.

## Prerequisites

Before starting this tutorial, you should have:

1. **Basic Python knowledge**: Understanding of Python syntax, functions, and classes
2. **ROS2 installation**: ROS2 Humble installed on your system (this comes pre-installed on Stretch robots)
3. **Stretch robot setup**: Access to a Stretch robot or simulation environment
4. **Basic terminal/command line familiarity**: Ability to navigate directories and run commands

If you're new to ROS2 concepts, we recommend first reading through the [Introduction to ROS2](intro_to_ros2.md) tutorial.

## Understanding the ROS2 Workspace Structure

Before we create our first node, let's understand how ROS2 organizes code. ROS2 uses a workspace structure where all your packages live in a `src` directory within your workspace. The standard workspace structure looks like this:

```
~/ament_ws/                    # Your workspace root
├── build/                     # Build artifacts (auto-generated)
├── install/                   # Installed packages (auto-generated)
├── log/                       # Build logs (auto-generated)
└── src/                       # Your source code goes here
    ├── package1/
    ├── package2/
    └── your_new_package/
```

## Step 1: Creating a New ROS2 Python Package

Let's start by creating a new ROS2 package. A package is a collection of related nodes, launch files, configuration files, and other resources.

First, navigate to your workspace source directory:

```{.bash .shell-prompt}
cd ~/ament_ws/src
```

Now create a new Python package using the `ros2 pkg create` command:

```{.bash .shell-prompt}
ros2 pkg create --build-type ament_python stretch_joint_reader
```

This command creates a new package called `stretch_joint_reader` with the following structure:

```
stretch_joint_reader/
├── package.xml              # Package metadata and dependencies
├── resource/
│   └── stretch_joint_reader
├── setup.cfg               # Setup configuration
├── setup.py               # Python package setup
├── stretch_joint_reader/  # Python module directory
│   └── __init__.py
└── test/                  # Test files
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py
```

Let's examine what was created:

```{.bash .shell-prompt}
cd stretch_joint_reader
ls -la
```

## Step 2: Understanding Package Configuration

Let's look at the key configuration files that were generated.

### package.xml

This file contains metadata about your package. Open it to see the current contents:

```{.bash .shell-prompt}
cat package.xml
```

The `package.xml` file defines dependencies, maintainer information, and other package metadata. For our joint state subscriber, we need to add a dependency on the `sensor_msgs` package, which contains the `JointState` message type.

### setup.py

This file tells Python how to install your package. Let's examine it:

```{.bash .shell-prompt}
cat setup.py
```

We'll need to modify this file to include our new node as an entry point so ROS2 can find and run it.

## Step 3: Writing Your First Node

Now let's create our first ROS2 node that subscribes to the `/joint_states` topic and prints information about Stretch's joints.

Create a new Python file for our node:

```{.bash .shell-prompt}
cd stretch_joint_reader
touch joint_state_subscriber.py
```

Open the file in your preferred text editor and add the following code:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateSubscriber(Node):
    """
    A ROS2 node that subscribes to joint states and prints joint information.
    """

    def __init__(self):
        # Initialize the node with a name
        super().__init__('joint_state_subscriber')

        # Create a subscription to the /joint_states topic
        self.subscription = self.create_subscription(
            JointState,                    # Message type
            '/joint_states',               # Topic name
            self.joint_state_callback,     # Callback function
            10                            # Queue size
        )

        # Prevent unused variable warning
        self.subscription

        # Log that the node has started
        self.get_logger().info('Joint State Subscriber node has started!')
        self.get_logger().info('Listening for joint states on /joint_states topic...')

    def joint_state_callback(self, msg):
        """
        Callback function that gets called whenever a new JointState message is received.

        Args:
            msg (JointState): The received joint state message
        """
        # Log basic information about the message
        self.get_logger().info(f'Received joint states for {len(msg.name)} joints')

        # Print detailed information about each joint
        self.get_logger().info('Joint Information:')
        for i, joint_name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else 'N/A'
            velocity = msg.velocity[i] if i < len(msg.velocity) else 'N/A'
            effort = msg.effort[i] if i < len(msg.effort) else 'N/A'

            self.get_logger().info(
                f'  {joint_name}: pos={position:.4f}, vel={velocity:.4f}, effort={effort:.4f}'
            )

        self.get_logger().info('---')

def main(args=None):
    """
    Main function to initialize ROS2, create the node, and spin.
    """
    # Initialize ROS2 Python client library
    rclpy.init(args=args)

    # Create an instance of our node
    joint_state_subscriber = JointStateSubscriber()

    try:
        # Spin the node to keep it alive and processing callbacks
        rclpy.spin(joint_state_subscriber)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        joint_state_subscriber.get_logger().info('Shutting down joint state subscriber...')
    finally:
        # Clean up
        joint_state_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Code Explanation

Let's break down what this code does:

1. **Imports**: We import the necessary ROS2 Python libraries and message types
2. **Class Definition**: We create a class that inherits from `Node`, which is the base class for all ROS2 nodes
3. **Constructor (`__init__`)**:
   - Calls the parent constructor with our node name
   - Creates a subscription to the `/joint_states` topic
   - Sets up logging messages
4. **Callback Function**: This function gets called every time a new message arrives on the subscribed topic
5. **Main Function**: Initializes ROS2, creates our node, and keeps it running

## Step 4: Configuring Package Dependencies and Entry Points

Now we need to update our package configuration to include the necessary dependencies and make our node executable.

### Update package.xml

Edit the `package.xml` file to add the required dependencies. Add these lines after the existing `<depend>` tags:

```xml
<depend>rclpy</depend>
<depend>sensor_msgs</depend>
```

### Update setup.py

Edit the `setup.py` file to add our node as an entry point. Find the `entry_points` section and modify it to look like this:

```python
entry_points={
    'console_scripts': [
        'joint_state_subscriber = stretch_joint_reader.joint_state_subscriber:main',
    ],
},
```

This tells ROS2 that when someone runs `ros2 run stretch_joint_reader joint_state_subscriber`, it should execute the `main` function from our `joint_state_subscriber.py` file.

## Step 5: Building Your Package

Now that we've created our node and configured our package, we need to build it. ROS2 uses the `colcon` build system.

Navigate back to your workspace root:

```{.bash .shell-prompt}
cd ~/ament_ws
```

Build your package:

```{.bash .shell-prompt}
colcon build --packages-select stretch_joint_reader
```

If the build is successful, you should see output similar to:

```{.bash .no-copy}
Starting >>> stretch_joint_reader
Finished <<< stretch_joint_reader [2.34s]

Summary: 1 package finished [2.56s]
```

If you encounter any errors, double-check that:
- Your Python code has correct indentation
- The `package.xml` and `setup.py` files are properly formatted
- All file names match exactly

After building, source the setup script to make your new package available:

```{.bash .shell-prompt}
source install/setup.bash
```

!!! note
    You need to source the setup script in every new terminal where you want to use your custom packages. To avoid doing this every time, you can add the source command to your `~/.bashrc` file:
    ```bash
    echo "source ~/ament_ws/install/setup.bash" >> ~/.bashrc
    ```

## Step 6: Running Your Node

Now comes the exciting part - running your node! But first, we need to make sure the Stretch robot driver is running so that joint state messages are being published.

### Start the Stretch Driver

In one terminal, launch the Stretch driver:

```{.bash .shell-prompt}
ros2 launch stretch_core stretch_driver.launch.py
```

Wait for the driver to fully initialize. You should see messages indicating that the robot is ready.

### Run Your Node

In a new terminal (remember to source your workspace), run your joint state subscriber:

```{.bash .shell-prompt}
cd ~/ament_ws
source install/setup.bash
ros2 run stretch_joint_reader joint_state_subscriber
```

You should see output similar to:

```{.bash .no-copy}
[INFO] [1699123456.789] [joint_state_subscriber]: Joint State Subscriber node has started!
[INFO] [1699123456.790] [joint_state_subscriber]: Listening for joint states on /joint_states topic...
[INFO] [1699123456.891] [joint_state_subscriber]: Received joint states for 15 joints
[INFO] [1699123456.891] [joint_state_subscriber]: Joint Information:
[INFO] [1699123456.891] [joint_state_subscriber]:   joint_left_wheel: pos=0.0000, vel=0.0000, effort=0.0000
[INFO] [1699123456.891] [joint_state_subscriber]:   joint_right_wheel: pos=0.0000, vel=0.0000, effort=0.0000
[INFO] [1699123456.891] [joint_state_subscriber]:   joint_lift: pos=0.2500, vel=0.0000, effort=0.0000
[INFO] [1699123456.891] [joint_state_subscriber]:   wrist_extension: pos=0.0000, vel=0.0000, effort=0.0000
[INFO] [1699123456.891] [joint_state_subscriber]:   joint_wrist_yaw: pos=0.0000, vel=0.0000, effort=0.0000
...
```

Congratulations! Your node is now receiving and displaying joint state information from Stretch.

### Understanding the Output

The joint state messages contain information about all of Stretch's joints:

- **Position**: Current joint position (in radians for revolute joints, meters for prismatic joints)
- **Velocity**: Current joint velocity
- **Effort**: Current effort/torque being applied to the joint

Key joints you'll see include:
- `joint_lift`: The vertical lift mechanism
- `wrist_extension`: The telescoping arm extension
- `joint_wrist_yaw`: Wrist rotation
- `joint_left_wheel` and `joint_right_wheel`: The mobile base wheels
- Various gripper joints for the fingers

## Step 7: Enhancing Your Node

Let's make our node more useful by filtering for specific joints and adding some additional functionality.

Create a new file called `filtered_joint_subscriber.py`:

```{.bash .shell-prompt}
cd ~/ament_ws/src/stretch_joint_reader/stretch_joint_reader
touch filtered_joint_subscriber.py
```

Add the following enhanced code:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class FilteredJointStateSubscriber(Node):
    """
    An enhanced ROS2 node that subscribes to joint states and filters for specific joints.
    """

    def __init__(self):
        super().__init__('filtered_joint_state_subscriber')

        # Define which joints we're interested in
        self.joints_of_interest = [
            'joint_lift',
            'wrist_extension',
            'joint_wrist_yaw',
            'joint_gripper_finger_left',
            'joint_gripper_finger_right'
        ]

        # Create subscription
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Create a timer to periodically print a summary
        self.timer = self.create_timer(5.0, self.print_summary)

        # Store the latest joint states
        self.latest_joint_states = {}

        self.get_logger().info('Filtered Joint State Subscriber started!')
        self.get_logger().info(f'Monitoring joints: {", ".join(self.joints_of_interest)}')

    def joint_state_callback(self, msg):
        """Process incoming joint state messages."""
        # Update our stored joint states
        for i, joint_name in enumerate(msg.name):
            if joint_name in self.joints_of_interest:
                self.latest_joint_states[joint_name] = {
                    'position': msg.position[i] if i < len(msg.position) else 0.0,
                    'velocity': msg.velocity[i] if i < len(msg.velocity) else 0.0,
                    'effort': msg.effort[i] if i < len(msg.effort) else 0.0
                }

    def print_summary(self):
        """Print a summary of current joint states."""
        if not self.latest_joint_states:
            self.get_logger().warn('No joint state data received yet...')
            return

        self.get_logger().info('=== Joint State Summary ===')
        for joint_name in self.joints_of_interest:
            if joint_name in self.latest_joint_states:
                state = self.latest_joint_states[joint_name]
                self.get_logger().info(
                    f'{joint_name:25}: {state["position"]:8.4f} rad/m'
                )
        self.get_logger().info('===========================')

def main(args=None):
    rclpy.init(args=args)
    node = FilteredJointStateSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Update your `setup.py` to include this new node:

```python
entry_points={
    'console_scripts': [
        'joint_state_subscriber = stretch_joint_reader.joint_state_subscriber:main',
        'filtered_joint_subscriber = stretch_joint_reader.filtered_joint_subscriber:main',
    ],
},
```

Rebuild your package:

```{.bash .shell-prompt}
cd ~/ament_ws
colcon build --packages-select stretch_joint_reader
source install/setup.bash
```

Run the enhanced node:

```{.bash .shell-prompt}
ros2 run stretch_joint_reader filtered_joint_subscriber
```

This enhanced version will print a summary every 5 seconds showing only the joints you're interested in, making it easier to monitor specific parts of the robot.

## Common Troubleshooting

### Build Errors

**Problem**: `colcon build` fails with Python syntax errors
- **Solution**: Check your Python code for proper indentation and syntax. Python is sensitive to indentation - use consistent spaces (typically 4 spaces per level).

**Problem**: Package not found during build
- **Solution**: Make sure you're in the correct workspace directory (`~/ament_ws`) and that your package is in the `src` folder.

**Problem**: Missing dependencies error
- **Solution**: Ensure all required dependencies are listed in your `package.xml` file.

### Runtime Errors

**Problem**: `ros2 run` command not found or package not found
- **Solution**: Make sure you've sourced your workspace: `source ~/ament_ws/install/setup.bash`

**Problem**: Node starts but no messages are received
- **Solution**:
  1. Check that the Stretch driver is running: `ros2 launch stretch_core stretch_driver.launch.py`
  2. Verify the topic exists: `ros2 topic list | grep joint_states`
  3. Check if messages are being published: `ros2 topic echo /joint_states`

**Problem**: Permission denied when running the node
- **Solution**: Make sure your Python file has execute permissions: `chmod +x joint_state_subscriber.py`

### Debugging Tips

1. **Use ROS2 command-line tools**:
   ```bash
   # List all available topics
   ros2 topic list

   # See what's being published on a topic
   ros2 topic echo /joint_states

   # Check topic information
   ros2 topic info /joint_states

   # List running nodes
   ros2 node list
   ```

2. **Add more logging**: Use `self.get_logger().info()`, `.warn()`, `.error()`, or `.debug()` to add debugging information to your code.

3. **Check the ROS2 graph**: Use `rqt_graph` to visualize the connections between nodes:
   ```bash
   ros2 run rqt_graph rqt_graph
   ```

## Understanding ROS2 Concepts

### Topics and Messages

- **Topics**: Named channels for data communication between nodes
- **Messages**: Data structures that define the format of information sent over topics
- **Publishers**: Nodes that send messages to topics
- **Subscribers**: Nodes that receive messages from topics

### Quality of Service (QoS)

The queue size parameter (10 in our examples) is part of ROS2's Quality of Service system. It determines how many messages to buffer if your callback function can't process them fast enough.

### Node Lifecycle

1. **Initialization**: `rclpy.init()` sets up the ROS2 runtime
2. **Node Creation**: Creating your node instance
3. **Spinning**: `rclpy.spin()` keeps the node alive and processes callbacks
4. **Cleanup**: `destroy_node()` and `rclpy.shutdown()` clean up resources
