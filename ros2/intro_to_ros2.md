# Introduction to ROS 2
In this tutorial we will explore rclpy, the client library for interacting with ROS 2 using the Python API. The rclpy library forms the base of ROS 2 and you will notice that all tutorials in the following sections will use it. In this section we will focus on a few common constructs of rclpy and then follow some examples using the IPython interpreter to get familiar with them.

## IPython
It is not always necessary to write a functional Python script while prototyping or exploring a new library. It's instructive and helpful to use what’s called an REPL (Read-Eval-Print Loop), which quickly allows us to execute Python instructions and see their output immediately. This allows better understanding of what each instruction does and is often a great way to debug unexpected behavior. IPython is a command line based interactive interpreter for Python that uses REPL and can be used to run Python snippets for quick prototyping.

To run IPython in a terminal, simply execute:
```{.bash .shell-prompt}
python3 -m IPython
```

Try out the following snippets for a ROS 2 quickstart:

## Initializationa and Shutdown
### rclpy.init()
All rclpy functionality can be exposed after initialization:
```{.bash .shell-prompt}
import rclpy

rclpy.init()
```

### rclpy.create_node()
To create a new ROS 2 node, one can use the create_node method with the node name as the argument:
```{.bash .shell-prompt}
node = rclpy.create_node('temp')
```

### rclpy.logging.get_logger()
The rclpy library also provides a logger to print messages with different severity levels to stdout. Here’s how you can use it:
```{.bash .shell-prompt}
import rclpy.logging
logger = rclpy.logging.get_logger('temp')
logger.info("Hello")
logger.warn("Robot")
logger.error("Stretch")
```

### rclpy.ok()
If you want to check whether rclpy has been initialized, you can run the following snippet. This is especially useful to simulate an infinite loop based on whether rclpy has been shutdown.
```{.bash .shell-prompt}
import time

while rclpy.ok():
	print("Hello")
	time.sleep(1.0)
```

Press ctrl+c to get out of the infinite loop.

### rclpy.shutdown()
Finally, to destroy a node safely and shutdown the instance of rclpy you can run:

```{.bash .shell-prompt}
node.destroy_node()
rclpy.shutdown()
```

## Publishing and subscribing
### create_publisher()
ROS 2 is a distributed communication system and one way to send data is through a publisher. It takes the following arguments: msg_type, msg_topic and a history depth (formerly queue_size):
```{.bash .shell-prompt}
from std_msgs.msg import String
import rclpy

rclpy.init()
node = rclpy.create_node('temp')
pub = node.create_publisher(String, 'hello', 10)
```

### create_subscription()
To receive a message, we need to create a subscriber with a callback function that listens to the arriving messages. Let's create a subscriber and define a callback called hello_callback() that logs the a message as soon as one is received:
```{.bash .shell-prompt}
def hello_callback(msg):
	print("Received message: {}".format(msg.data))

sub = node.create_subscription(String, 'hello', hello_callback, 10)
```

### publish()
Now that you have defined a publisher and a subscriber, let’s send a message and see if it gets printed to the console:
```{.bash .shell-prompt}
msg = String()
msg.data = "Hello"
pub.publish(msg)
```

### rclpy.spin_once()
That didn’t do it! Although the message was sent, it didn't get printed to the console. Why? Because the hello_callback() method was never called to print the message. In ROS, we don’t call this method manually, but rather leave it to what’s called the executor. The executor can be invoked by calling the spin_once() method. We pass the node object and a timeout of 2 seconds as the arguments. The timeout is important because the spin_once() method is blocking and it will wait for a message to arrive indefinitely if a timeout is not defined. It returns immediately once a message is received.
```{.bash .shell-prompt}
rclpy.spin_once(node, timeout_sec=2.0)
```

### rclpy.spin()
The spin_once() method only does work equivalent to a single message callback. What if you want the executor to process callbacks continuously? This can be achieved using the spin() method. While retaining the current interpreter instance, let’s open a new terminal window with a new instance of IPython and execute the following:

Terminal 2:
```{.bash .shell-prompt}
import rclpy
from std_msgs.msg import String
rclpy.init()
node = rclpy.create_node('temp2')
def hello_callback(msg):
	print("I heard: {}".format(msg.data))
sub = node.create_subscription(String, 'hello', hello_callback, 10)
rclpy.spin(node)
```

Now, from the first IPython instance, send a series of messages and see what happens:

Terminal 1:
```{.bash .shell-prompt}
for i in range(10):
	msg.data = "Hello {}".format(i)
	pub.publish(msg)
```

Voila! Finally, close both the terminals to end the session.

## Service Server and Client
### create_service()
Let’s explore another common way of using ROS 2. Imagine a case where you need to request some information from a node and you expect to receive a response. This can be achieved using the service client paradigm in ROS 2. Let’s fire up IPython again and create a quick service:
```{.bash .shell-prompt}
import rclpy
from example_interfaces.srv import AddTwoInts
rclpy.init()

def add_ints(req, res):
 	print("Received request")
 	res.sum = req.a + req.b
 	return res

node = rclpy.create_node('temp')
srv = node.create_service(AddTwoInts, 'add_ints', add_ints)

# you need to spin to receive the request
rclpy.spin_once(node, timeout_sec=2.0)
```

The add_ints() method is the callback method for the service server. Once a service request is received, this method will act on it to generate the response. Since a service request is a ROS message, we need to invoke the executor with a spin method to receive the message.

### create_client()
Now, while retaining the current IPython session, open another session of the IPython interpreter in another terminal to write the service client:
```{.bash .shell-prompt}
import rclpy
from example_interfaces.srv import AddTwoInts
rclpy.init()
node = rclpy.create_node('temp2')

cli = node.create_client(AddTwoInts, 'add_ints')

req = AddTwoInts.Request()
req.a = 10
req.b = 20

req_future = cli.call_async(req)
rclpy.spin_until_future_complete(node, req_future, timeout_sec=2.0)

print("Received response: {}".format(req_future.result().sum))
```

### rclpy.spin_until_future_complete()
Notice that the spin method manifests itself as the spin_until_future_complete() method which takes the node, future and timeout_sec as the arguments. The future is an object in ROS 2 that’s returned immediately after an async service call has been made. We can then wait on the result of this future. This way the call to the service is not blocking and the code execution can continue as soon as the service call is issued.
