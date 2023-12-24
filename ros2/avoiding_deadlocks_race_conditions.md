# Avoiding Race Conditions and Deadlocks
ROS 2 was created with real time safety in mind. Some of the features of ROS 2, although exposed for the convenience of users, can get in the way if one is not aware about them.

## Race conditions
In parallel programming, race conditions can occur when data is being written to and read from a shared memory space concurrently by two threads. This creates a condition where a thread that’s reading from the shared memory gets conflicting data. Such conditions are often difficult to debug as it requires one to introspect multiple threads. Hence, it is always better to recognize and avoid common pitfalls that lead to race conditions.

## Deadlocks
In parallel programming, deadlocks occur when multiple threads try to gain access to the same system resource at the same time. In the context of ROS 2, this can occur when a resource like the executor which is shared by multiple callback methods is asked to service more than one callback concurrently. Again, this is another pitfall that’s difficult to debug as it often manifests itself with nodes becoming unresponsive without any apparent error message. Hence, it is always better to recognize and avoid common pitfalls that lead to deadlocks.

## Executor
We had a brief introduction to the ROS 2 executor in the previous tutorial on [Introduction to ROS 2](https://docs.hello-robot.com/0.2/stretch-tutorials/ros2/intro_to_ros2/). We looked at ways to invoke the executor with various spin methods. At a high level, the job of the executor is to service callback messages as they arrive and process and relay messages in the callback. In parallel programming, sometimes you might want parts of a process to run concurrently to avoid deadlocks, or in some cases, you might not want certain parts of a process to run concurrently to avoid race conditions. Fortunately, you can control this by defining the executor model.

An executor is defined as a SingleThreadedExecutor by default which is perfect for servicing fast running callbacks sequentially. However, if parallelism is desired, an executor can be defined as a MultiThreadedExecutor. This allows long running callbacks to run in parallel with fast running ones. A custom executor model can also be defined, but that is beyond the scope of this tutorial.

## Callback groups
A callback is the method that receives a ROS message and is where a ROS message is processed. Depending on the kind of data that’s being worked upon, a callback method could finish executing in virtually no time or take several seconds to process. Several seconds is a long time in programming and the longer a callback method takes, the longer the next callback has to wait for the executor to service it.

With a MultiThreadedExecutor, callbacks can be serviced in parallel. However, this makes the process prone to race conditions as multiple threads work on the same shared memory. Fortunately, this can be avoided by defining callback groups. Grouping callbacks such that the ones that deal with the same shared memory space never execute concurrently ensures that data doesn’t get corrupted due to race conditions.

There are two different kinds of callback groups available. A MutuallyExclusiveCallbackGroup, the default, ensures that the callbacks belonging to this group never execute in parallel. You would use this when two callbacks access and write to the same shared memory space and having them execute them together would result in a race condition. A ReentrantCallbackGroup ensures that callbacks belonging to this group are able to execute parallelly. You would use this when a long running callback occupies the bulk of the executors time and you want shorter fast running callbacks to run in parallel.

Now, let’s explore what we have learned so far in the form of a real example. You can try them by creating a Python file and run it in your terminal as `python3 FILE_NAME`.

## Race Condition Example
It is instructive to see an example code that generates a race condition. The below code simulates a race condition by defining two subscriber callbacks that write and read from shared memory simultaneously.
!!! note
	Before executing the Race Condition Example you first need to launch the stretch core driver as ros2 launch stretch_core stretch_driver.launch.py

```python
import rclpy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import SingleThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
import time

class ParallelExec(Node):
	def __init__(self, cb_group):
		super().__init__('parallel_execution')
		self.shared_memory = True
		self.joint_state = JointState()
		self.mode = String()
		self.cb_group = cb_group
		
	def callback_one(self, msg):
		self.joint_states = msg
		if not self.shared_memory:
			time.sleep(0.2)
			self.get_logger().info("Switching from {} to True".format(self.shared_memory))
			self.shared_memory = True
		
	def callback_two(self, msg):
		self.mode = msg
		if self.shared_memory:
			time.sleep(0.2)
			self.get_logger().info("Switching from {} to False".format(self.shared_memory))
			self.shared_memory = False
		
	def main(self):			
		sub_joint_states = self.create_subscription(JointState, '/stretch/joint_states', self.callback_one, 1, callback_group=self.cb_group)
		sub_mode = self.create_subscription(String, 'mode', self.callback_two, 1, callback_group=self.cb_group)
		
if __name__ == '__main__':
	rclpy.init()
	
	executor = None
	cb_group = None
	
	x = input("Select the executor model: Press 1 for SingleThreadedExecutor(); Press 2 for MultiThreadedExecutor()")
	if x == '1':
		print("Using single-threaded execution")
		executor = SingleThreadedExecutor()
	elif x == '2':
		print("Using multi-threaded execution")
		executor = MultiThreadedExecutor(num_threads=2)
		
	y = input("Select the callback group: Press 1 for MutuallyExclusiveCallbackGroup(); Press 2 for ReentrantCallbackGroup()")
	if y == '1':
		print("Processing callbacks one after the other")
		cb_group = MutuallyExclusiveCallbackGroup()
	elif y == '2':
		print("Processing callbacks in parallel")
		cb_group = ReentrantCallbackGroup()
	
	node = ParallelExec(cb_group)
	node.main()
	
	executor.add_node(node)
	executor.spin()
	
	node.destroy_node()
	rclpy.shutdown()
```

Executing the above script, the expected behavior is to see the logged statements conform to the conditional statements in the callback methods. To elaborate, callback_one() receives joint_state messages and should print "Switching from False to True" only if shared_memory is False. Whereas callback_two() receives mode messages and should print "Switching from True to False" only if shared_memory is True. However, without setting up protection against race conditions we see that this is not what ends up happening.

Executing the above code, you are presented with two prompts, first to select the executor, either a SingleThreadedExecutor or a MultiThreadedExecutor; and then to select a callback group type, either a MutuallyExclusiveCallbackGroup or a ReentrantCallbackGroup.

Selecting a SingleThreadedExecutor, irrespective of which callback group is selected, results in callbacks being executed sequentially. This is because the executor is spun using a single thread that can only service one callback at a time. In this case, we see that there is no memory corruption and the observed behavior is the same as the expected behavior.

Things get interesting when we choose the MultiThreadedExecutor along with a ReentrantCallbackGroup. Multiple threads are used by the executor to service callbacks, while callbacks are allowed to execute in parallel. This allows multiple threads to access the same memory space and execute read/write operations. The observed behavior is that, sometimes you see the callbacks print statements like  "Switching from True to True" or "Switching from False to False" which go against the conditions set in the callbacks. This is a race condition.

Selecting a MultiThreadedExecutor along with a MutuallyExclusiveCallbackGroup allows us to circumvent this problem by using parallelism but still protecting shared memory from race conditions.

## Deadlock Example

A great example of a deadlock is provided in the official ROS 2 documentation on [sync deadlock](https://docs.ros.org/en/iron/How-To-Guides/Sync-Vs-Async.html#sync-deadlock), so this example will directly build off of the same code. The server side defines a callback method add_two_ints_callback() which returns the sum of two requested numbers. Notice the call to spin in the main() method which persistently executes the callback method as a service request arrives. For the requested numbers you will need to input them in the terminal manually.

```python
from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main():
    rclpy.init()
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Now let's look at the client side. This script makes a synchronous service call to the add_two_ints service. 

```python
import sys
from threading import Thread

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClientSync(Node):
    def __init__(self):
        super().__init__('minimal_client_sync')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self):
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        return self.cli.call(self.req)

def main():
    rclpy.init()
    minimal_client = MinimalClientSync()
    response = minimal_client.send_request()
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (minimal_client.req.a, minimal_client.req.b, response.sum))

    rclpy.spin()
	minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Every call to a service expects a response which is a ROS message that requires a callback to listen to. Although not explicitly defined, this callback method is created when the client is initialized. Like any other callback method, the executor needs to call the callback method for the response to be received. Notice that the above example only invokes the executor (with rclpy.spin()) after the synchronous service call has been issued through `self.cli.call(self.req)` in the send_request() method. This creates a deadlock because the execution can't move forward until a response is received, but the executor has not been invoked yet to service the response callback method.

The way to get around this is to invoke the executor in a separate thread before the synchronous service call is made. This way, the response callback is called in a separate thread from the main execution thread. Here's how to do this:

```python
def main():
    rclpy.init()
    minimal_client = MinimalClientSync()
    
	# Invoke the executor in a separate thread
	spin_thread = Thread(target=rclpy.spin, args=(minimal_client,))
    spin_thread.start()
    
	response = minimal_client.send_request()
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (minimal_client.req.a, minimal_client.req.b, response.sum))
    minimal_client.destroy_node()
    rclpy.shutdown()
```

An alternative to this, a feature that's new to ROS 2, is to use an asynchronous service call. This allows one to monitor the response through what's called a future object in ROS 2. The future holds the status of whether a service call was accepted by the server and also the returned response. Since the future is returned immediately on making an async service call, the execution is not blocked and the executor can be invoked in the main execution thread. Here's how to do it:

```python
class MinimalClientAsync(Node):
	
	...

	def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        
		# Make an async call and wait until the executor receives a response
		self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    
	# The response is a future object which can be queried to get the result
	response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()
```