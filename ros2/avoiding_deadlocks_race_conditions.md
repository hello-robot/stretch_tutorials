# Avoiding Race Conditions and Deadlocks
ROS 2 was created with real time safety in mind. Some of the features of ROS 2, although exposed for the convenience of users, can get in the way if one is not aware about them.

## Race conditions
In parallel programming, [race conditions]() can occur when data is being written to and read from a shared memory space concurrently by two threads. This creates a condition where a thread that’s reading from the shared memory gets conflicting data. Such conditions are often difficult to debug as it requires one to introspect multiple threads. Hence, it is always better to recognize and avoid common pitfalls that lead to race conditions.

## Deadlocks
In parallel programming, [deadlocks]() occur when multiple threads try to gain access to the same system resource together. In the context of ROS 2, this can occur when a resource like the executor which is shared by multiple callbacks is asked to service more than one callback concurrently. Again, this is another pitfall that’s difficult to debug as it often manifests itself with nodes becoming unresponsive without any apparent error message. Hence, it is always better to recognize and avoid common pitfalls that lead to deadlocks.

## Executor
We had a brief introduction to the ROS 2 executor in the previous tutorial on [ROS 2 with rclpy](). There, we looked at ways to invoke the executor with various spin methods. At a high level, the job of the executor is to service callback messages as they arrive and process and relay messages in the callback. In parallel programming, sometimes you might want certain parts of a process to run concurrently to avoid deadlocks, or in some cases, you might not want certain parts of a process to run concurrently to avoid deadlocks. Fortunately, you can control this by defining the executor model.

An executor is defined as a SingleThreadedExecutor by default which is perfect for servicing fast running callbacks sequentially. However, if parallelism is desired, an executor can be defined as a MultiThreadedExecutor. This allows long running callbacks to run in parallel with fast running ones. A custom executor model can also be defined, but that is beyond the scope of this tutorial.

## Callback groups
A callback is the method that receives a ROS message and is where a ROS message is processed. Depending on the kind of data that’s being worked upon, a callback method could finish executing in no time or take several seconds to process. Several seconds is a long time in programming and the longer a callback method takes, the longer the next callback has to wait for the executor to service it.

With a MultiThreadedExecutor, callbacks can be serviced in parallel. However, this makes the process prone to race conditions as multiple threads work on the same shared memory. Fortunately, this can be avoided by defining callback groups. Grouping callbacks such that the ones that deal with the same shared memory space never execute concurrently ensures that data doesn’t get corrupted due to race conditions.

There are two different kinds of callback groups available. A MutuallyExclusiveCallbackGroup, the default, ensures that the callbacks belonging to this group never execute in parallel. You would use this when two callbacks access and write to the same shared memory space and having them execute them together would result in a race condition. A ReentrantCallbackGroup ensures that callbacks belonging to this group are able to execute parallelly. You would use this when a long running callback occupies the bulk of the executors time and you want shorter fast running callbacks to run in parallel.

Now, let’s explore what we have learned so far in the form of a real example.

## Race Condition Example
It is instructive to see an example code that generates a race condition. The below code simulates a race condition by defining two subscriber callbacks that write and read from shared memory simultaneously.

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

Executing the above code, you are presented with two prompts, first to select the executor, either a SingleThreadedExecutor or a MultiThreadedExecutor; and then to select a callback group type, either a MutuallyExclusiveCallbackGroup or a ReentrantCallbackGroup.

Selecting a SingleThreadedExecutor, irrespective of which callback group is selected, results in callbacks being executed sequentially. This is because the executor is spun using a single thread that can only service one callback at a time. In this case, we see that there is no memory curroption as True or False appear in the right sequence.

Things get interesting when we choose the MultiThreadedExecutor along with a ReentrantCallbackGroup. This mimics a case when you would want callbacks to execute in parallel in two separate threads. However, doing so exposes shared memory spaces to corruption as multiple threads gain access to the same memory space to read and write data at the same time. This might give rise to a condition where one thread writes data according to a certain observation, but right before the write is executed, the data based on which the observation was made changes altogether because of a separate thread writing to the same memory space. 

