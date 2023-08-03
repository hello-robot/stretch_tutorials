# Motivation
The aim of this document is to be the starting point for generating new ROS tutorials and to maintain consistency in structure and tone across the documentation. It starts off by formalizing the key objectives and then goes on to templatize the structure and tone that we should adopt going forward.

## Objectives
1. **Be inclusive of all levels of understanding** - start from the basics and link relevant external content to be concise yet informative
2. **Reinforce that getting started with Stretch is a breeze** - use easy to understand vocab and a friendly tone to encourage readers
3. **Encourage users to read other tutorials** - wherever possible, link other tutorials from the documentation to convey completeness and complexity
4. **Have a clear flow** - start with the theory, show with GIFs what to expect, and then breakdown the code

What follows can be considered the template.

# Major Topic
In this tutorial, we will work with Stretch to explore *the main theme of tutorial* using *primary module* and also learn how to achieve *secondory theme*. If you want to know more about *any previously covered topic* on Stretch and how to get it up and running, we recommend visiting the previous tutorials on [*link to topic*]() and [*link to topic*]().

*Motivation for the problem the topic solves*. The great thing about Stretch is that it comes preloaded with software that makes it a breeze to achieve *theme of the tutorial*.

By the end of this tutorial, we will have a clear idea about how *first minor topic* works and how to use it to achieve *second minor topic* with Stretch. Let's jump in!

## First Minor Topic Title
[PyTorch](https://pytorch.org/) is an open source end-to-end machine learning framework that makes many pretrained production quality neural networks available for general use. In this tutorial we will use the YOLOv5s model trained on the COCO dataset.

[YOLOv5](https://github.com/ultralytics/yolov5) is a popular object detection model that divides a supplied image into a grid and detects objects in each cell of the grid recursively. The YOLOv5s model that we have deployed on Stretch has been pretrained on the [COCO dataset](https://cocodataset.org/#home) which allows Stretch to detect a wide range of day to day objects. However, that’s not all, in this demo we want to go a step further and use this extremely versatile object detection model to extract useful information about the scene.

## Second Minor Topic Title
Now, let’s use what we have learned so far to upgrade the collision avoidance demo in a way that Stretch is able to scan an entire room autonomously without bumping into things or people. To account for dynamic obstacles getting too close to the robot, we will define a keepout distance of 0.4 m - detections below this value stop the robot. To keep Stretch from getting too close to static obstacles, we will define another variable called turning distance of 0.75 m - frontal detections below this value make Stretch turn to the left until it sees a clear path ahead.

Building up on this, let's implement a simple logic for obstacle avoidance. The logic can be broken down into three steps:
1. If the minimum value from the frontal scans is greater than 0.75 m, then continue to move forward
2. If the minimum value from the frontal scans is less than 0.75 m, then turn to the right until this is no longer true
3. If the minimum value from the overall scans is less than 0.4 m, then stop the robot

## Third Minor Topic Title
*If a tutorial covers more than two minor topics, it might be a good idea to break it down into multiple tutorials*

## Warning
Running this tutorial on Stretch might result in *harm to robot, humans or the surrounding environment*. Please ensure *these conditions*. We recommend *taking these actions* to ensure safe operation.

## See It In Action
Go ahead and execute the following commands to run the demo and visualize the result in RViz:
Terminal 1:
```{.bash .shell-prompt}
Enter first command here
```

Terminal 2:
```{.bash .shell-prompt}
Enter second command here
```

*Enter GIF to show robot behavior*
<p align="center">
  <img height=600 src="https://raw.githubusercontent.com/hello-robot/stretch_tutorials/ROS2/images/avoider.gif"/>
</p>

*Enter GIF to show robot and sensor visualization in RViz*
<p align="center">
  <img height=600 src="https://raw.githubusercontent.com/hello-robot/stretch_tutorials/ROS2/images/scanfilter.gif"/>
</p>

## Code Breakdown
Now, let's jump into the code to see how things work under the hood. Follow along [*link to code*]() to have a look at the entire script.

We make use of two separate Python classes for this demo. The FrameListener class is derived from the Node class and is the place where we compute the TF transformations. For an explantion of this class, you can refer to the [TF listener](https://docs.hello-robot.com/0.2/stretch-tutorials/ros2/example_10/) tutorial.
```python
class FrameListener(Node):
```

The AlignToAruco class is where we command Stretch to the pose goal. This class is derived from the FrameListener class so that they can both share the node instance.
```python
class AlignToAruco(FrameListener):
```

The constructor initializes the Joint trajectory action client. It also initializes the attribute called offset that determines the end distance between the marker and the robot.
```python
    def __init__(self, node, offset=0.75):
        self.trans_base = TransformStamped()
        self.trans_camera = TransformStamped()
        self.joint_state = JointState()
        self.offset = offset
        self.node = node

        self.trajectory_client = ActionClient(self.node, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory')
        server_reached = self.trajectory_client.wait_for_server(timeout_sec=60.0)
        if not server_reached:
            self.node.get_logger().error('Unable to connect to arm action server. Timeout exceeded.')
            sys.exit()
```

The apply_to_image() method passes the stream of RGB images from the realsense camera to the YOLOv5s model and returns detections in the form of a dictionary consisting of class_id, label, confidence and bouding box coordinates. The last part is exactly what we need for further computations.
```python
    def apply_to_image(self, rgb_image, draw_output=False):
        results = self.model(rgb_image)
 
        ...

        if draw_output:
            output_image = rgb_image.copy()
            for detection_dict in results:
                self.draw_detection(output_image, detection_dict)

        return results, output_image
```

*Motivate users to play with the code or continue exploring more topics*. Now go ahead and experiment with a few more pretrained models using PyTorch or OpenVINO on Stretch. If you are feeling extra motivated, try creating your own neural networks and training them. Stretch is ready to deploy them!