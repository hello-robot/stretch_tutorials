PyTorch is an open source end-to-end machine learning framework that makes many pretrained production quality neural networks available for general use. In this tutorial we will use the YOLOv5s model trained on the COCO dataset.

YOLOv5 is a popular object detection model that divides the supplied image into a grid and detects objects in each cell of the grid recursively. The YOLOv5s model that we have deployed on Stretch has been pretrained on the COCO dataset which allows Stretch to detect a wide range of day to day objects. However, that’s not all, in this demo we want to go a step further and use this extremely versatile object detection model to extract useful information about the scene.

Often, it’s not enough to simply identify an object. Stretch is a mobile manipulator and its job is to manipulate objects in its environment. But before we do that, it needs information of where exactly the object is located with respect to itself so that a motion plan to reach the object can be generated. This is possible by knowing which pixels correspond to the object of interest in the image frame and then using that to extract the depth information in the camera frame. Once we have this information, it is possible to compute a transform of these points in the end effector frame for Stretch to generate a motion plan. 

For the sake of brevity, we will limit the scope of this tutorial to drawing bounding boxes around objects of interest to point to pixels in the image frame and drawing a detection plane corresponding to depth pixels in the camera frame. Go ahead and execute the following command to run the inference and visualize the detections in RViz:

```
ros2 launch stretch_deep_perception stretch_detect_objects.launch.py
```

Voila! You just executed the first deep learning model on Stretch!

That’s not it. Detecting objects is just one thing Stretch can do well, it can also detect people and their faces. We will be using Intel’s OpenVINO toolkit with OpenCV to achieve this. Like PyTorch, OpenVINO is a toolkit to optimize and deploy machine learning inference popularized by Intel that can utilize hardware acceleration dongles such as the Intel Neural Compute Stick with Intel based compute architectures. More convenient is the fact that most of the neural network models in the Open Model Zoo are accessible and configurable using the familiar OpenCV API with the opencv-python-inference-engine library extension. Fortunately, these packages come preinstalled with Stretch to make it easy for us to hit the ground running!

With that, let’s jump right into it! The cool thing about the model we are using is that it not only detects human faces, but also detects important features of the human face such as the eyes, nose and the lips. This is important in the context of precise assistive tasks such as feeding and combing the hair where we want to know the exact location of the facial features the end effector must reach. Alright! Let’s execute the following command to see what it looks like:

```
ros2 launch stretch_deep_perception stretch_detect_faces.launch.py
```