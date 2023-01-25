## Changing the Tool

If you wish to remove the default gripper and add a different tool, you will typically edit /stretch_description/urdf/stretch_description.xacro. Specifically, you will replace the following line in order to include the xacro for the new tool and then follow directions within stretch_ros/stretch_calibration to generate a new calibrated urdf file (stretch.urdf) that includes the new tool.

`<xacro:include filename="stretch_gripper.xacro" />`

As an example we provide the xacro `stretch_dry_erase_marker.xacro` and its dependent mesh files with stretch_ros. 

Some of the tools found in the [Stretch Body Tool Share](https://github.com/hello-robot/stretch_tool_share/) include URDF data. To integrate these tools into the URDF for your Stretch

```bash
>>$ cd ~/repos
>>$ git clone https://github.com/hello-robot/stretch_tool_share
>>$ cd stretch_tool_share/<tool name>
>>$ cp stretch_description/urdf/* ~/catkin_ws/src/stretch_ros/stretch_description/urdf/
>>$ cp stretch_description/meshes/* ~/catkin_ws/src/stretch_ros/stretch_description/meshes/
```

Next add the xacro for the particular tool to `/stretch_description/urdf/stretch_description.xacro`. Then you can generate and preview the uncalibrated URDF:

```bash
>>$ cd ~/catkin_ws/src/stretch_ros/stretch_description/urdf
>>$ cp stretch.urdf stretch.urdf.bak
>>$ rosrun stretch_calibration update_urdf_after_xacro_change.sh
```

Now visualize the new tool

```bash
>>$ roslaunch stretch_calibration simple_test_head_calibration.launch
```
