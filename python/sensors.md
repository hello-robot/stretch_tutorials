# Working with Sensors

## Realsense Cameras

Use the `pyrealsense2` library to interact with the two Intel Realsense depth cameras on Stretch 3. The library can configure camera parameters, collect imagery, calculate point clouds, and much more. Start by creating a "pipeline", which is an interface for streaming data from the camera.

```python
import pyrealsense2 as rs

cam = rs.pipeline()
cam.start()
cam.stop()
```

`start()` returns a "profile", which returns information about the Realsense camera.

```
profile = cam.start()
print(profile.get_device().get_info()) # "D435if"
```

### Configuration

We pass a "config" to `start()` to choose which camera we want to stream from, what resolution the imagery should be, what frames per second the data should arrive at, and more.

```
config = rs.config()
config.enable_device(d405_info['serial_number'])
width, height, fps = 640, 480, 15
config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
profile = cam.start(config)
```

### Frames



### API and Docs

https://github.com/IntelRealSense/librealsense/tree/master/wrappers/python#python-wrapper

https://intelrealsense.github.io/librealsense/python_docs/_generated/pyrealsense2.html



