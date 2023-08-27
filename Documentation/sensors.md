# AutoSceneGen Sensors

This page describes all of the senors provided by this plugin.

Quick Links:
- [Home Page](https://github.com/tsender/AutomaticSceneGeneration)
- Actors
- Components

We currently provide a few sensors. They can be attached to any actor (not just vehicles). All sensors are derived from the `UBaseSensor` class, which inherits from `USceneComponent`. All ROS topic names will follow the naming hierarchy `/asg_workerX/vehicle_name/sensors/sensor_name`. If an AutoSceneGen worker or vehicle is not present, then the associated sub-name in the topic will be omitted.

## Localization Sensor

This sensor provides the position and orientation of the sensor component. It will publish data using a `geometry_msgs/PoseStamped` message. The configurable parameters are under the "Localization Sensor" tab in the details panel:
- `Sensor Name`: The name of the sensor to be used in the ROS topic.
- `Frame Rate`: The frame rate in Hz that the sensor will run at.

## CompleteCameraSensor

This sensor is a multifunctional camera sensor supporting the following types of cameras:
- Color Camera (Color Cam): This is the standard RGB color camera that capture the scene's true colors.
- Depth Camera (Depth Cam): This camera provides the depth data. The raw data in the `sensor_msgs/Image` message will use the `32FC1` encoding.
- Traversability Segmentation Camera (Trav Cam): This camera provides a semantic segmentation image of traversable objects (white), non-traversable objects (black), and the sky (blue).
- Semantic Segmentation Camera (Seg Cam): This camera provides a semantic segmentation image corresponding to a user-specified color scheme. All objects can be configured to have a semantic segmentation color, and this color will used to create this image.

All cameras publish data with the `sensor_msgs/Image` message. All of the above cameras (except the depth camera) will encode the raw data using the `rgb8` encoding. The main color camera ROS topic name will be of the form `/asg_workerX/vehicle_name/sensors/camera_name/color_image`.

**CAUTION**: These semantic segmentation cameras do not use the depth stencil buffer that Unreal Engine provides; this may be more efficient but it limits the number of segmentation colors to 256. Instead, we adopt the approach taken by [UnrealCV](https://github.com/unrealcv/unrealcv) in which we manually change the mesh texture to create the new image. This added flexibility unfortunately comes with a cost, being the cost of rerendering the scene multiple times per sensor tick. Enabling these segmentation cameras will significantly slow down the game frame rate. We recommend you only use these sensors to help collect training data for DNNs and set the frame rate to be about 5 Hz.

The configurable parameters are under the "Complete Camera Sensor" tab in the details panel:
- `Color Camera Post Process Settings`: These are the camera postprocess settings that the color camera will use.
- `Image Width`: The image width, in pixels.
- `Image Height`: The image height, in pixels.
- `Field of View`: The camera's field of view in degrees
- `Sensor Name`: The name of the camera sensor to be used in the ROS topic.
- `Save Images to Disk`: Indicates if images from the color camera, trav camera, and seg camera should be saved to disk. If so, then they will be saved in the folder `/Game/TraininData/camera_name/` and will each have their own subfolder.
- `Frame Rate`: The frame rate in Hz that the sensor will run at.
- `Enable Depth Cam`: Indicates if the depth camera should be activated. ROS topic name will be of the form `/asg_workerX/vehicle_name/sensors/camera_name/depth_image`.
- `Enable Trav Cam`: Indicates if the traversability segmentation camera should be enabled. ROS topic name will be of the form `/asg_workerX/vehicle_name/sensors/camera_name/trav_image`.
- `Enable Seg Cam`: Indicates if the semantic segmentation camera should be enabled. ROS topic name will be of the form `/asg_workerX/vehicle_name/sensors/camera_name/seg_image`.
