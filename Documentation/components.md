# AutoSceneGen Components

This page describes all of the primary actor components provided by this plugin.

Quick Links:
- [Home Page](https://github.com/tsender/AutomaticSceneGeneration)
- Actors
- Sensors


## AnnotationComponent

This component is part of every AutoSceneGen actor and keeps track of the actor's active material (the material that is currently being rendered). This component is primarily used by the CompleteCameraSensor so the camera can create semantic segmentation images. You most likely will not be interacting with this component in any way, except possibly to adjust the semantic segmentation color.

## PIDDriveByWireComponent

This component is used in every AutoSceneGenVehicle and is what the user interacts with via ROS to send control commands to the vehicle. The throttle is controlled by a PD controller and steering is controlled via the vehicle's PhysX model (unfortunately, PhysX uses a separaete mechanism for applying steering commands than it does for throttle).

### Control Modes

- **Manual**: This mode allows the user to control the vehicle directly via the WASD keys. To activate manual mode, check the `ManualDrive` flag in the "PID Drive By Wire" details tab. If manual mode is activated, then the other modes below will not function.
- **Bypass**: This mode allows the user to send pose commands for the vehicle to be placed at, bypassing the vehicle's control system. To use this mode, uncheck the `ManualDrive` flag, and begin publishing messages that the Bypass Control Sub listens to (see below).
- **PhysX**: This mode allows the user to utilize the vehicle's control system and to send throttle, steering, and handbrake commands. To use this mode, uncheck the `ManualDrive` flag, set the `KpThrottle` and `KdThrottle` values as desired, and begin publishing messages on the topic that the PhysX Control Sub listens to (see below).

The component will autodetect between the bypass and PhysX control modes based on the last message that was published on the appropriate topic. While it is technically possible to use these modes interchangeably, this is not recommended.

### ROS Objects

Lists any publishers, subscribers, clients, and/or services monitored by this actor. All instances of "vehicle_name" in the below topic names get replaced with the actual vehicle name as set in the Blueprint.

**Subscribers:**
- Bypass Control Sub:
  - Topic: `/asg_workerX/vehicle_name/control/bypass`
  - Type: `geometry_msgs/Pose`
  - Description: Subscribes to messages describing the desired pose for the vehicle
- PhysX Control Sub
  - Topic: `/asg_workerX/vehicle_name/control/physx`
  - Type: `auto_scene_gen_msgs/PhysXControl`
  - Description: Subscribes to messages indicating the most recent set of control commands to apply to the vehicle
