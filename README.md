# AutomaticSceneGeneration Plugin for Unreal Engine 4

## Description
The purpose of this plugin is to support the development, validation, and testing of off-road autonomous vehicles (AVs). 

There exist numerous simulator packages designed for the development and testing of autonomous vehicles in urban environments, most notably with the [CARLA](https://carla.org/) simulator, but there are very few tools, if any, designed for testing AVs in natural, unstructured off-road environments. 

Evaluating the robustness of an autonomy stack efficiently requires the ability to test a vehicle in as many diverse environments as possible. With this in mind, we developed a plugin for UE4 that allows an external client to test a virtual vehicle in any scene (suppported by our custom scene description protocol) across as many UE4 editors as one has the computational resources for. We also provide an external ROS2 interface that you can use to interact with this plugin.

MOVE: The client speciifes the scene description and the vehicle task, and this plugin will create the desired scene, and then it will monitor the vehicle's performance as it performs the task. When the vehicle succeeds or fails, the plugin will end the simulation, send the vehicle's trajectory information to the client for further processing, and wait for another request.

## The AutoSceneGen Ecosystem and Dependencies

This plugin is designed to be used with ROS. As such, we provide all the needed libraries to communicate with this plugin.

### Simulation Computer (Windows or Ubuntu)
1. AutomaticSceneGeneration Plugin for UE4 (this repo)
2. [ROSIntegration](https://github.com/tsender/ROSIntegration): A plugin for UE4 that enables ROS communication. You will need to use my fork of this repo as I have made numerous additions. One day I will submit those changes in a PR to the original repo.

### Client Computer (Ubuntu)
1. auto_scene_gen_ros2: A ROS2 interface that provides the necessary tools to interact with this plugin
2. [ros1_bridge](https://github.com/ros2/ros1_bridge): Since the ROSIntegration plugin does not yet support ROS2, we will need this repo to convert ROS1 <--> ROS2 messages
3. [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite): Required by the ROSIntegration plugin
4. auto_scene_gen_ros1: Provides our ROS1 msg/srv definitions and addresses a weird phenomena (or bug?) between the `rosbridge` and `ros1_bridge` that prevents a ROS2 client from sending a service request to a ROS1 server inside UE4.
