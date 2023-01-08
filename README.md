# AutomaticSceneGeneration Plugin for Unreal Engine 4

Table of Contents
- [Description](#description)
- [The AutoSceneGen Ecosystem and Dependencies](#the-autoscenegen-ecosystem-and-dependencies)

## Description
The purpose of this plugin is to support the development, validation, and testing of off-road autonomous vehicles (AVs). One of the primary knowledge gaps pertaining to off-road AV evaluation and testing is in the ability to create and conduct *arbitrary* off-road driving scenarios in simulation. While there exist numerous simulator packages designed for the development and testing of AVs in urban environments, most notably the [CARLA](https://carla.org/) simulator, there are very few open-source tools designed for testing AVs in natural, unstructured off-road environments. 

Evaluating the robustness of an autonomy stack efficiently requires the ability to test a vehicle in as many diverse environments as possible. With this in mind, we developed a plugin for UE4 that allows an external client to test a virtual vehicle in any scene (suppported by our custom scene description protocol) across as many UE4 editors as your computational resources can support. We also provide an external ROS2 interface to interact with this plugin.

MOVE: The client speciifes the scene description and the vehicle task, and this plugin will create the desired scene, and then it will monitor the vehicle's performance as it performs the task. When the vehicle succeeds or fails, the plugin will end the simulation, send the vehicle's trajectory information to the client for further processing, and wait for another request.

## The AutoSceneGen Ecosystem and Dependencies

There are three main components that make up the workflow when using this plugin: Unreal Engine (which includes this plugin), the autonomy stack under test, and an external client to determine what scenario configurations to test. All of these components are designed to communicate with each other using ROS, and as such, we utilize existing libraries and provide any custom libraries required to make the platform work.

Platforms currently supported:
- Unreal Engine 4.23-4.27 (this repo has only been tested in 4.26 on Windows 10, but it should work on the listed versions)
- ROS2 Foxy

### Simulation Computer (Windows or Ubuntu)
1. AutomaticSceneGeneration Plugin for UE4 (this repo)
2. [ROSIntegration](https://github.com/tsender/ROSIntegration/tree/feature/specify_ros_version): A plugin for UE4 that enables ROS communication. You will need to use my fork of this repo.

### Client Computer (Ubuntu)
1. auto_scene_gen_ros2: A ROS2 interface that provides the necessary tools to interact with this plugin.
2. [rosbridge_suite](https://github.com/tsender/rosbridge_suite/tree/foxy): Required by the ROSIntegration plugin. You need to use my fork because the authors of `rosbridge_suite` do not want to accept one of my pull requests.

### Legacy Requirements
1. [ros1_bridge](https://github.com/ros2/ros1_bridge): Since the ROSIntegration plugin does not yet support ROS2, we will need this repo to convert ROS1 <--> ROS2 messages
2. auto_scene_gen_ros1: Provides our ROS1 msg/srv definitions and addresses a weird phenomena (or bug?) between the `rosbridge` and `ros1_bridge` that prevents a ROS2 client from sending a service request to a ROS1 server inside UE4.
