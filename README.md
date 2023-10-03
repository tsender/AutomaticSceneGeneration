# AutomaticSceneGeneration Plugin for Unreal Engine 4

Table of Contents
- [Description](#description)
- [The AutoSceneGen Ecosystem and Dependencies](#the-autoscenegen-ecosystem-and-dependencies)
- [Installation](#installation)
- [Overview](#overview)

## Description
The purpose of this plugin is to support the development, validation, and testing of off-road autonomous vehicles (AVs). One of the primary knowledge gaps pertaining to off-road AV evaluation and testing is in the ability to *quickly and automatically* test an AV system in *arbitrary and diverse* simulated environments. While there exist numerous simulator packages designed for the development and testing of AVs in urban environments, most notably the [CARLA](https://carla.org/) simulator, there are very few high-fidelity open-source tools designed for testing AVs in natural, unstructured off-road environments. Consequently, we are developing a plugin for Unreal Engine specifically for this purpose. Our plugin allows for an external ROS2 client to test a virtual vehicle in any scenario (suppported by our custom scenario description protocol) across as many UE4 editors as your computational resources can support. We also provide an external ROS2 interface to interact with this plugin.

*Automation* is an integral component for efficiently conducting navigation scenarios one after the other. This process includes tearing down the previous scene, creating the new one, resetting the internal state of the vehicle's processing nodes, and signalling to the vehicle processing nodes when it's okay to run. All of these functional requirements needed to perform automatic scene creation and vehicle testing are embedded into this platform, hence the name "Automatic Scene Generation".

Moving forward, we will often refer to the UE plugin by the abbreviated name "AutoSceneGen". This plugin currently only supports UE4.

### A Note from the Author 
My name is Ted Sender, I am a PhD candidate at the University of Michigan, and I am developing this platform as part of my PhD research project. This platform is still in its infancy and will be under active development as I continue my research.

I am aware that some features you may wish to have (e.g. a larger sensor suite, a weather system, controlling the 3D terrain elevation, etc.) have not yet been added to this platform. Some improvements are a work in progress and/or will be added in the near future as I continue my research, some might get added in my free time, and others may just be a stretch goal. My goal is that together, we can improve the capabilities of this platform and the state-of-the-art in testing off-road AVs.

### Citation

If you use our work in an academic context, we would greatly appreciate it if you used the following citation:

TODO

## The AutoSceneGen Ecosystem and Dependencies

The entire ecosystem consists of a few plugins for Unreal Engine and a few ROS packages.

**Supported Systems:**
- Unreal Engine 4.23-4.27
  - This repo was written and tested using UE4.26 on Windows 10, but it should work on the listed versions.
- ROS2 Foxy+
  - The ROS2 interface was written and tested with ROS2 Foxy on Ubuntu 20.04, but it should work on Foxy and up.

**Required Software Libraries:**
1. AutomaticSceneGeneration Plugin for UE4 (this repo)
2. [ROSIntegration](https://github.com/tsender/ROSIntegration/tree/feature/specify_ros_version): A plugin for UE4 that enables ROS communication. You will need to use the `feature/specify_ros_version` branch of @tsender's fork.
3. [rosbridge_suite](https://github.com/tsender/rosbridge_suite/tree/main): Required by the ROSIntegration plugin. You will need to use the `main` branch on @tsender's fork because the authors of `rosbridge_suite` have not yet accepted the PR https://github.com/RobotWebTools/rosbridge_suite/pull/824 (please feel free to contribute to the PR in any way).
4. auto_scene_gen: A ROS2 interface that provides the necessary tools to interact with this plugin.
   - Since this repo is the minimalistic ROS2 interface, it is often more convenient to add the two ament packages to the repo you are developing.

## Installation

**Installing the UE4 Plugins**
1. Install a supported version of Unreal Engine 4 and create a code project (let's refer to this project as "MyProject").
2. Download the `ROSIntegration` and `AutomaticSceneGeneration` plugins using the links above (making sure you download the specified branches). Copy these plugins into your `MyProject/Plugins/` folder.
3. Open up your UE4 project and let the editor build the plugins (they will build automatically the first time you open the project).
4. Once everything builds and the project opens, verify the plugins are active by going to Edit -> Plugins. If they are for some reason inactive, then activate them and restart the editor.

**Installing the ROS2 Packages**

1. Install a supported ROS2 version OR use our provided dockerfiles in Ubuntu with all the needed dependencies. If using docker, then you can download our docker image with the tag `tsender/tensorflow:gpu-focal-foxy` (you may need to login to your docker account from the command line) or you can modify the [original docker image](https://github.com/tsender/dockerfiles/blob/main/tensorflow_foxy/Dockerfile) and build it.
2. Let's put all of our code in a common folder: `mkdir ~/auto_scene_gen_ws`
3. Let's clone and build rosbridge_suite
   ```
   cd ~/auto_scene_gen_ws/
   mkdir rosbridge_suite
   cd rosbridge_suite/
   git clone https://github.com/tsender/rosbridge_suite.git src
   source /opt/ros/foxy/setup.bash
   colcon build
   ```
3. Let's clone and build the `auto_scene_gen` repo. Rplace this repo with your primary development repo, so long as contains the `auto_scene_gen` packages.
   ```
   cd ~/auto_scene_gen_ws/
   source rosbridge_suite/install/setup.bash # rosbridge_suite is our underlay
   mkdir auto_scene_gen
   cd auto_scene_gen
   git clone https://github.com/tsender/auto_scene_gen.git src
   colcon build
   source install/setup.bash # This sources the overlay
   ```
   In all new terminals you open, make sure to source the overlay:
   ```
   source ~/auto_scene_gen_ws/auto_scene_gen/install/setup.bash
   ```

**Initial UE4 Setup**
1. Open your UE4 project.
2. In the Content Browser area, go to View Options and check "Show Plugin Content".
3. Go to Project Settings -> Maps and Modes, then set the GameInstance object to `ROSIntgrationGameInstance`. Save your settings.
4. Make sure the Output Log is visible in your display by clicking Window -> Developer Tools -> Output Log (the UE4 plugins make heavy use of logging).
5. Add a `ROSBridgeParamOverride` actor to the level. Click on this actor in the World Outline. Under the Details display, Set the `ROSVersion` to 2, and adjust the `ROSBridgeServerHost` and `ROSBridgeServerPort` according to your setup. Save the level.
8. Open a terminal in your Ubuntu computer and launch the rosbridge TCP node
   ```
   ros2 launch rosbridge_server rosbridge_tcp_launch.xml bson_only_mode:=True
   ```
   Or feel free to create another version of this launch file if you have other needs.
7. Press Play in your UE4 project, and you should see some log lines indicating that the game connected to rosbridge. You should also see a few lines printed to the rosbridge launch terminal indicating the client subscribed to a few ROS topics with a prefix `/unreal_ros/`. See the `ROSintegration` README if you have issues.

## Overview

![text](Documentation/AutoSceneGen_Workflow.PNG)

The diagram above shows the typical workflow for interacting with the platform. There are three main components: Unreal Engine (which includes this plugin), the autonomy stack under test, and an external client. The platform is configured to work following a server-client model, with the AutoSceneGenWorker as the server and the AutoSceneGenClient as the client. The general process is as follows:
1. The AutoSceneGenClient sends a `RunScenario` request to the AutoSceneGenWorker describing all of the scenario's parameters (scene description and mavigation task).
2. The AutoSceneGenWorker processes the `RunScenario` request, creates the desired scene, places the vehicle at the desired starting pose, and relays the goal location to the AutoSceneGenVehicleNodes.
3. The autonomy stack, consisting of AutoSceneGenVehicleNodes, processes sensor data and sends control commands to the AutoSceneGenVehicle to reach the goal location.
4. Once the AutoSceneGenVehicle receives its first control command, the AutoSceneGenWorker continuously monitors the vehicle's performance. Once a termination condition is triggered, the worker will reset the vehicle and send a `AnalyzeScenario` request to the client containing information about the vehicle's trajectory and performance.
5. The AutoSceneGenClient will process the `AnalyzeScenario` request and when ready, it will create and submit a new `RunScenario` request describing the next navigation task to test. This process repeats until the client is done running tests.

This plugin provides a variety of actors, components, and sensors that you will be interacting with and will need to configure. Below are links for documentation on each of these items:
- [AutoSceneGen Actors](https://github.com/tsender/AutomaticSceneGeneration/blob/main/Documentation/actors.md)
- [AutoSceneGen Components](https://github.com/tsender/AutomaticSceneGeneration/blob/main/Documentation/components.md)
- [AutoSceneGen Sensors](https://github.com/tsender/AutomaticSceneGeneration/blob/main/Documentation/sensors.md)

Please consult the [auto_scene_gen](https://github.com/tsender/auto_scene_gen) repository for documentation on how to use the ROS2 interface.
