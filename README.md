# AutomaticSceneGeneration Plugin for Unreal Engine 4

Table of Contents
- [Description](#description)
- [The AutoSceneGen Ecosystem and Dependencies](#the-autoscenegen-ecosystem-and-dependencies)

## Description
The purpose of this plugin is to support the development, validation, and testing of off-road autonomous vehicles (AVs). One of the primary knowledge gaps pertaining to off-road AV evaluation and testing is in the ability to quickly and automatically test an AV system in *arbitrary* and *diverse* simulated environments. While there exist numerous simulator packages designed for the development and testing of AVs in urban environments, most notably the [CARLA](https://carla.org/) simulator, there are very few high-fidelity open-source tools designed for testing AVs in natural, unstructured off-road environments. Consequently, we developed a plugin for UE4 that allows an external client to test a virtual vehicle in any scenario (suppported by our custom scenario description protocol) across as many UE4 editors as your computational resources can support. We also provide an external ROS2 interface to interact with this plugin.

Moving forward, we will often refer to this plugin by the abbreviated name "AutoSceneGen".

MOVE: The client speciifes the scene description and the vehicle task, and this plugin will create the desired scene, and then it will monitor the vehicle's performance as it performs the task. When the vehicle succeeds or fails, the plugin will end the simulation, send the vehicle's trajectory information to the client for further processing, and wait for another request.

## The AutoSceneGen Ecosystem and Dependencies

There are three main components that make up the workflow when using this plugin: Unreal Engine (which includes this plugin), the autonomy stack under test, and an external client to determine what scenario configurations to test. All of these components are designed to communicate with each other using ROS, and as such, we utilize existing libraries and provide any custom libraries required to make the platform work.

**Supported Systems:**
- Unreal Engine 4.23-4.27
  - This repo was written and tested using UE4.26 on Windows 10, but it should work on the listed versions.
- ROS2 Foxy+
  - This repo was wWritten and tested with ROS2 Foxy on Ubuntu 20.04, but it should work on Foxy and up.

**Required libraries:**
1. AutomaticSceneGeneration Plugin for UE4 (this repo)
2. [ROSIntegration](https://github.com/tsender/ROSIntegration/tree/feature/specify_ros_version): A plugin for UE4 that enables ROS communication. You will need to use the `feature/specify_ros_version` branch of @tsender's fork.
3. [rosbridge_suite](https://github.com/tsender/rosbridge_suite/tree/main): Required by the ROSIntegration plugin. You need to use the `main` branch on @tsender's fork because the authors of `rosbridge_suite` have not yet accepted accepted the PR https://github.com/RobotWebTools/rosbridge_suite/pull/824 (please feel free to contribute to the PR in any way).
4. auto_scene_gen_ros2: A ROS2 interface that provides the necessary tools to interact with this plugin.
   - Since this repo is the minimalistic ROS2 interface, it is often more convenient to add the ament packages in the `auto_scene_gen_ros2` repo to the repo you are developing.

## Installation

**Installing the UE4 Plugins**
1. Install supported version of Unreal Engine 4 and create a UE4 code project (let's refer to this project as "MyProject").
2. Download the `ROSIntegration` and `AutomaticSceneGeneration` plugins using the links above (making sure you download the specified branches). Copy these plugins into your `MyProject/Plugins/` folder.
3. Open up your UE4 project and let the editor build the plugins (they will only build automatically the first time you open the project).
4. Once everything builds and the project opens, verify the plugins are active by going to Edit -> Plugins. If they are for some reason inactive, then activate them and restart the editor.

**Installing the ROS2 Packages**

We will assume you are using Ubuntu for all-things ROS-related.
1. Install a supported ROS2 version OR use our provided dockerfiles in Ubuntu with all the needed dependencies. If using docker, then you can download our docker image with the tag `tsender/tensorflow:gpu-focal-foxy` (you may need to login to your docker account from the command line) or you can modify the [original docker image](https://github.com/tsender/dockerfiles/blob/main/tensorflow_foxy/Dockerfile) and build it.
2. Let's put all of our code in a common folder: `mkdir ~/auto_scene_gen`
3. Let's clone and build rosbridge_suite
   ```
   cd ~/auto_scene_gen/
   mkdir rosbridge_suite
   cd rosbridge_suite/
   git clone https://github.com/tsender/rosbridge_suite.git src
   source /opt/ros/foxy/setup.bash
   colcon build
   ```
3. Let's clone and build the `auto_scene_gen` repo. Rplace this repo with your primary development repo, so long as contains the `auto_scene_gen` packages.
   ```
   cd ~/auto_scene_gen/
   source rosbridge_suite/install/setup.bash # rosbridge_suite is our underlay
   mkdir auto_scene_gen
   cd auto_scene_gen
   git clone https://github.com/tsender/auto_scene_gen.git src
   colcon build
   source install/setup.bash # This sources the overlay
   ```
   In all new terminals you open, make sure to source the overlay: `source ~/auto_scene_gen/auto_scene_gen/install/setup.bash`

**Initial UE4 Setup**
1. Open your UE4 project.
2. In the Content Browser area, go to View Options and check "Show Plugin Content".
3. Go to Project Settings -> Maps and Modes, then set the GameInstance object to `ROSIntgrationGameInstance`. Save your settings.
4. Make sure the Output Log is visible in your display by clicking Window -> Developer Tools -> Output Log (the UE4 plugins make heavy use of logging).
5. Add a `ROSBridgeParamOverride` actor to the level. Click on this actor in the World Outline. Under the Details display, Sst the ROSVersion to 2, and adjust the `ROSBridgeServerHost` and `ROSBridgeServerPort` according to your setup. Save the level.
8. Open a terminal in your Ubuntu computer and launch the rosbridge TCP node
   ```
   ros2 launch rosbridge_server rosbridge_tcp_launch.xml bson_only_mode:=True
   ```
7. Press Play in your UE4 project, and you should see some log lines indicating that the game connected to rosbridge. You should also see a few lines printed to the rosbridge launch terminal indicating the client subscribed to a few ROS topics with a prefix `/unreal_ros/`. See the `ROSintegration` README if you have issues.

## Usage

### Typical Workflow

### Setting up an AutoSceneGenVehicle

### Setting up Structural Scene Elements
