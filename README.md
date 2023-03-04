# AutomaticSceneGeneration Plugin for Unreal Engine 4

Table of Contents
- [Description](#description)
- [The AutoSceneGen Ecosystem and Dependencies](#the-autoscenegen-ecosystem-and-dependencies)

## Description
The purpose of this plugin is to support the development, validation, and testing of off-road autonomous vehicles (AVs). One of the primary knowledge gaps pertaining to off-road AV evaluation and testing is in the ability to create and conduct *arbitrary* off-road driving scenarios in simulation. While there exist numerous simulator packages designed for the development and testing of AVs in urban environments, most notably the [CARLA](https://carla.org/) simulator, there are very few open-source tools designed for testing AVs in natural, unstructured off-road environments. 

Evaluating the robustness of an autonomy stack efficiently requires the ability to test a vehicle in as many diverse environments as possible. With this in mind, we developed a plugin for UE4 that allows an external client to test a virtual vehicle in any scene (suppported by our custom scene description protocol) across as many UE4 editors as your computational resources can support. We also provide an external ROS2 interface to interact with this plugin.

Moving forward, we will often refer to this plugin by the abbreviated name "AutoSceneGen".

MOVE: The client speciifes the scene description and the vehicle task, and this plugin will create the desired scene, and then it will monitor the vehicle's performance as it performs the task. When the vehicle succeeds or fails, the plugin will end the simulation, send the vehicle's trajectory information to the client for further processing, and wait for another request.

## The AutoSceneGen Ecosystem and Dependencies

There are three main components that make up the workflow when using this plugin: Unreal Engine (which includes this plugin), the autonomy stack under test, and an external client to determine what scenario configurations to test. All of these components are designed to communicate with each other using ROS, and as such, we utilize existing libraries and provide any custom libraries required to make the platform work.

**Supported Systems:**
- Unreal Engine 4.23-4.27 (this repo has only been written/tested in UE4.26 on Windows 10, but it should work on the listed versions)
- ROS2 Foxy (Only tested on Ubuntu systems. We also provide dockerfiles with all needed dependencies.)

**Required libraries:**
1. AutomaticSceneGeneration Plugin for UE4 (this repo)
2. [ROSIntegration](https://github.com/tsender/ROSIntegration/tree/feature/specify_ros_version): A plugin for UE4 that enables ROS communication. You will need to use my fork of this repo. Use the `feature/specify_ros_version` branch.
3. [rosbridge_suite](https://github.com/tsender/rosbridge_suite/tree/foxy): Required by the ROSIntegration plugin. You need to use my fork because the authors of `rosbridge_suite` do not want to accept one of my pull requests that add back the TCP handler. Use the `foxy` branch.
4. auto_scene_gen_ros2: A ROS2 interface that provides the necessary tools to interact with this plugin.
   - Since the above repo is the "bare-bones" repo (it's just the ROS2 interface), it is often more convenient to add the ament packages in the `auto_scene_gen_ros2` repo to the repo you are developing. In our case, we developed the [adversarial_scene_gen_ros2](https://github.com/tsender/adversarial_scene_gen_ros2) repo which contains everything in the `auto_scene_gen_ros2` plus additional code we needed to conduct our research. You can use this repo as a full example on how to interact with this plugin or you can build on top of it for your own needs.

If for some reason you choose to use the original ROSIntegration repo (which currently does not support ROS2), then you will need these additional legacy requirements:
1. [ros1_bridge](https://github.com/ros2/ros1_bridge): Since the ROSIntegration plugin does not yet support ROS2, we will need this repo to convert ROS1 <--> ROS2 messages
2. [auto_scene_gen_ros1](https://github.com/tsender/auto_scene_gen_ros1): Provides our ROS1 msg/srv definitions and addresses a weird phenomena (or bug?) between the `rosbridge` and `ros1_bridge` that prevents a ROS2 client from sending a service request to a ROS1 server inside UE4.
## Installation

**Installing the UE4 Plugins**
1. Install supported version of Unreal Engine 4 and create a UE4 code project (I will refer to this project as "MyProject").
2. Download the `ROSIntegration` and `AutomaticSceneGeneration` plugins using the links above (making sure you download the specified branches). Copy these plugins into your `MyProject/Plugins/` folder.
3. Open up your UE4 project and let the editor build the plugins (they will only build automatically the first time you open the project).
4. Once everything build and the project opens, verify the plugins are active by going to Edit -> Plugins. If they are for some reason inactive, then activate them and restart the editor.

**Installing the ROS2 Packages**

We will assume you are using Ubuntu for all-things ROS-related.
1. Install a supported ROS2 version per the ROS2 documentation OR use our provided dockerfiles in Ubuntu with all the needed dependencies. If using docker, then you can download our docker image with the tag `tsender/tensorflow:gpu-focal-foxy` (you may need to login to your docker account from the command line) or you can modify the original docker image [here](https://github.com/tsender/dockerfiles/blob/main/tensorflow_foxy/Dockerfile) and build it.
2. Let's put all of our code in a common folder: `mkdir ~/auto_scene_gen`
3. Let's clone and build rosbridge_suite
   ```
   cd ~/auto_scene_gen/
   mkdir rosbridge_suite
   cd rosbridge_suite/
   git clone https://github.com/tsender/rosbridge_suite.git src
   cd src
   git checkout foxy
   cd ..
   
   # Build rosbridge_suite from scratch
   source /opt/ros/foxy/setup.bash
   colcon build
   ```
3. Let's clone and build the `adversarial_scene_gen_ros2` repo since it contains more than just the bare interface. You may replace this repo with your primary development repo, so long as contains the `auto_scene_gen_ros2` packages.
   ```
   cd ~/auto_scene_gen/
   source rosbridge_suite/install/setup.bash # rosbridge_suite is our underlay
   mkdir adversarial_scene_gen_ros2
   cd adversarial_scene_gen_ros2
   git clone https://github.com/tsender/adversarial_scene_gen_ros2.git src
   
   # Build the packages
   colcon build
   source install/setup.bash # This sources the overlay
   ```

**Initial UE4 Setup**
1. Open your UE4 project.
2. In the Content Browser area, go to View Options and check "Show Plugin Content".
3. Go to Project Settings -> Maps and Modes, then set the GameInstance object to `ROSIntgrationGameInstance`. Save your settings.
4. Make sure the Output Log is visible in your display by clicking Window -> Developer Tools -> Output Log (the UE4 plugins make heavy use of logging).
5. Add a `ROSBridgeParamOverride` actor to the level. Click on this actor in the World Outline. Under the Details display, Sst the ROSVersion to 2, and adjust the `ROSBridgeServerHost` and `ROSBridgeServerPort` according to your setup. Save the level.
8. Open a terminal in your Ubuntu computer and laucnh the rosbridge TCP node
   ```
   source ~/auto_scene_gen/adversarial_scene_gen_ros2/install/setup.bash # Use your development repo as mentioned above
   ros2 launch rosbridge_server rosbridge_tcp_launch.xml bson_only_mode:=True
   ```
7. Press Play in your UE4 project, and you should see some log lines indicating that the game connected to rosbridge. You should also see a few lines printed to the rosbridge launch terminal indicating the client subscribed to a few ROS topics with a prefix `/unreal_ros/`. See the `ROSintegration` README if you have issues.

## Usage

### Typical Workflow

### Setting up an AutoSceneGenVehicle

### Setting up Structural Scene Elements
