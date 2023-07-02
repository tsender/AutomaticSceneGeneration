# AutomaticSceneGeneration Plugin for Unreal Engine 4

Table of Contents
- [Description](#description)
- [The AutoSceneGen Ecosystem and Dependencies](#the-autoscenegen-ecosystem-and-dependencies)

## Description
The purpose of this plugin is to support the development, validation, and testing of off-road autonomous vehicles (AVs). One of the primary knowledge gaps pertaining to off-road AV evaluation and testing is in the ability to quickly and automatically test an AV system in *arbitrary* and *diverse* simulated environments. While there exist numerous simulator packages designed for the development and testing of AVs in urban environments, most notably the [CARLA](https://carla.org/) simulator, there are very few high-fidelity open-source tools designed for testing AVs in natural, unstructured off-road environments. Consequently, we are developing a plugin for UE4 specifically for this purpose. Our plugin allows an external ROS2 client to test a virtual vehicle in any scenario (suppported by our custom scenario description protocol) across as many UE4 editors as your computational resources can support. We also provide an external ROS2 interface to interact with this plugin.

Moving forward, we will often refer to this plugin by the abbreviated name "AutoSceneGen".

### A Note from the Author 
My name is Ted Sender, I am a PhD candidate at the University of Michigan, and I am developing this platform as part of my PhD research project. Hence this platform will be under active development until I graduate. 

I am aware that some features you may wish to have (e.g. a larger sensor suite, a weather system, controlling the 3D terrain elevation, etc.) have not yet been added to this platform. Some improvements are a work in progress and/or will be added in the near future as I continue my research, some might get added in my free time, and others may just be a stretch goal. My goal is that together, we can improve the capabilities of this platform and the state-of-the-art in testing off-road AVs.

## The AutoSceneGen Ecosystem and Dependencies

The entire ecosystem consists of a few plugins for Unreal Engine and a few ROS packages.

**Supported Systems:**
- Unreal Engine 4.23-4.27
  - This repo was written and tested using UE4.26 on Windows 10, but it should work on the listed versions.
- ROS2 Foxy+
  - This repo was written and tested with ROS2 Foxy on Ubuntu 20.04, but it should work on Foxy and up.

**Software Libraries:**
1. AutomaticSceneGeneration Plugin for UE4 (this repo)
2. [ROSIntegration](https://github.com/tsender/ROSIntegration/tree/feature/specify_ros_version): A plugin for UE4 that enables ROS communication. You will need to use the `feature/specify_ros_version` branch of @tsender's fork.
3. [rosbridge_suite](https://github.com/tsender/rosbridge_suite/tree/main): Required by the ROSIntegration plugin. Use the `main` branch on @tsender's fork because the authors of `rosbridge_suite` have not yet accepted accepted the PR https://github.com/RobotWebTools/rosbridge_suite/pull/824 (please feel free to contribute to the PR in any way).
4. auto_scene_gen_ros2: A ROS2 interface that provides the necessary tools to interact with this plugin.
   - Since this repo is the minimalistic ROS2 interface, it is often more convenient to add the ament packages in the `auto_scene_gen_ros2` repo to the repo you are developing.

## Installation

**Installing the UE4 Plugins**
1. Install a supported version of Unreal Engine 4 and create a code project (let's refer to this project as "MyProject").
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

![text](Documentation/AutoSceneGen_Workflow.PNG)

The diagram above shows the typical workflow for interacting with the platform. There are three main components: Unreal Engine (which includes this plugin), the autonomy stack under test, and an external client. The platform is configured to work following a server-client model, with the AutoSceneGenWorker as the server and the AutoSceneGenClient as the client. The general process is as follows:
1. The AutoSceneGenClient sends a `RunScenario` request to the AutoSceneGenWorker describing all of the scenario's parameters (scene description and mavigation task).
2. The AutoSceneGenWorker processes the `RunScenario` request, creates the desired scene, places the vehicle at the desired starting location and orientation, and relays the goal location to the AutoSceneGenVehicleNodes.
3. The autonomy stack, consisting of AutoSceneGenVehicleNodes, then process exchanges sensor data and control commands to control the vehicle to reach the goal location.
4. Once the AutoSceneGenVehicle receives its first control command, the AutoSceneGenWorker continuously monitors the vehicle's performance. If the vehicle succeeds or fails (see below for how we define failure), then the worker will reset the vehicle and send a `AnalyzeScenario` request to the client containing information about the vehicle's trajectory and performance.
5. The client will process the `AnalyzeScenario` request and when ready, it will create and submit a new `RunScenario` request describing the next navigation task to test. This process repeats until the client is done running tests.

### The AutoSceneGenWorker Actor

This is the main actor controlling everything within the simulation and is the server that the AutoSceneGen client node communicates with. This actor is responsible for processing the RunScenario requests, creating the specified scene, and monitoring the navigation task.

Every level must have one of these actors in the World Outliner to take advantage of the features provided by this plugin. While most of the settings in the details panel under the category "AutoSceneGenVehicle" are overwritten from the RunScenario request, there are a few that should be modified prior to pressing Play:
* `Worker ID`: The ID associated with this worker. Used in communication with the AutoSceneGenClient. All ROS topics related to this plugin will have the prefix `/asg_workerX/` where "X" is the worker ID.
* `Landscape Material`: The material/texture that will be applied to the AutoSceneGenLandscape actor.
* `Vehicle Start Location`: The starting location for the vehicle. Make sure this location is within the landscape bounds and is just above the landscape (a Z value of 50 cm should be fine). This ensures the vehicle starts on the default landscape.
* `Auto Scene Gen Client Name`: The name of the ROS AutoSceneGenClient node.

### The AutoSceneGenVehicle Actor

This is the base vehicle actor class. This class comes with a custom `PIDDriveByWireComponent` to allow you to control it externally via ROS, and you can attach any number of our provided sensors to the vehicle. To create your vehicle model, follow these steps:
1. Make a Blueprint class that inherits from `AutoSceneGenVehicle`.
2. Click on the MeshComponent.
   - Under "Animation", provide the `Animation Mode` and `Anim Class`.
   - Under "Mesh", provide the `Skeletal Mesh`.
   - Under "Materials", make sure the skeletal mesh materials appear.
3. Go the physics asset for the skeletal mesh. Click on each component in the Skeleton Tree and under "Collision", check the box "Simulation Generates Hit Events".
4. Go to the Class Defaults for the BP actor
   - Adjust the `Linear Motion Threshold` value as desired. This value is used to determine if the vehicle is stuck or idling.
   - Set the vehicle name as desired. All ROS topics pertaining to the vehicle will have the prefix `/asg_workerX/vehicle/`. If no worker is present, then the prefix will just be `/vehicle/`.
6. Click on the `DriveByWireComponent`
   - Under "PID Drive By Wire", make sure `Manual Drive` is unchecked, and set the Kp and Kd values for the throttle PID controller.
7. dd

### StructuralSceneActor
