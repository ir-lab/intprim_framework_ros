## Running an Experiment with CoppeliaSim using IntPrim ROS Framework

This tutorial will show you how to run an IntPrim ROS Framework experiment in a simulation using CoppeliaSim. In the simulation, two robots will perform inverse kinematics to touch a point in 3D space. The point is randomly generated for an interaction, and the overall interaction consists of two robots reaching for the same point.

<p align="center">
  <img src="../media/test1.gif" width="600"/>
</p>


## 1 Getting Started
It is assumed you have either [installed the framework locally](sub_tutorials/install_locally.md) or are using the (COMING SOON) [container for IntPrim ROS framework](sub_tutorials/install_container.md).

## 1.1 Adding Device Drivers
In order to interact with CoppeliaSim, device drivers for the robots are needed (which are available in the [irl_robot_drivers](https://github.com/ir-lab/irl_robot_drivers) repository). The device driver constantly publishes the robot's current state, obtained by getting the kinematic state inside of CoppeliaSim (in this case), to the framework for inference to be performed. Likewise, the framework is also continuously publishing commands, which the device driver listens on to control the robot. Once the driver is added and implemented as a publisher and subscriber, the interfaces inside of the framework will be ready to setup. This will be discussed in more detail in the next section. Briefly, the central idea behind this pattern is to create/add a `Robot Interface` for the robots in the framework. The Robot Interface for this experiment lets IntPrim know which robot should be commanded during the experiment. The driver will be receiving commands from this Interface. Since the robots are also listed as devices in the parameter files, the framework will additionally create a `Device Interface` for each robot. The Device interface for this experiment lets IntPrim know that incoming sensor readings will be used when running the experiment. In this case, the incoming sensor readings are actually the robot's state information published by the driver.  There are two robots being used in this tutorial, which we call the UR5c and UR5l. The 'c' suffix represents the robot "controlled" by the framework (receiving inference) and the 'l' suffix represents the robot "leading" the interaction (fixed trajectory). To see the source code for these drivers, please check out [irl_robot_drivers](https://github.com/ir-lab/irl_robot_drivers). You can use for reference the [creating new device](4_creating_new_device.md) tutorial to see more details on the step-by-step guide to adding Interfaces for your driver and to understand the API your driver will be communicating with.

## 1.2 Adding Robot & Device Interfaces
The [Robot Interfaces](../../src/devices) for the two UR5's are included in the framework. We recommend going through the source code for these Interfaces and using the [creating new device](4_creating_new_device.md) tutorial as a reference, which explains more details on adding Interfaces. Overall, as discussed above, the Interfaces provide an API for the device drivers to communicate with.

## **1.3 Create Experiment** - Adjusting the parameter files
The [parameter files](../../parm) for this tutorial are also included in the framework. We recommend going through them to understand how they are implemented, but it is not necessary. To get everything working, you will need to modify the local file path parameters and indicate the primary interaction. The [create new experiment](5_creating_new_experiment.md) tutorial can be used for reference and contains definitions for all of the parameters. Inside [intprim_param.yaml](../../param/intprim_param.yaml), there are two interaction id's. This tutorial for CoppeliaSim uses the second entry (id=1; name="CoppeliaSim"); The tutorial for the handwritten trajectories uses the first entry (id=0; name="Simple Example"). You will need to change the following paths **in the second interaction (id=1)** to real existing paths:

* import_data: "\<path\>/trained_bip.bip"
* observation_noise: "\<path\>/observation_noise.noise"
* mip_test_directory: "\<path\>"
* debug_directory: "\<path\>/debug"
* primary: true

1. Change the top two paths to the desired directories on your computer (or docker container / VM envrionment paths if running virtually) and choose file names for the .noise and .bip files. These files represent the underlying "BIP model" that we will be creating.
2. The `mip_test_directory` should be the location of the converted csv files on your computer (discussed below).
3. The `debug_directory` is used for debugging IntPrim, which is beyond the scope of this tutorial. You can set that to any arbitrary directory for now.
4. **IMPORTANT:** the `primary` parameter is used to indicate which interaction parameters get loaded when the Interactive Application CLI starts up. Make sure `primary: true` is set for id=1 when running this tutorial, and `primary: false` is set for id=0 since we do not want to load the parameters for the handwritten tutorial.

## **2.1 Launching CoppeliaSim**
Every HRI scenario is unique: Research groups, roboticists, and hobbyists tend to have different lab environments. These enviornments may contain unique motion capture equipment, robots, and people of various heights/shapes. For this reason, it is challenging to repeat experiments in new environments and on different robots or subjects since the state space is different. Intprim aims to make the robot learning process easier by providing a framework for quickly learning interactions between two parties. One of the benefits of using Intprim is that only the trajectories for each DOF of the interacting members (robots/partners) are needed for an interaction to be learned. This tutorial gives all users the opportunity to use the same simulator environment so that there aren't any discrepancies across different interactions (aside from randomly generated variables).<br>

We will be collecting data by using the framework's Interactive Application CLI. When running Train -> Export data to rosbag in the CLI, the data collection process will be triggered and a start command will be sent to the simulator. When the start command is sent, the robots will move to the key frames that are randomly generated. The driver will need to be running so that inference on the live state information and joint state information can be collected.<br>

Start by running the [start_coppelia.sh](start_coppelia.sh) script:
```bash
./start_coppelia.sh
```
located in the same folder as this tutorial. If your environment variable in the script is correctly set and the script is executed in the same folder as the [world file](tutorial_world.ttt), you should see CoppeliaSim pop up and the environment will have two robots facing each other.

## **2.2 Collecting Training Data**
This step assumes that you have already followed the [local installation](sub_tutorials/install_locally.md#installing-irl_robot_drivers) instructions for adding the drivers to your catkin workspace.

### Collecting rosbags using the Interactive Application CLI
We will now begin running the experiment and will record the movements of the interaction. The trajectories will be saved in a rosbag for every demonstration and later will be converted to csv files so that we can train a model for the interaction. The general steps for collecting rosbags are outlined [here](sub_tutorials/interactive_application_cli.md#collect-rosbags-during-training), but the specific steps are repeated below for your convenience. <br>
Open a new terminal, and launch the Interactive Application CLI for this experiment:
```
roslaunch intprim_framework_ros interaction_application.launch
```
You should now see the prompt for the CLI that lists all of your experiments and asks to select a scenario.

### Option 1 - Manual:
This is the manual method for collecting rosbags. If you would like to automatically collect rosbags, go to Option 2. Repeat the following steps as many times as necessary to collect several training examples:
1. Select the correct experiment/scenario for CoppeliaSim. The following prompt will appear:
 ```
Please select a scenario category:
  [0] Train
  [1] Test
```
2. Select 0, "Train". The following prompt will appear:
```
Please select a scenario action:
  [0] Export data to rosbag
  [1] Export data to csv
  [2] Export data to csv from rosbag
  [3] Delete last rosbag
  [4] Export data to rosbag (Multiple Automated)
```
3. Select 0, "Export data to rosbag". The following prompt will appear:
```
UR5's will touch a random point in between them...
Please get ready and press [space] to begin demonstration.
```
4. Press space to begin the demonstration. <br>
At this point, the start command to begin the interaction will be sent to the simulator and the framework will start recording the interaction as a rosbag. The rosbag will be saved in the directory specified by the `default_record_dir` parameter in the `interaction.yaml` file.

### Option 2 - Automatic (recommended):
1. Select the correct experiment/scenario for CoppeliaSim. The following prompt will appear:
 ```
Please select a scenario category:
  [0] Train
  [1] Test
```
2. Select 0, "Train". The following prompt will appear:
```
Please select a scenario action:
  [0] Export data to rosbag
  [1] Export data to csv
  [2] Export data to csv from rosbag
  [3] Delete last rosbag
  [4] Export data to rosbag (Multiple Automated)
```

3. Select 4, "Export data to rosbag (Multiple Automated)". The following prompt will appear:
```
How many automated demonstrations do you want to run?
```
4. Enter the desired number of recordings you'd like to capture. You will now see:<br>
```
UR5's will touch a random point in between them...
Executing demonstration in 1 second...
```
5. The start command to begin the interaction will be sent to the simulator and the framework will start recording the interaction as a rosbag. The rosbag will be saved in the directory specified by the `default_record_dir` parameter in the `interaction.yaml` file. This will happen as many times as you specify in the previous step.

## **2.3 Converting rosbags to csv files**
1. IMPORTANT: After collecting rosbags, you need to restart the Interactive Application. Before doing so, you must do the following: Open up the [interactive_application.launch](../../launch/interaction_application.launch) file and comment out the coppelia_controller.launch line by doing the following:
```
    <!-- <include file="$(find irl_robot_drivers)/launch/coppelia_controller.launch" /> -->
```
This will prevent the CoppeliaSim controller from reading the state of the simulator and publishing it as a rostopic. When converting rosbags to csv files, the rosbags are played. If there is conflicting data coming from a topic from the simulator, then the csv data will be corrupted. For an additional safety measure, close CoppeliaSim when converting to csv.
2. Follow the instructions in the [Interactive Application CLI (convert rosbags)](sub_tutorials/interactive_application_cli.md#convert-rosbags-to-csv-files) sub_tutorial to convert the rosbags from the above training demonstrations into csv files. The `default_playback_dir` parameter indicates the location of the rosbags, or you can manually enter the path where the rosbags are located when prompted. If the default playback directory is set for the handwritten trajectories tutorial, make sure to change the parameter to reflect the new location of the CoppeliaSim rosbags.

## **3.1 Training** - Training model from csv files
1. Follow the instructions for the [Train model from csv files](sub_tutorials/interactive_application_cli.md#train-model-from-csv-files) section in the Interactive Application CLI reference, which covers the general steps for training a model from csv files. **NOTE**: The CLI for this tutorial can be launched by running `roslaunch intprim_framework_ros interaction_application.launch`. Make sure to select the experiment for `CoppeliaSim` from the list of experiments and use the csv files generated from the previous step. For details on how to pick / tune the model parameters, view the tutorial notebooks in the Intprim library repository.

## **4.1 Testing** - Testing model with Live input (Simulation)
For this tutorial, the UR5c robot is the robot performing inference. In other words, the UR5l will be leading the interaction and the UR5c must decide which trajectory to execute. When tests are executed through the Interactive Application CLI, control will be applied to the robot specified by the `controller` parameter in the `experiments.yaml` file. All other devices, such as the UR5l, will provide state/sensor data only for inference to be performed on. In this tutorial, the Interactive Application CLI will trigger the UR5 to execute an arbitrary trajectory.

1. Open up the [interactive_application.launch](../../launch/interaction_application.launch) file and uncomment the coppelia_controller.launch line by doing the following:
```
    <include file="$(find irl_robot_drivers)/launch/coppelia_controller.launch" />
```
This will re-enable the CoppeliaSim controller.

2. Follow the instructions in the [Interactive Application CLI (test model - live)](sub_tutorials/interactive_application_cli.md#test-model-with-live-input) tutorial to test the model created above!

3. Observe the inference predictions and controlled robot to see how well the model performs!

## 5.1 Expected Results
You should see the robot on the right perform actions that are similar to the actions performed in training. This robot, the controlled robot, will have learned the interaction if enough training samples are used. Typically between 50 and 200 demonstration are needed to accurately reproduce the interaction and generalize over many conditions. Feel free to use the trained model that we have provided instead, or replicate the model by using the recorded demonstrations (provided as csv files).

<p align="center">
  <img src="../media/test1.gif" width="330"/>
  <img src="../media/test2.gif" width="330"/>
  <img src="../media/test3.gif" width="330"/>
</p>
