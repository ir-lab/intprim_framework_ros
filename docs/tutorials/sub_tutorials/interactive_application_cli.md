## Interactive Application CLI

Features:
* [Convert rosbags to csv files](#convert-rosbags-to-csv-files)
* [Train a model from csv files](#train-model-from-csv-files)
* [Test a model from csv files](#test-model-from-csv-files)
* [Test a model with live input](#test-model-with-live-input)
* [Collect rosbags during training](#collect-rosbags-during-training)

## Convert rosbags to csv files

1. Ensure that ROS is not running and that rostopics are not being published on your system. When the rosbags get converted to csv format, they must be played; since the active topics are listened to, if there are any extra topics not involved in the interaction being published, the csv data will be different than expected. 

2. Launch the Interaction Application CLI, select the experiment, and select the "Train" option. The following prompt will appear: 
```
Please select a scenario action:
  [0] Export data to rosbag
  [1] Export data to csv
  [2] Export data to csv from rosbag
  [3] Delete last rosbag
  [4] Export data to rosbag (Multiple Automated)
```

3. Select "Export data to csv from rosbag". Note that the prefix on the file names must be the same as the experiment prefix defined in experiments.yaml (and what exporting to rosbag automatically defines). The following prompt will appear:
```
Please enter the path to the directory with the scenario rosbags or leave blank to use the default.
```

4. If the `default_playback_dir` parameter is set and correctly reflects the location of your recorded rosbags during training, then you can press space. Otherwise, enter the path to the rosbags that you'd like to convert to csv format. This method will loop through every rosbag in the path you specify, so the conversion process may take some time. After this process has completed, you will see the csv files in the same directory that the rosbags are in.


## Train model from csv files
1. If you already have a model trained (located in the directory specified in the parameter files), you must move this model to a new location. Otherwise, the new training samples will be appended to the existing model, and your results will likely not match your expectations.

2. Launch the main menu of the Interactive Application CLI for your intended experiment. For example, you should see something similar to:

```
Welcome to the MIP Interactive Application!

You may press [q] at any menu in order to cancel your selection and return to the previous menu.
You may press [d] at the main menu in order to open the dashboard.

Please select which scenario you would like to work with:
  [0] Simple Experiment
```

3. Open up the graphical dashboard to train a BIP model. To do so, press "d" on the main menu of the Interaction Application CLI:
![](../media/dashboard.png)

4. Press "Select Demonstrations", select the desired csv files for training, and then press "Open". Select all of the files that appear in the window by clicking them, and then select "Train Demonstration(s)" followed by "Export Primitive". Lastly, we must determine the appropriate observation noise, so with the files still selected press "Export Observation Noise" and select "Ok". The primitive and observation noise should now be in the directory specified by the parameter files. You are now finished training the model.
 
## Test model with live input

1. With a BIP model trained, it is relatively straightforward to test against real-time data. Launch the Interaction Application CLI, select the experiment, and this time select the "Test" option.
```
Please select a scenario category:
    [0] Train
    [1] Test
```

2. You will now see the following options:
```
Please select a scenario action:
  [0] Export data to rosbag
  [1] Do not export
  [2] Test from rosbag
  [3] Test from csv
  [4] Do not export (Multiple Automated)
```

2. From here, we can test a live demonstration by selecting either "Export data to rosbag" or "Do not export". The choice is left to the user, but this selection determines whether to save the testing trajectories for further analysis after the interaction. The rosbags may take up a lot of space, so sometimes it is preferable to select "Do not export" if you only plan to simply watch the interaction. You will now see:

```
Prepare to execute arbitrary trajectory...
Please get ready and press [space] to begin test.
```

3. If you select the manual options (0-3), Press space to begin the interaction and live inference prediction! Some implementations used in this framework are designed to trigger an arbitrary trajectory for the passive (non-predictive) robot after the space bar is pressed. However, if this trigger is not in place, you will have to execute an arbitrary trajectory on the non-predictive robot after pressing space so that it will move. Otherwise, if the send/kill demo signal is setup (like in the CoppeliaSim tutorial), then you can automatically test multiple demonstrations!

## Test model from csv files

1. With a BIP model trained, it is relatively straightforward to test against pre-recorded data. Launch the Interaction Application CLI, select the experiment, and select the "Test" option.
```
Please select a scenario category:
    [0] Train
    [1] Test
```

2. You will now see the following options:
```
Please select a scenario action:
  [0] Export data to rosbag
  [1] Do not export
  [2] Test from rosbag
  [3] Test from csv
  [4] Do not export (Multiple Automated)

```

2. From here, we can test against csv data by selecting "Test from csv". You will now see:

```
Please enter the path to the directory with the scenario CSVs or leave blank to use the default.
```

3. Enter the directory where the csv files for testing are stored, or press enter to use the default (specified by `default_playback_dir` parameter).


## Collect rosbags during training
### Option 1 - Manual:
Launch the Interactive Application CLI for your experiment. For example:
```
roslaunch intprim_framework_ros interaction_application.launch
```
You should now see the prompt for the CLI that lists all of your experiments and asks to select a scenario. Repeat the following steps as many times as necessary to collect several training examples:

1. Select the correct experiment/scenario. The following prompt will appear:
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
Prepare to execute arbitrary trajectory...
Please get ready and press [space] to begin demonstration.
```
4. Press space to begin the demonstration. <br> 
At this point, the start command to begin the interaction will be sent to the robots and the framework will start recording the interaction as a rosbag. The rosbag will be saved in the directory specified by the `default_record_dir` parameter.


### Option 2 - Automatic (recommended):
Launch the Interactive Application CLI for your experiment. For example:
```
roslaunch intprim_framework_ros interaction_application.launch
```
You should now see the prompt for the CLI that lists all of your experiments and asks to select a scenario. Repeat the following steps as many times as necessary to collect several training examples:

1. Select the correct experiment/scenario. The following prompt will appear:
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
Prepare to execute arbitrary trajectory...
Executing demonstration in 1 second...
```
5. The start command to begin the interaction will be sent and the framework will start recording the interaction as a rosbag. The rosbag will be saved in the directory specified by the `default_record_dir` parameter in the `interaction.yaml` file. This will happen as many times as you specify in the previous step.
