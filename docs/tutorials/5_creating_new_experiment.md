# Create New Experiment
In this tutorial, you will learn about the contents of the parameter files that are used in the Intprim ROS framework. After completion, you should be familiar with the functionality/responsibility of each parameter.

## **1.1 Setting up an Experiment**
In order to create an experiment, it is useful to know how the Intprim ROS framework is structured. The launchfile (which starts up Intprim Service) begins by loading the user's parameter files to the ROS server. Let's start by examining the contents of the launchfile:

```xml
<launch>
    <rosparam command="load" file="$(find intprim_framework_ros)/param/interaction.yaml" />
    <rosparam command="load" file="$(find intprim_framework_ros)/param/experiments.yaml" />
    <rosparam command="load" file="$(find intprim_framework_ros)/param/intprim_param.yaml" />

    <include file="$(find irl_robot_drivers)/launch/coppelia_controller.launch" />

    <node name="intprim_service_node" pkg="intprim_framework_ros" type="intprim_service.py" output="screen" respawn="true" />

    <node name="interaction_application_node" pkg="intprim_framework_ros" type="interaction_application" output="screen" required="true" />
</launch>

```

As you can see, there are three parameter (.yaml) files that are loaded first. These files define metadata, robots, devices, and tunable parameters. The diagram below represents the execution flow after running `roslaunch intprim_framework_ros interaction_application.launch`. Note the controller node for the CoppeliaSim robots is launched for running the simulated experiments:

<img src="media/roslaunch.png" width="1000" />

### 1.1.1 Configuring experiments.yaml
The experiments.yaml file contains a list of experiments that can be executed with Intprim. As you can see below, there is only one experiment in this file; however, if you have multiple experiments, you can maintain a list of them here.

```yaml
-   id: 0
    name: "CoppeliaSim"
    timeout: 60 # In seconds
    sub_actions:
    -   prefix: "coppelia"
        message: "UR5's will touch a random point in between them..."
    devices:
    -   "ur5l" # lead
    -   "ur5c" # control
    controller: "ur5c"
```

| experiments.yaml 	| Description 	|
|:------------------|:--------	|
| id               	| Experiment id number. Allows users to define multiple experiments which will appear on the Interactive Application CLI. |
| name             	| Name as appears on the Interactive Application CLI. |
| timeout          	| Maximum time for the interaction to take place.	|
| [**sub_actions**](#sub_actions) |        	|
| devices          	| Names of the devices that will be used during the interaction. These device names specify which drivers to use for controlling the robot during an interaction. |
| controller       	| The device driver to use for controlling the robot. |
| error            	| "none" |


<a name="sub_actions"></a>

| sub_actions | Description |
|:------------|:------------|
| prefix      | This term is used to identify csv files (with the corresponding the prefix) when using them within the framework. |
| message     | What message would you like to output when starting this sub action? |


### 1.1.2 Configuring intprim_param.yaml
The intprim_param.yaml file is loaded as the "bip" param for ROS. This parameter file stores all of the information related to the degrees of freedom being captured during the experiment and the filter being used.

```yaml
bip:
-   id: 0
    name: "CoppeliaSim"
    modalities:
    -   name: "ur5l"
        indices: [0, 6]
        dof_names: [
            "UR5l Pos 1",
            "UR5l Pos 2",
            "UR5l Pos 3",
            "UR5l Pos 4",
            "UR5l Pos 5",
            "UR5l Pos 6"
        ]
        basis_model:
            type: "Gaussian"
            degree: 7
            scale: 0.07
            start_phase: 0.0
            end_phase: 1.0
        scaling_groups : [
            0,
            0,
            0,
            0,
            0,
            0
        ]
        noise_bias: 0.0 # Added to the measurement noise for each DoF of this modality.
        generate: false
        active: true
        active_from: 0.0 # Each DoF will only be active when the estimated phase is between from/until. When inactive, it will not be perturbed by noise and the measurement noise will be increased.
        active_until: 1.01
    #--------------------------------------------------------------------------
    -   name: "ur5c"
        indices: [6, 12]
        dof_names: [
            "UR5c Pos 1",
            "UR5c Pos 2",
            "UR5c Pos 3",
            "UR5c Pos 4",
            "UR5c Pos 5",
            "UR5c Pos 6"
        ]
        basis_model:
            type: "Gaussian"
            degree: 7
            scale: 0.07
            start_phase: 0.0
            end_phase: 1.0
        scaling_groups : [
            0,
            0,
            0,
            0,
            0,
            0
        ]
        noise_bias: 0.0 # Added to the measurement noise for each DoF of this modality.
        generate: true
        active: true
        active_from: 0.0 # Each DoF will only be active when the estimated phase is between from/until. When inactive, it will not be perturbed by noise and the measurement noise will be increased.
        active_until: 1.01
    #--------------------------------------------------------------------------
    filter:
        name: "enkf"
        ensemble_size: 100 # Max number of demonstrations!
        initial_phase: 0.0
        initial_phase_variance: 0.01
        initial_phase_velocity: 0.003028400493295413
        initial_phase_velocity_variance: 2.9307595907134784e-07
        initial_phase_acceleration: 0.0001
        initial_phase_acceleration_variance: 4.012610510198e-10
        process_variance: 1e-7
        time_delta: 1.0
        measurement_noise_bias: 10000.0
        system_order: 1
    prior:
        init_with_demonstrations: true # If this is true, the ensemble_size is upper bounded by the number of demonstrations available.
        reg_covar: 1e-6
        num_components: 1
    num_samples: 40
    phase_lookahead: 0.00
    scale_observations: false # Perform basis standardization
    cyclical: false # If true, the phase will roll back to 0 once it is >= 1.
    debug: false
    import_data: "<path_to_trained_model_dir>/trained_bip.bip"
    observation_noise: "<path_to_observation_noise_dir>/observation_noise.noise"
    mip_test_directory: "<path_to_recorded_rosbags_dir>"
    debug_directory: "<path_to_debug_dir>"
    config_name: "Config2"
    primary: true
```

| intprim_param.yaml | Description |
|:----------|:------------|
| id | Identifier of a specific Intprim model. This allows definitions for multiple models. |
| name | Title of the interaction. |
| modalities | A list of objects containing the following information. |
| [**filter**](#filter) | Information related to the type of filter used for inference. Set the Kalman filter parameters here. |
| [**prior**](#prior) |  Settings for the prior |
| num_samples | How many samples was the model trained on? |
| phase_lookahead | Denotes how many steps we would like to look ahead in the phase.  |
| scale_observations | This indicates if we perform basis standardization or not. |
| cyclical | Is the interaction cyclical? I.e, are we expecting the phase to start back at zero after we are at 100% interaction? |
| debug | This indicates whether the inference statistics and filtering predictions should be published and exported during runtime. If true, you can interact with the visual tool provided to debug predictions and infer why actions were chosen. |
| import_data | Where is the exported bip model located? |
| observation_noise | Where is the observation_noise associated with the model located (full path)? |
| mip_test_directory | When you train samples using the Intprim GUI, this is the default location where the application will look. |
| debug_directory | Path to debug directory, which can be used for the debug parameter above. |
| config_name | Name of this configuration. |
| primary | Dictates which model gets used by Intprim when the service is launched. |

<a name="filter"></a>
| filter                              	| Description	|
|:-------------------------------------	|:---	|
| name                                	| Name of filter to be used (ekf (Extended Kalman Filter), enkf (Ensemble Kalman Filter). |
| ensemble_size                       	| Maximum number of demonstrations. |
| initial_phase                       	| Indicates the percentage (start) of the phase. |
| initial_phase_variance              	| What is the variance associated with the initial phase estimate? Ex: 0.01 |
| initial_phase_velocity              	| What is the initial phase velocity for the filter? |
| initial_phase_velocity_variance     	| What is the variance associated with the initial phase velocity? |
| initial_phase_acceleration          	| What is the acceleration of the initial phase? (If we wish to model with second order properties)	|
| initial_phase_acceleration_variance 	| What is the variance associated with the initial phase acceleration? (If we wish to model with second order properties) |
| process_variance                    	| Process variance of the corresponding filter. |
| time_delta                          	| Indicates how much time to move forward (scale by) in the interaction after one iteration/observation. |
| measurement_noise_bias              	| States how much to scale the measurement noise by. |
| system_order                        	| Order/Degree of the system (1 if we exclude acceleration). |

<a name="prior"></a>
| prior                    	| Description  	|
|:--------------------------|:---	|
| init_with_demonstrations 	| Should we initialize the prior based on the demonstration data? |
| reg_covar                	| *Not currently used* Do we add regularization? If so, this is the hyperparameter for it. 0 for no regularization. |
| num_components           	| *Not currently used* Number of principal components used in the analysis. Value should be between 1 and **min**(# DoFs, # Samples) |

### 1.1.3 Configuring interaction.yaml
The interaction.yaml file contains many tunable parameters related to the devices of the current experiment running, such as PID control and maximum velocity.

```yaml
interaction:
    observation_frequency: 40.0
    max_observation_length: 800
    min_observation_length: 1
    response_frequency: 30.0
    single_point_trajectory: # Whether to only generate a trajectory consisting of a single point at the given phase.
        use: false
        phase: "current" # This can either be a floating point value indicating a specific phase to generate at, or "current" to use the currently estimated phase.
    start_generation_phase: 0.07 # Start generating trajectories if estimated phase exceeds this value.
    stop_generation_phase: 1.1 # Stop generating trajectories if estimated phase exceeds this value.
    playback_factor: 1.0
    default_record_dir: "record_dir"
    default_playback_dir: "playback_dir"
control:
    control_topic: "/continuous_controller"
    ur5:
        p_gain: 1.5
        i_gain: 0.0
        d_gain: 0.0
        max_i: 0.5
        min_i: -0.5
        max_velocity: 7.0
        max_acceleration: 7.0
        joint_distance_threshold: 0.13
        control_time_buffer: 1.0
        control_frequency: 30
```

| interaction.yaml params 	| Description 	|
|:-------------------------	|:-------------	|
| [**interaction**](#interaction) |  Describes the interaction |
| [**control**](#control) |  Describes the control |

<a name="interaction"></a>
| interaction             	| Description  	|
|:-------------------------	|:---	|
| observation_frequency   	| This specifies the output frequency to the csv file when converting from a rosbag to csv.	|
| max_observation_length  	| Maximum number of frames that are present in the csv file. |
| min_observation_length  	| Minimum number of frames that are present in the csv file. |
| response_frequency      	| At what frequency (Hz) should Intprim (ideally) output commands to control the robot? This may be limited by hardware capabilities, network speeds, and various factors that affect performance. |
| [**single_point_trajectory**](#spt)	| Whether to only generate a trajectory consisting of a single point at the given phase.|
| start_generation_phase  	| Start generating trajectories if estimated phase exceeds this value. |
| stop_generation_phase   	| Stop generating trajectories if estimated phase exceeds this value.	|
| playback_factor         	| At which factor should the demonstration be played back at? (Recommended value of 1) |
| default_record_dir      	|  Where the rosbags will be saved when recording demonstrations using the CLI. |
| default_playback_dir    	|  When we "Test from rosbag" using the CLI, where should the application look? (Can be same as record directory)	|

<a name="control"></a>
| control       	| Description  	|
|:---------------	|:---	|
| control_topic 	| topic that control commands should be published to|
| [**robot_name**](#robot_name) |   	|

<a name="robot_name"></a>
| robot_name               	| Description  	|
|:--------------------------|:--------------|
| p_gain                   	| Proportional gain |
| i_gain                   	| Interval gain  |
| d_gain                   	| Derivative gain  |
| max_i                    	| Maximum value for i term |
| min_i                    	| Minimum value for i term |
| max_velocity             	| Maximum velocity for robot |
| max_acceleration         	| Maximum acceleration for robot |
| control_frequency        	| At what frequency (Hz) will the robot be controlled at? |
| joint_distance_threshold 	| Threshold that the robot interface will check against when publishing	current trajectory. |

<a name="spt"></a>
| single_point_trajectory | Description  |
|:---------------|:---	|
| use 	| Whether or not to use the feature parameterized by the value below. |
| phase | This can either be a floating point value indicating a specific phase to generate at, or "current" to use the currently estimated phase. |


## Summary
By creating three parameter files (shown above), you are able to run an experiment and perform inference with custom settings. Placing the files into the same launch file is necessary and can be done as found in the interaction_application.launch file. More information about the implementation of these parameters can be found by looking in the source code and searching for the name of the corresponding parameter.
