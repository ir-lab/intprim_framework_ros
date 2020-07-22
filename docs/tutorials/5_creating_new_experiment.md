# Create New Experiment
In this tutorial, you will learn about the contents of the parameter files that are used in the Intprim ROS framework. After completion, you should be familiar with the functionality/responsibility of each parameter.

## **1.1 Setting up an Experiment**
In order to create an experiment, it is useful to know how the Intprim ROS framework is structured. The launchfile (which starts up Intprim Service) begins by loading the user's parameter files to the ROS server. Let's start by examining the contents of the launchfile:

```xml
<launch>
    <rosparam command="load" file="$(find intprim_framework_ros)/param/interaction.yaml"/>
    <rosparam command="load" file="$(find intprim_framework_ros)/param/experiments.yaml"/>
    <rosparam command="load" file="$(find intprim_framework_ros)/param/intprim_param.yaml"/>

    <node name="intprim_service_node" pkg="intprim_framework_ros" type="intprim_service.py" output="screen" respawn="true" />

    <node name="interaction_application_node" pkg="intprim_framework_ros" type="interaction_application" output="screen" required="true"/>
</launch>
```

As you can see, there are three parameter (.yaml) files that are loaded first. These files define metadata, robots, devices, and tunable parameters. The diagram below represents the execution flow after running `roslaunch intprim_framework_ros interaction_application.launch`:

<img src="media/roslaunch.png" width="1000" />

### 1.1.1 Configuring experiments.yaml
The experiments.yaml file contains a list of experiments that can be executed with Intprim. As you can see below, there is only one experiment in this file; however, if you have multiple experiments, you can maintain a list of them here.

```yaml
experiments:
-   id: 0
    name: "Simple Experiment"
    timeout: 10 # In seconds
    sub_actions:
    -   prefix: "simple"
        message: "Prepare to execute arbitrary trajectory..."
    devices:
    -   "ur5"
    -   "rigid_body"
    controller: "ur5" # "none"
    error: "none"
```

| experiments.yaml 	| Description 	|
|:------------------|:--------	|
| id               	| Experiment id number. This is directly linked to (and should be consistent with) the intprim_param.yaml file's id numbers (which contains more specific information about this particular experiment).      	|
| name             	| Name as appears on the intprim launch screen       	|
| timeout          	| Maximum time for the interaction to take place       	|
| [**sub_actions**](#sub_actions)      	|        	|
| devices          	| Names of the devices that will be used during the interaction. These device names specify which drivers to use for controlling the robot during an interaction.        	|
| controller       	| The device driver to use for controlling the robot.       	|
| error            	| "none"       	|


<a name="sub_actions"></a>

| sub_actions | Description |
|:------------|:------------|
| prefix      | this is the term that is prepended to the action, allowing the framework to identify it as a subaction. |
| message     | What message would you like to output when starting this sub action? |


### 1.1.2 Configuring intprim_param.yaml
The intprim_param.yaml file is loaded as the "bip" param for ROS. This parameter file stores all of the information related to the degrees of freedom being captured during the experiment and the filter being used.

```yaml
bip:
-   id: 0
    name: "SimpleExample"
    modalities:
    -   name: "ur5"
        indices: [0, 6]
        dof_names: [
            "Robot Pos 1",
            "Robot Pos 2",
            "Robot Pos 3",
            "Robot Pos 4",
            "Robot Pos 5",
            "Robot Pos 6"
        ]
        basis_model:
            type: "Polynomial"
            degree: 12
            scale: 0.04
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
    -   name: "box pos"
        indices: [6, 9]
        dof_names: [
            "Box X",
            "Box Y",
            "Box Z"
        ]
        basis_model:
            type: "Gaussian"
            degree: 9
            scale: 0.04
            start_phase: 0.0
            end_phase: 1.0
        scaling_groups: [
            1,
            1,
            1
        ]
        noise_bias: 0.0 # Added to the measurement noise for each DoF of this modality.
        generate: false
        active: true
        active_from: 0.0 # Each DoF will only be active when the estimated phase is between from/until. When inactive, it will not be perturbed by noise and the measurement noise will be increased.
        active_until: 1.01
    #--------------------------------------------------------------------------
    -   name: "box orien"
        indices: [9, 13]
        dof_names: [
            "Box Ori W",
            "Box Ori X",
            "Box Ori Y",
            "Box Ori Z"
        ]
        basis_model:
            type: "Gaussian"
            degree: 7
            scale: 0.07
            start_phase: 0.0
            end_phase: 1.0
        scaling_groups: [
            2,
            2,
            2,
            2
        ]
        noise_bias: 0.0 # Added to the measurement noise for each DoF of this modality.
        generate: false
        active: false
        active_from: 0.0 # Each DoF will only be active when the estimated phase is between from/until. When inactive, it will not be perturbed by noise and the measurement noise will be increased.
        active_until: 1.01
    #--------------------------------------------------------------------------
    filter:
        name: "enkf"
        ensemble_size: 100 # Max number of demonstrations!
        initial_phase: 0.0
        initial_phase_variance: 0.01
        initial_phase_velocity: 0.003
        initial_phase_velocity_variance: 5e-07
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
    debug: true
    import_data: "<path>/trained_bip.bip"
    observation_noise: "<path>/observation_noise.noise"
    mip_test_directory: "<path>"
    debug_directory: "<path>/debug"
    config_name: "Config1"
    primary: true
```

| intprim_param.yaml | Description |
|:----------|:------------|
| id | Identifier of a specific Intprim model. This allows definitions for mulitple models. |
| name | Title of the interaction |
| modalities | A list of objects containing the following information |
| [**filter**](#filter) | Information related to the type of filter used for inference. Set the Kalman filter parameters here. |
| [**prior**](#prior) |  Settings for the prior |
| num_samples | How many samples was the model trained on? |
| phase_lookahead |  |
| scale_observations | This indicates if we perform basis standardization or not. |
| cyclical | Is the interaction cyclical? I.e, are we expecting the phase to start back at zero after we are at 100% interaction? |
| debug | This indicates whether the inference statistics and filtering predictions should be published and exported during runtime. If true, you can interact with the visual tool provided to debug predictions and infer why actions were chosen. |
| import_data | Where is the exported bip model located? |
| observation_noise | Where is the observation_noise associated with the model located (full path)? |
| mip_test_directory | When you train samples using the Intprim GUI, this is the default location where the application will look. |
| debug_directory | path to debug directory |
| config_name | config name |
| primary | ? |

<a name="filter"></a>
| filter                              	| Description	|
|:-------------------------------------	|:---	|
| name                                	| name of filter to be used (ekf (extended kalman filter), enkf (ensemble kalman filter) |
| ensemble_size                       	| Maximum number of demonstrations.  	|
| initial_phase                       	| Indicates the percentage (start) of the phase.  	|
| initial_phase_variance              	| What is the variance associated with the initial phase estimate? Ex: 0.01 |
| initial_phase_velocity              	| Can be found from the grid search parameters function in intprim.  	|
| initial_phase_velocity_variance     	| Can be found from the grid search parameters function in intprim.  	|
| initial_phase_acceleration          	| What is the acceleration of the initial phase?  	|
| initial_phase_acceleration_variance 	| Can be found from the grid search parameters function in intprim.  	|
| process_variance                    	| tbd  	|
| time_delta                          	| Indicates how much time to move forward in the interaction after one iteration/observation.  	|
| measurement_noise_bias              	| States how much to scale the measurement noise by.  	|
| system_order                        	| Degree of the system  	|

<a name="prior"></a>
| prior                    	| Description  	|
|:--------------------------|:---	|
| init_with_demonstrations 	| Should we initialize the prior based on the demonstration data?  	|
| reg_covar                	| Do we add regularization? If so, this is the hyperparameter for it. 0 for no regularization.  	|
| num_components           	| Number of principal components used in the analysis. Value should be between 1 and **min**(number of degrees of freedom, number of samples)  	|

### 1.1.3 Configuring interaction.yaml
The interaction.yaml file contains many tunable parameters related to the device, such as PID control and maximum velocity.

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
| [**interaction**](#interaction)            	|  describes the interaction           	|
| [**control**](#control)                 	|  describes the control           	|

<a name="interaction"></a>
| interaction             	| Description  	|
|:-------------------------	|:---	|
| observation_frequency   	| What frequency (Hz) are the observations being received at? This value should be the same as the rate at which ROS topics are echoed from the robot. (or the csv time step??)	|
| max_observation_length  	| Maximum number of frames that are present in the CSV file.  	|
| min_observation_length  	| Minimum number of frames that are present in the CSV file.  	|
| response_frequency      	| At what frequency (Hz) should Intprim (ideally) output commands to control the robot? This may be limited by hardware capabilities, network speeds, and various factors that affect performance.  	|
| [**single_point_trajectory**](#spt) 	| Whether to only generate a trajectory consisting of a single point at the given phase.|
| start_generation_phase  	|   	|
| stop_generation_phase   	|   	|
| playback_factor         	|   	|
| default_record_dir      	|   	|
| default_playback_dir    	|   	|

<a name="control"></a>
| control       	| Description  	|
|:---------------	|:---	|
| control_topic 	| topic that control commands should be published to|
| [**robot_name**](#robot_name) |   	|

<a name="robot_name"></a>
| robot_name               	| Description  	|
|:--------------------------|:--------------|
| p_gain                   	| proportional gain |
| i_gain                   	| interval gain  |
| d_gain                   	| derivative gain  |
| max_i                    	| maximum value for i  |
| min_i                    	| minimum value for i  |
| max_velocity             	| maximum velocity for robot |
| max_acceleration         	| maximum acceleration for robot |
| joint_distance_threshold 	|   	|
| control_time_buffer      	|   	|
| control_frequency        	|   	|

<a name="spt"></a>
| single_point_trajectory | Description  |
|:---------------|:---	|
| use 	|   	|
| phase |   	|


## Summary
By creating three paramater files (shown above), you are able to run an experiment and perform inference with custom settings. Placing the files into the same launch file is necessary and can be done as found in the interaction_application.launch file.
