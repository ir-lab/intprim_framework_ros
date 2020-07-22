# IntPrim Framework ROS

The intprim_framework_ros package provides a ROS framework for the IntPrim library which simplifies the process of setting up and deploying human-robot interaction experiments (both real and simulated).
This framework also provides in-depth analysis tools which allow easy exploration of the trained prior as well as visualization of the internal state information at each time step of executed experiments.
This framework has been successfully deployed in real-time HRI scenarios between humans and a wide variety of robots, including: UR5, Baxter, Franka Emika, and several custom designs including robots driven by pneumatic artificial muscles (PAMs).
A full list of peer-reviewed publications that have utilized this package can be found further below.

This package provides the following features out of the box:
* Automates the capture and management of training data
* Synchronizes data from multiple input sources, accounting for differences in sampling frequency
* Automates the generation of response signals by Bayesian Interaction Primitives (via IntPrim)
* Provides an extensible mechanism for transmitting response signals to the relevant robot controllers
* Seamlessly operates over both offline (pre-recorded) and online (live) data sources
* Provides graphical utilities for training Bayesian Interaction Primitives and selecting appropriate hyper-parameters
* Provides graphical, interactive utilities for analyzing performance

As of the current release, only interfaces for the UR5 robot and Kuka LBR4 have been provided (as well as a [separate driver](https://github.com/ir-lab/irl_robot_drivers) for interfacing with Coppelia Sim), however, this ROS framework was designed to be as easy as possible to extend to accommodate other requirements and experimental setups.
Furthermore, additional device interfaces may be available upon request (refer to contact information below).

The documentation for the code may be found here: https://ir-lab.github.io/intprim_framework_ros/

## Human-Robot Demonstration
IntPrim Framework can be used for Human-Robot interaction, Robot-Robot interaction, or even Human-Human interaction. In the example of Joe lifting a box below, there are two parties involved- just as there are in the robot-robot next to it. The important thing about this framework is that the inference is agnostic to the type of partner; in other words, any combination of human/robot can be used to teach interactions!

<table>
  <tr>
    <center><td>Human-Robot Interaction example</td></center>
    <center><td>Robot-Robot Interaction example</td></center>
  </tr>
  <tr>
    <td><img src="docs/media/box_lq.gif" width="480" height="240"/></td>
    <td><img src="docs/media/test1.gif" width="480" height="240"/></td>
  </tr>
 </table>


## Overview
<p align="center">
  <img src="docs/media/overview.png?raw=true" width="500"/>
</p>

An overview of the IntPrim Framework is shown above.
It consists of five main layers: Interaction Core and Interactive Application (the entry point to running experiments); the device interface layer which communicates with any hardware drivers to send/receive information; IntPrim Service which exchanges information between Interaction Core and IntPrim; and the IntPrim layer itself which runs Bayesian Interaction Primitives.

All inter-process communication is handled via the ROS topic and message system, which also allows the framework to operate independently of the data layer.
This means it can process data in real-time from live sensor data or from recorded data that is played back via the rosbag system.
This system is used for both training and testing, which are covered in the tutorials.

## Installation

### Method 1:
We recommend [installing the framework locally](docs/tutorials/sub_tutorials/install_locally.md).

### Method 2:
COMING SOON! Alternatively, you can use the [container for Intprim ROS framework](docs/tutorials/sub_tutorials/install_container.md).


## Tutorials

1. [Introduction](docs/tutorials/1_introduction.md)
Applications of Intprim ROS framework and how it can be used for different types of interactions. These include Human-Robot, Robot-Robot, and Human-Human interactions.
2. [Minimal Example](docs/tutorials/2_minimal_example.md)
Learn how to train a model that learns a simple interaction consisting of two DOF trajectories. This notebook covers the interaction with Intprim's GUI and a step-by-step procedure for training and testing.
3. [Robot Example](docs/tutorials/3_robot_example.md)
Setup your own experiment, uitilize the Intprim ROS Framework, and train a model.
4. [Creating New Device](docs/tutorials/4_creating_new_device.md)
Add a device to the Intprim ROS framework.
5. [Creating New Experiment](docs/tutorials/5_creating_new_experiment.md)
Learn about the contents of the parameter files that are used in the Intprim ROS framework


## References

J. Campbell, A. Hitzmann, S. Stepputtis, S. Ikemoto, K. Hosoda, and H. Ben Amor. Learning Interactive Behaviors for Musculoskeletal Robots Using Bayesian Interaction Primitives. IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Macau, China, November 2019.

J. Campbell, S. Stepputtis, and H. Ben Amor. Probabilistic Multimodal Modeling for Human-Robot Interaction Tasks. Robotics: Science and Systems (RSS), Freiburg, Germany, June 2019.

J. Campbell and H. Ben Amor. Bayesian Interaction Primitives: A SLAM Approach to Human-Robot Interaction. Conference on Robot Learning (CoRL), Mountain View, California, November 2017.

## Contact
[Joseph Campbell](https://sites.google.com/asu.edu/jcampbell/), <jacampb1@asu.edu>
Arizona State University, Interactive Robotics Lab

Michael Drolet, <mdrolet@asu.edu>
Arizona State University, Interactive Robotics Lab

[Heni Ben Amor](http://henibenamor.weebly.com/), <hbenamor@asu.edu>
Arizona State University, Interactive Robotics Lab
