## Prerequisites
#### First, The following items must be installed locally:
- [ROS](http://wiki.ros.org/ROS/Installation) (Kinetic or later)
- [Intprim Library](https://github.com/ir-lab/intprim#prerequisites)
- [CoppeliaSim](#Installing-CoppeliaSim)
 

#### Next, the following catkin packages must be installed in your <catkin_root>/src folder:
- [Intprim Framework ROS](#Installing-Intprim-Framework-ROS)
- [irl_robot_drivers](#Installing-irl_robot_drivers)<br>
After installing the catkin packages above, make sure to run `source devel/setup.bash` from <catkin_root>


## Installing CoppeliaSim
CoppeliaSim can be downloaded from https://www.coppeliarobotics.com/downloads <br>
**Important:** After downloading, you must set an environment variable to point to the installation path, since this is used during the catkin build and by the Intprim ROS framework.

Create a shell variable pointing to the root of the installation folder, e.g.:
```
export COPPELIA_SIM_PATH="<install_prefix>/CoppeliaSim_Edu_V4_0_0_Ubuntu18_04"
```
Note: you may want to add this to the bashrc so you don't have to manually run it each time.

## Installing IntPrim framework ROS
In order to install this framework simply clone it into a catkin workspace and build it:
```bash
cd <catkin_root>

git clone git@github.com:ir-lab/intprim_framework_ros.git src/intprim_framework_ros

catkin_make
```

## Installing irl_robot_drivers
irl_robot_drivers can be downloaded from https://github.com/ir-lab/irl_robot_drivers

In order to install this robot driver (for the CoppeliaSim tutorial), simply clone it into a catkin workspace and build it:
```bash
cd <catkin_root>

git clone git@github.com:ir-lab/irl_robot_drivers.git src/irl_robot_drivers

catkin_make
```
