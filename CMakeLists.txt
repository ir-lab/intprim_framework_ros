cmake_minimum_required(VERSION 2.8.3)
project(intprim_framework_ros)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

# Workaround if OPTIONAL_COMPONENTS is not supported.
find_package(catkin OPTIONAL_COMPONENTS
  irl_robot_drivers
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rosgraph_msgs
  message_generation
  tf
  sensor_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(irl_robot_drivers QUIET)

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Statistics.msg
  Trajectory.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  AddDemonstration.srv
  CalculateComponents.srv
  ComputeStandardization.srv
  EvaluateTrajectory.srv
  GenerateTrajectory.srv
  GetApproximateTrajectory.srv
  GetDistribution.srv
  GetDistributionParameters.srv
  GetMeanTrajectory.srv
  GetStatistics.srv
  ExportData.srv
  InitializeState.srv
  AddBasisSelectionDemonstration.srv
  PerformBasisSelection.srv
  GetBasisError.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs rosgraph_msgs message_runtime
  DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include/intprim_framework_ros
)

## Declare a C++ executable
list(APPEND CORE_SRC
    src/interaction_application_main.cpp
    src/interaction_application.cpp
    src/interaction_core.cpp
    src/robot_controller.cpp
    src/devices/single_predictor.cpp
)

list(APPEND CONTROLLER_SRC
    src/robot_controller_main.cpp
    src/robot_controller.cpp
)

if(irl_robot_drivers_FOUND)
    list(APPEND CORE_SRC
        src/devices/lbr4.cpp
        src/devices/ur5.cpp
    )

    list(APPEND CONTROLLER_SRC
        src/devices/lbr4.cpp
        src/devices/ur5.cpp
    )
endif()

add_executable(
    interaction_application
    ${CORE_SRC}
)

add_executable(
    robot_controller
    ${CONTROLLER_SRC}
)

if(irl_robot_drivers_FOUND)
    target_compile_definitions(interaction_application PRIVATE IRL_ROBOTS_AVAILABLE)
    target_compile_definitions(robot_controller PRIVATE IRL_ROBOTS_AVAILABLE)
endif()


## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(interaction_application ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(robot_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(interaction_application
  ${catkin_LIBRARIES}
  stdc++fs
)

target_link_libraries(robot_controller
  ${catkin_LIBRARIES}
)
