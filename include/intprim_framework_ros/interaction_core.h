/* @author Joseph Campbell <jacampb1@asu.edu>, Interactive Robotics Lab, Arizona State University */
#pragma once

/*!
    @addtogroup interaction_core
    @{
*/

#include "devices/device_interface.h"
#include "devices/predictor_interface.h"

#include "ros/ros.h"
#include "rosgraph_msgs/Clock.h"
#include "intprim_framework_ros/Trajectory.h"

#include <atomic>
#include <mutex>
#include <thread>
#include <vector>

/*!
    The InteractionCore class is responsible for starting and stopping interactions, collecting and synchronizing data, and communicating with the IntPrim Python library in order to generate responses.

    Starting/stopping interactions
    ---------------
    An interaction can take one of two forms: either a train interaction or a test interaction.
    Train interactions consist of observations which are used to create demonstrations to serve as prior knowledge for the probabilistic model created by Bayesian Interaction Primitives.
    Test interactions consist of observations which are fed to the probabilistic model created by Bayesian Interaction Primitives in order to generate an appropriate response, e.g., a robot control policy.

    As such, starting a train interaction consists of creating a single process to collect and synchronize observations at a given frequency. This is handled by begin_demonstration().
    Once the interaction has been stopped via end_demonstration(), the observations may be exported to a CSV for further processing with export_demonstration().
    See introduction.ipynb (LINK THIS) for information on how these files may be utilized to create a BIP prior.

    A test interaction is started and ended with begin_response() and end_response() respectively.
    In contrast to training, testing starts two processes which run concurrently: a process which collects and synchronizes observations (the same as in training); and a process which takes the collected observations and passes them off to IntPrim at a given frequency.
    IntPrim will perform inference and return a predicted response.
    By default, InteractionCore will publish this response to the ROS topic defined in the rosparam control/control_topic.
    @todo Modify such that the topic can be set by the user via ROS parameter.

    Collecting and synchronizing data
    ---------------
    The collection and synchronization of data is handled by observe().
    This function spins at a rate of observation_rate Hz and every iteration collects the latest observation available from each device_interface and appends it to an internal buffer.
    In order to avoid blocking execution, this function is spun off into its own thread.
    Note that this is a private function and should *not* be called externally. It is internally triggered through begin_demonstration() and begin_response().

    Communicating with IntPrim
    ---------------
    Communication with the IntPrim library is handled by respond().
    This function spins at a rate of respond_rate Hz and sends the observations stored in the internal buffer to IntPrim to process via a ROS service.
    The response from IntPrim is published out to the ROS topic defined in the rosparam control/control_topic.
 */
class InteractionCore
{
public:
    /*!
        Initializes the InteractionCore instance.

        @param handle The ROS node handle
        @param device_interfaces A vector of device_interfaces from which observations will be collected. These are polymorphic objects, so the specific device does not matter as long as they fully implement the DeviceInterface API.
        @param predictor_interface The predictor_interface which will be utilized to communicate with IntPrim.
     */
    InteractionCore(ros::NodeHandle handle, std::vector<std::unique_ptr<DeviceInterface>> device_interfaces, std::unique_ptr<PredictorInterface> predictor_interface);

    /*!
        Begins a training interaction. Observations are collected at the frequency specified by observation_rate and stored in an internal buffer.

        @param observation_rate The rate at which observations are collected (in Hz).
        @param max_observation_length The maximum number of observations to keep in the internal buffer. Once this limit is reached, the oldest observation will be dropped when a new one is added.
        @param require_clock If true, observations will not be collected until a message is published on the /clock ROS topci.
        @param wait_for_control If true, observations will not be collected until a message is published on the control topic.
        @param check_valid If true, only valid observations (refer to DeviceInterface for more details) are collected.
    */
    void begin_demonstration(unsigned int observation_rate, unsigned int max_observation_length, bool require_clock = false, bool wait_for_control = false, bool check_valid = true);

    /*!
        Ends a training interaction. Observations will no longer be collected after this point.
    */
    void end_demonstration();

    /*!
        Exports the observations currently stored in the internal buffer to a CSV file with the name file_name.

        @param file_name The name of the exported CSV file.
    */
    void export_demonstration(std::string file_name);

    /*!
        Begins a testing interaction. Observations are collected at the frequency specified by observation_rate and stored in an internal buffer.
        The observations are removed from the buffer and sent to IntPrim at the frequency specified by response_rate.
        The response from IntPrim is published on the topic defined by the rosparam control/control_topic.

        @param observation_rate The rate at which observations are collected (in Hz).
        @param max_observation_length The maximum number of observations to keep in the internal buffer. Once this limit is reached, the oldest observation will be dropped when a new one is added.
        @param response_rate The rate at which the observations are sent to IntPrim for inference.
        @param minimum_observation_length The minimum number of observations that must be in the buffer before they are sent to IntPrim.
        @param require_clock If true, observations will not be collected until a message is published on the /clock ROS topci.
        @param wait_for_control If true, observations will not be collected until a message is published on the control topic.
        @param check_valid If true, only valid observations (refer to DeviceInterface for more details) are collected.
    */
    void begin_response(unsigned int observation_rate, unsigned int max_observation_length, unsigned int response_rate, unsigned int minimum_observation_length, bool require_clock = false, bool wait_for_control = false, bool check_valid = true);

    /*!
        Ends a testing interaction.
    */
    void end_response();

    /*!
        Takes an exported demonstration (specified by file_name) and sends it to IntPrim for evaluation in batches of minimum_observation_length.
        The generated response is compared to the ground truth from the demonstation and the MSE values are calculated.
        See BayesianInteractionPrimitiveService for further details.

        @param file_name The name of the demonstration to evaluate.
        @param minimum_observation_length The minimum number of observations that must be in the buffer before they are sent to IntPrim.

        @returns The overall MSE value of the demonstration.
    */
    float evaluate_demonstration(std::string file_name, unsigned int minimum_observation_length);


    /*!
        Exports the statistics and debugging information from the previous interaction.
        The export directory is dictated by ROS parameters.
        See BayesianInteractionPrimitiveService for further details.

        @param bag_file The name of the ground truth bag file if available, else an empty string.
    */
    void get_statistics(std::string bag_file);

private:
    ros::NodeHandle                 m_handle;

    std::vector<std::unique_ptr<DeviceInterface>> m_device_interfaces;
    std::unique_ptr<PredictorInterface>           m_predictor_interface;

    ros::Subscriber                 m_clock_listener;
    ros::Subscriber                 m_control_listener;

    ros::Publisher                  m_robot_talker;

    std::vector<float>              m_trajectory;
    std::mutex                      m_trajectory_mutex;
    std::size_t                     m_state_size;

    std::thread                     m_observation_thread;
    std::atomic<bool>               m_observe;

    std::thread                     m_response_thread;
    std::atomic<bool>               m_respond;

    std::atomic<bool>               m_clock_received;
    std::atomic<bool>               m_control_received;

    std::string                     m_observation_noise_path;
    std::string                     m_control_topic;

    void clock_callback(const rosgraph_msgs::Clock::ConstPtr& message);
    void control_callback(const intprim_framework_ros::Trajectory::ConstPtr& message);

    /*!
        Spins at observation_rate and collects observations.

        @param observation_rate The rate at which observations are collected (in Hz).
        @param max_observation_length The maximum number of observations to keep in the internal buffer. Once this limit is reached, the oldest observation will be dropped when a new one is added.
        @param require_clock If true, observations will not be collected until a message is published on the /clock ROS topci.
        @param wait_for_control If true, observations will not be collected until a message is published on the control topic.
        @param check_valid If true, only valid observations (refer to DeviceInterface for more details) are collected.
    */
    void observe(unsigned int observation_rate, unsigned int max_observation_length, bool require_clock, bool wait_for_control, bool check_valid);

    /*!
        Spins at response_rate and sends observations to IntPrim.

        @param response_rate The rate at which the observations are sent to IntPrim for inference.
        @param minimum_observation_length The minimum number of observations that must be in the buffer before they are sent to IntPrim.
    */
    void respond(unsigned int response_rate, unsigned int minimum_observation_length);

    /*!
        Publishes the reponse from IntPrim to a ROS topic.

        @param sevice_message The response message from the IntPrim service.
    */
    void send_control_command(const intprim_framework_ros::Trajectory& service_message);

    /*!
        Adds the most recent observation from each device_interface to the internal buffer.

        @param max_observation_length The maximum number of observations to keep in the internal buffer. Once this limit is reached, the oldest observation will be dropped when a new one is added.
        @param check_valid If true, only valid observations (refer to DeviceInterface for more details) are collected.
    */
    void update_trajectory(unsigned int max_observation_length, bool check_valid);
};
/** @} */
