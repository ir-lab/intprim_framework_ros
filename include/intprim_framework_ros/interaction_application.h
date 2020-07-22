/* @author Joseph Campbell <jacampb1@asu.edu>, Interactive Robotics Lab, Arizona State University */
#pragma once

/*!
    @addtogroup interaction_core
    @{
*/

#include <atomic>
#include <experimental/filesystem>
#include <string>
#include <utility>
#include <vector>

#include "devices/device_interface.h"
#include "devices/robot_interface.h"
#include "devices/predictor_interface.h"

#include "interaction_core.h"

#include "ros/ros.h"

#include <std_msgs/Int32.h>


/*!
    The InteractionApplication class provides a simple CLI application for running experiments.
    This is the entry point for performing experiments, e.g., collecting training demonstrations and performing test interactions.
    This application is responsible for constructing the relevant DeviceInterfaces and starting/stopping interactions through InteractionCore.
*/
class InteractionApplication
{
public:
    /*!
        Initializes the InteractionApplication instance.

        @param handle The ROS node handle.
    */
    InteractionApplication(ros::NodeHandle* handle);

    /*!
        Entry point for the CLI application.
    */
    void run();
private:

    /*!
        Enumeration class for the DeviceInterface types which are available to use.
        In order to create new devices, they must be added to this list (and the mapping function create_devices()).
        See the tutorial on creating a new device for further details (LINK HERE).
    */
    enum class DeviceInterfaceTypes
    {
        none,
        lbr4,
        ur5,
        ur5l,
        ur5c
    };

    enum end_demo_signal
    {
        user_quit,
        demo_ended_okay,
        timed_out
    };

    struct SubAction
    {
        std::string prefix;
        std::string message;
    };

    /*!
        An experiment. One struct is created for each experiment defined in the experiments ROS parameter file.
        See the tutorial on creating a new experiment for further details (LINK HERE).
    */
    struct Experiment
    {
        int                               id;
        std::string                       name;
        int                               timeout;
        std::vector<SubAction>            sub_actions;
        std::vector<DeviceInterfaceTypes> devices;
        DeviceInterfaceTypes              controller;
        unsigned int                      controller_frequency;
    };

    const static std::vector<std::string>                 m_scenario_options;
    const static std::vector<std::vector<std::string>>    m_scenario_actions;

    ros::Subscriber                                       m_demo_status_subscriber;
    ros::Publisher                                        m_demo_status_publisher;

    ros::Subscriber                                       m_demo_reset_subscriber;
    ros::Publisher                                        m_demo_reset_publisher;

    ros::NodeHandle*                                      m_handle;
    std::vector<Experiment>                               m_experiments;
    std::string                                           m_last_demonstration_name;
    std::string                                           m_default_record_dir;
    std::string                                           m_default_playback_dir;
    unsigned int                                          m_observation_frequency;
    unsigned int                                          m_max_observation_length;
    unsigned int                                          m_min_observation_length;
    unsigned int                                          m_response_frequency;
    unsigned int                                          m_rosbag_record_delay_ms;
    double                                                m_playback_factor;
    std::atomic<bool>                                     m_demo_active;
    bool                                                  m_demo_done_reset;
    bool                                                  m_reset_after_demo;

    std::unique_ptr<DeviceInterface> create_device(DeviceInterfaceTypes interface_type);
    std::unique_ptr<RobotInterface> create_robot(DeviceInterfaceTypes interface_type);
    std::vector<std::unique_ptr<DeviceInterface>> create_devices(const Experiment& experiment);
    std::unique_ptr<PredictorInterface> create_predictor(const Experiment& experiment);
    void demo_status_callback(const std_msgs::Int32::ConstPtr& message);
    void demo_reset_callback(const std_msgs::Int32::ConstPtr& message);
    void display_main_menu();
    void display_scenario_menu(const Experiment& experiment);
    void display_scenario_submenu(const Experiment& experiment, unsigned int option_idx);
    void perform_action(const Experiment& experiment, unsigned int option_idx, unsigned int action_idx);
    void test_export_rosbag(const Experiment& experiment);
    void test_no_export(const Experiment& experiment);
    void test_no_export_multiple_automated(const Experiment& experiment);
    void test_from_rosbag(const Experiment& experiment);
    void test_from_csv(const Experiment& experiment);
    void read_experiments();
    void read_parameters();
    void set_sim_time(bool sim_time);
    void end_demo_or_test(end_demo_signal sig);
    void start_demo();
    void start_test();
    DeviceInterfaceTypes string_to_device(std::string device_name);
    void train_export_csv(const Experiment& experiment);
    void train_export_rosbag(const Experiment& experiment);
    void train_export_rosbag_multiple_automated(const Experiment& experiment);
    void train_export_csv_from_rosbag(const Experiment& experiment);

    bool check_path(std::experimental::filesystem::path file_name, std::string prefix, std::string extension);
    std::string get_path(std::string input, std::string prefix, std::string extension);
    char get_keyboard_input(unsigned int timeout = 60, bool check_demo_status = false);
    end_demo_signal wait_for_space_or_demo_stop(unsigned int timeout = 0);
    bool wait_for_space(unsigned int timeout = 0);
    bool wait_for_quit(unsigned int timeout = 0);
    bool wait_for_kill_to_finish(int wait_seconds = 6);
    void send_demo_reset();
    bool wait_for_demo_reset(int seconds);
};
/** @} */
