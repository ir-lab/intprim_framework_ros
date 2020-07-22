/* @author Joseph Campbell <jacampb1@asu.edu>, Interactive Robotics Lab, Arizona State University */
#include "interaction_application.h"
#include "devices/single_predictor.h"
#include "robot_controller.h"

#ifdef IRL_ROBOTS_AVAILABLE
    #include "devices/lbr4.h"
    #include "devices/ur5.h"
#endif

#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <limits.h>
#include <set>
#include <sstream>
#include <stdexcept>
#include <termios.h>
#include <thread>
#include <unistd.h>

#include <intprim_framework_ros/Trajectory.h>

const std::vector<std::string> InteractionApplication::m_scenario_options = {
    "Train",
    "Test"
};

const std::vector<std::vector<std::string>> InteractionApplication::m_scenario_actions = {
    {
        "Export data to rosbag",
        "Export data to csv",
        "Export data to csv from rosbag",
        "Delete last rosbag",
        "Export data to rosbag (Multiple Automated)"
    },
    {
        "Export data to rosbag",
        "Do not export",
        "Test from rosbag",
        "Test from csv",
        "Do not export (Multiple Automated)"
    }
};

InteractionApplication::InteractionApplication(ros::NodeHandle* handle) :
    m_demo_status_publisher(),
    m_demo_status_subscriber(),
    m_demo_reset_publisher(),
    m_demo_reset_subscriber(),
    m_handle(handle),
    m_last_demonstration_name(),
    m_demo_active(false),
    m_reset_after_demo(false),
    m_demo_done_reset(false)
{
    read_experiments();
    read_parameters();
}

void InteractionApplication::run()
{
    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::thread main_thread = std::thread(&InteractionApplication::display_main_menu, this);

    main_thread.join();

    spinner.stop();

    std::cout << "Exiting...\n";
}

void InteractionApplication::read_experiments()
{
    m_experiments.clear();

    XmlRpc::XmlRpcValue param_list;
    if(m_handle->getParam("experiments", param_list))
    {
        for(std::size_t index = 0; index < param_list.size(); index++)
        {
            auto& experiment = param_list[index];

            Experiment new_experiment = {
                static_cast<int>(experiment["id"]),
                static_cast<std::string>(experiment["name"]),
                static_cast<int>(experiment["timeout"]),
                {},
                {},
                string_to_device(static_cast<std::string>(experiment["controller"])),
                1
            };

            for(std::size_t sub_action_index = 0; sub_action_index < experiment["sub_actions"].size(); sub_action_index++)
            {
                auto& sub_action = experiment["sub_actions"][sub_action_index];
                new_experiment.sub_actions.push_back(SubAction { static_cast<std::string>(sub_action["prefix"]), static_cast<std::string>(sub_action["message"]) });
            }

            for(std::size_t device_index = 0; device_index < experiment["devices"].size(); device_index++)
            {
                new_experiment.devices.push_back(string_to_device(static_cast<std::string>(experiment["devices"][device_index])));
            }

            XmlRpc::XmlRpcValue control_param_list;
            if(m_handle->getParam("control", control_param_list))
            {
                new_experiment.controller_frequency = static_cast<int>(control_param_list[static_cast<std::string>(experiment["controller"])]["control_frequency"]);
            }

            m_experiments.push_back(new_experiment);
        }
    }
}

void InteractionApplication::read_parameters()
{
    XmlRpc::XmlRpcValue param_list;
    if(m_handle->getParam("interaction", param_list))
    {
        m_default_record_dir = static_cast<std::string>(param_list["default_record_dir"]);
        m_default_playback_dir = static_cast<std::string>(param_list["default_playback_dir"]);
        m_playback_factor = static_cast<double>(param_list["playback_factor"]);
        m_observation_frequency = std::ceil(static_cast<double>(param_list["observation_frequency"]) * m_playback_factor);
        m_max_observation_length = static_cast<int>(param_list["max_observation_length"]);
        m_min_observation_length = static_cast<int>(param_list["min_observation_length"]);
        m_response_frequency = std::ceil(static_cast<double>(param_list["response_frequency"]) * m_playback_factor);
        m_rosbag_record_delay_ms = static_cast<int>(param_list["rosbag_record_delay_ms"]);

        std::string demo_topic_name;
        if(!m_handle->getParam("interaction/demo_status_topic", demo_topic_name))
        {
            throw std::invalid_argument("Missing \"interaction/demo_status_topic\" ROS parameter. Please specify and re-run node.");
        }

        m_demo_status_publisher = m_handle->advertise<std_msgs::Int32>(demo_topic_name, 1);
        m_demo_status_subscriber = m_handle->subscribe(demo_topic_name, 1, &InteractionApplication::demo_status_callback, this);

        bool reset_after_demo;
        if(m_handle->getParam("interaction/reset_after_demo", reset_after_demo))
        {
            if(reset_after_demo)
            {
                m_reset_after_demo = true;
                std::string reset_topic_name;
                if(!m_handle->getParam("interaction/demo_reset_topic", reset_topic_name))
                {
                    throw std::invalid_argument("Missing \"interaction/demo_reset_topic\" ROS parameter. Please specify and re-run node.");
                }

                m_demo_reset_publisher = m_handle->advertise<std_msgs::Int32>(reset_topic_name, 1);
                m_demo_reset_subscriber = m_handle->subscribe(reset_topic_name, 1, &InteractionApplication::demo_reset_callback, this);
            }
        }
        else
        {
            throw std::invalid_argument("Missing \"interaction/reset_after_demo\" ROS parameter. Please specify and re-run node.");
        }

    }
    else
    {
        throw std::invalid_argument("Missing \"interaction\" ROS parameters. Please set them and re-run.");
    }
}

InteractionApplication::DeviceInterfaceTypes InteractionApplication::string_to_device(std::string device_name)
{
    InteractionApplication::DeviceInterfaceTypes new_device = InteractionApplication::DeviceInterfaceTypes::none;

    if(device_name == "ur5")
    {
        new_device = InteractionApplication::DeviceInterfaceTypes::ur5;
    }
    else if(device_name == "ur5l")
    {
        new_device = InteractionApplication::DeviceInterfaceTypes::ur5l;
    }
    else if(device_name == "ur5c")
    {
        new_device = InteractionApplication::DeviceInterfaceTypes::ur5c;
    }
    else if(device_name == "lbr4")
    {
        new_device = InteractionApplication::DeviceInterfaceTypes::lbr4;
    }

    return new_device;
}

std::unique_ptr<DeviceInterface> InteractionApplication::create_device(DeviceInterfaceTypes interface_type)
{
    std::unique_ptr<DeviceInterface> device_interface;

    switch(interface_type)
    {
        case InteractionApplication::DeviceInterfaceTypes::ur5:
            device_interface = create_robot(interface_type);
            break;
        case InteractionApplication::DeviceInterfaceTypes::ur5l:
            device_interface = create_robot(interface_type);
            break;
        case InteractionApplication::DeviceInterfaceTypes::ur5c:
            device_interface = create_robot(interface_type);
            break;
        case InteractionApplication::DeviceInterfaceTypes::lbr4:
            device_interface = create_robot(interface_type);
            break;
    }

    return device_interface;
}

std::unique_ptr<RobotInterface> InteractionApplication::create_robot(DeviceInterfaceTypes interface_type)
{
    std::unique_ptr<RobotInterface> robot_interface;

    switch(interface_type)
    {
        case InteractionApplication::DeviceInterfaceTypes::ur5:
            #ifdef IRL_ROBOTS_AVAILABLE
                robot_interface = std::unique_ptr<UR5Interface>(new UR5Interface(*m_handle, "regular"));
            #else
                throw std::runtime_error("Experiment uses UR5Interface but IRL robots unavailable.");
            #endif
            break;
        case InteractionApplication::DeviceInterfaceTypes::ur5l:
            #ifdef IRL_ROBOTS_AVAILABLE
                robot_interface = std::unique_ptr<UR5Interface>(new UR5Interface(*m_handle, "observe_sim"));
            #else
                throw std::runtime_error("Experiment uses UR5LInterface but IRL robots unavailable.");
            #endif
            break;
        case InteractionApplication::DeviceInterfaceTypes::ur5c:
            #ifdef IRL_ROBOTS_AVAILABLE
                robot_interface = std::unique_ptr<UR5Interface>(new UR5Interface(*m_handle, "control_sim"));
            #else
                throw std::runtime_error("Experiment uses UR5CInterface but IRL robots unavailable.");
            #endif
            break;
        case InteractionApplication::DeviceInterfaceTypes::lbr4:
            #ifdef IRL_ROBOTS_AVAILABLE
                robot_interface = std::unique_ptr<LBR4Interface>(new LBR4Interface(*m_handle));
            #else
                throw std::runtime_error("Experiment uses LBR4Interface but IRL robots unavailable.");
            #endif
            break;
    }

    return robot_interface;
}

std::vector<std::unique_ptr<DeviceInterface>> InteractionApplication::create_devices(const Experiment& experiment)
{
    std::vector<std::unique_ptr<DeviceInterface>> device_interfaces;

    for(auto interface_type : experiment.devices)
    {
        device_interfaces.push_back(std::move(create_device(interface_type)));
    }

    return device_interfaces;
}

std::unique_ptr<PredictorInterface> InteractionApplication::create_predictor(const Experiment& experiment)
{
    return std::unique_ptr<PredictorInterface>(new SinglePredictorInterface(m_handle));
}

void InteractionApplication::test_export_rosbag(const Experiment& experiment)
{
    set_sim_time(false);

    // Right. Need to get control_frequency here and get it to RobotController somehow.
    // How do I want to retrieve the control_frequency?
    RobotController robot_controller(*m_handle, experiment.controller_frequency, create_robot(experiment.controller));

    for(const auto& sub_action : experiment.sub_actions)
    {

        std::cout << sub_action.message << "\n";
        std::cout << "Please get ready and press [space] to begin test.\n\n";

        if(!wait_for_space())
        {
            return;
        }

        robot_controller.start_control();

        // Purposely re-create the IC object everytime so previous states are wiped out.
        InteractionCore mip(*m_handle, create_devices(experiment), create_predictor(experiment));

        std::cout << "Executing test in 3 seconds. Get ready...\n\n";

        std::this_thread::sleep_for(std::chrono::seconds(3));

        std::stringstream command;
        command << "rosbag record -a -O " << get_path("", sub_action.prefix, ".bag");
        m_last_demonstration_name.assign(command.str().substr(20));
        command << " --duration=" << experiment.timeout;
        command << " __name:=mip_recorder &";

        std::cout << command.str() << std::endl;
        std::system(command.str().c_str());

        // Wait X ms for rosbag record to initialize (found experimentally), then start demo
        std::this_thread::sleep_for(std::chrono::milliseconds(m_rosbag_record_delay_ms));
        start_test();
        mip.begin_response(m_observation_frequency, m_max_observation_length, m_response_frequency, m_min_observation_length, false, false, false);

        end_demo_signal sig = wait_for_space_or_demo_stop(experiment.timeout);

        // Stop recording
        command.str("");
        command << "rosnode kill /mip_recorder";
        std::system(command.str().c_str());
        std::cout << "Stopping recording early!" << std::endl;

        // Stop Interaction Core and Robot Controller
        mip.end_response();
        mip.get_statistics("");
        robot_controller.end_control();

        end_demo_or_test(sig);

    }
}

void InteractionApplication::demo_status_callback(const std_msgs::Int32::ConstPtr& message)
{
    if(message->data)
    {
        m_demo_active.store(true);
    }
    else
    {
        m_demo_active.store(false);
    }
}

void InteractionApplication::demo_reset_callback(const std_msgs::Int32::ConstPtr& message)
{
    // Demonstration should send back a '2' when done resetting
    if(message->data == 2)
    {
        m_demo_done_reset = true;
    }
}

void InteractionApplication::end_demo_or_test(end_demo_signal sig)
{
    // If the user pressed space or 'q' (i.e, demo did not end naturally)
    if(sig == user_quit)
    {
        // Send kill signal to the Robot driver
        std::cout << "Stopping experiment...\n";
        std_msgs::Int32 message;
        message.data = 5;
        m_demo_status_publisher.publish(message);

        // Wait for demo_status_callback to receive the "demo ended" signal (confirmation the kill was successful)
        bool killed = wait_for_kill_to_finish();
        if(!killed)
        {
            std::cout << "Warning: Kill action has timed-out. The Robot driver never replied back with a 'demo ended' message after the kill signal was sent.\n";
        }
    }
    else if(sig == demo_ended_okay)
    {
        std::cout << "Received 'Demo Ended' signal.\n";
    }
    else
    {
        std::cout << "Demo timed out!\n";
    }

    if(m_reset_after_demo)
    {
        send_demo_reset();
        // Wait for `reset_timeout` seconds (maximum) for the reset to finish
        int reset_timeout = 8;
        if(wait_for_demo_reset(reset_timeout))
        {
            std::cout << "Demo Reset successfully.\n";
        }
        else
        {
            std::cout << "Warning: Demo Reset has timed-out. Please restart Interactive Application.\n";
            return;
        }
    }

    // Demo is no longer active
    m_demo_active.store(false);
    std::cout << "Experiment Ended!\n";
}

void InteractionApplication::start_demo()
{
    // Explicitly make demo flag true so anybody that checks flag before message is received won't cancel out.
    std::cout << "Starting Demo... Press space or 'q' to end demonstration. \n";
    m_demo_active.store(true);

    std_msgs::Int32 message;
    message.data = 1;
    m_demo_status_publisher.publish(message);
}

void InteractionApplication::start_test()
{
    // Explicitly make demo flag true so anybody that checks flag before message is received won't cancel out.
    std::cout << "Starting Test... Press space or 'q' to end interaction. \n";
    m_demo_active.store(true);

    std_msgs::Int32 message;
    message.data = 3;
    m_demo_status_publisher.publish(message);
}

void InteractionApplication::test_no_export(const Experiment& experiment)
{
    set_sim_time(false);

    RobotController robot_controller(*m_handle, experiment.controller_frequency, create_robot(experiment.controller));

    for(const auto& sub_action : experiment.sub_actions)
    {

        std::cout << sub_action.message << "\n";
        std::cout << "Please get ready and press [space] to begin test.\n\n";

        if(!wait_for_space())
        {
            return;
        }

        robot_controller.start_control();

        // Purposely re-create the IC object everytime so previous states are wiped out.
        InteractionCore mip(*m_handle, create_devices(experiment), create_predictor(experiment));

        std::cout << "Executing demonstration in 3 seconds. Get ready...\n\n";
        std::this_thread::sleep_for(std::chrono::seconds(3));

        start_test();
        mip.begin_response(m_observation_frequency, m_max_observation_length, m_response_frequency, m_min_observation_length, false, false, false);

        end_demo_signal sig = wait_for_space_or_demo_stop(experiment.timeout);

        // Stop Interaction Core and Robot Controller
        mip.end_response();
        mip.get_statistics("");
        robot_controller.end_control();

        end_demo_or_test(sig);

    }
}

void InteractionApplication::test_no_export_multiple_automated(const Experiment& experiment)
{
    // NOTE: reset should be implemented for automated demonstrations to work properly (unless robots always end in original state after interaction).
    set_sim_time(false);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "How many automated demonstrations do you want to run?" << std::endl;
    int num_trains;
    std::cin >> num_trains;
    if(!std::cin)
    {
        std::cout << "Bad input! Returning to main menu." << std::endl;
        return;
    }
    for(int i = 0; i < num_trains; i++)
    {
        for(const auto& sub_action : experiment.sub_actions)
        {
            RobotController robot_controller(*m_handle, experiment.controller_frequency, create_robot(experiment.controller));

            std::cout << sub_action.message << "\n";

            robot_controller.start_control();

            // Purposely re-create the IC object everytime so previous states are wiped out.
            InteractionCore mip(*m_handle, create_devices(experiment), create_predictor(experiment));

            std::cout << "Executing demonstration in 3 seconds...\n\n";

            std::this_thread::sleep_for(std::chrono::seconds(3));

            start_test();
            mip.begin_response(m_observation_frequency, m_max_observation_length, m_response_frequency, m_min_observation_length, false, false, false);

            end_demo_signal sig = wait_for_space_or_demo_stop(experiment.timeout);

            // Stop Interaction Core and Robot Controller
            mip.end_response();
            mip.get_statistics("");
            robot_controller.end_control();

            end_demo_or_test(sig);

            if(sig == user_quit)
            {
                return;
            }
        }
    }
}

void InteractionApplication::test_from_rosbag(const Experiment& experiment)
{
    set_sim_time(true);

    RobotController robot_controller(*m_handle, experiment.controller_frequency, create_robot(experiment.controller));

    std::string dir_path;

    std::cout << "Please enter the path to the directory with the scenario rosbags or leave blank to use the default.\n";
    getline(std::cin, dir_path);

    if(dir_path.empty())
    {
        dir_path = m_default_playback_dir;
        std::cout << "Using default directory: " << m_default_playback_dir << std::endl;
    }

    for(const auto& sub_action : experiment.sub_actions)
    {
        std::string prefix(sub_action.prefix);

        std::size_t num_files = 0;
        for(const auto& name : std::experimental::filesystem::directory_iterator(dir_path))
        {
            if(check_path(name.path(), prefix, ".bag"))
            {
                num_files += 1;
            }
        }

        std::size_t current_file = 1;
        for(const auto& name : std::experimental::filesystem::directory_iterator(dir_path))
        {
            if(check_path(name.path(), prefix, ".bag"))
            {
                std::cout << name.path().string() << ". File " << current_file << "/" << num_files << ".\n";

                std::cout << "\n** If you wish to abort this run, press [q] within the next 2 seconds.**\n\n";
                if(wait_for_quit(2))
                {
                    return;
                }

                std::cout << "Before control" << std::endl;

                robot_controller.start_control();

                std::cout << "After control" << std::endl;

                // Purposely re-create the IC object everytime so previous states are wiped out.
                InteractionCore mip(*m_handle, create_devices(experiment), create_predictor(experiment));

                std::cout << "Created core" << std::endl;

                std::stringstream command;
                command << "rosbag play --clock --hz=100 --delay=2 --quiet --rate=" << m_playback_factor << " ";
                command << name.path().string();

                std::cout << "Made command" << std::endl;

                mip.begin_response(m_observation_frequency, m_max_observation_length, m_response_frequency, m_min_observation_length, true, false, true);

                std::cout << "Begin response" << std::endl;

                std::this_thread::sleep_for(std::chrono::seconds(1));

                std::system(command.str().c_str());

                mip.end_response();
                mip.get_statistics(name.path().string());
                robot_controller.end_control();

                current_file += 1;
            }
        }
    }

    set_sim_time(false);
}

void InteractionApplication::test_from_csv(const Experiment& experiment)
{
    set_sim_time(false);

    std::string dir_path;
    std::vector<float> mse_values;

    std::cout << "Please enter the path to the directory with the scenario CSVs or leave blank to use the default.\n";
    getline(std::cin, dir_path);

    if(dir_path.empty())
    {
        dir_path = m_default_playback_dir;
        std::cout << "Using default directory: " << m_default_playback_dir << std::endl;
    }

    for(const auto& sub_action : experiment.sub_actions)
    {
        std::string prefix(sub_action.prefix);

        std::size_t num_files = 0;
        for(const auto& name : std::experimental::filesystem::directory_iterator(dir_path))
        {
            if(check_path(name.path(), prefix, ".csv"))
            {
                num_files += 1;
            }
        }

        float mse = 0.0;
        std::size_t current_file = 0;
        for(const auto& name : std::experimental::filesystem::directory_iterator(dir_path))
        {
            if(check_path(name.path(), prefix, ".csv"))
            {
                std::cout << name.path().string() << ". File " << current_file << "/" << num_files << ".\n";

                // Purposely re-create the IC object everytime so previous states are wiped out.
                InteractionCore mip(*m_handle, create_devices(experiment), create_predictor(experiment));

                float current_mse = mip.evaluate_demonstration(name.path().string(), m_min_observation_length);
                mse_values.push_back(current_mse);

                std::cout << "MSE: " << current_mse << std::endl;

                mse += current_mse;

                current_file += 1;
            }
        }

        std::cout << "MSE for session is: " << mse / static_cast<float>(num_files) << std::endl;
    }
}

void InteractionApplication::train_export_rosbag(const Experiment& experiment)
{
    set_sim_time(false);

    for(const auto& sub_action : experiment.sub_actions)
    {
        std::cout << sub_action.message << "\n";
        std::cout << "Please get ready and press [space] to begin demonstration.\n\n";

        if(!wait_for_space())
        {
            return;
        }

        std::cout << "Executing demonstration in 3 seconds. Get ready...\n\n";
        std::this_thread::sleep_for(std::chrono::seconds(3));

        std::stringstream command;
        command << "rosbag record -a -O " << get_path("", sub_action.prefix, ".bag");
        m_last_demonstration_name.assign(command.str().substr(20));
        std::cout << "Last demonstration name: " << m_last_demonstration_name << std::endl;
        command << " --duration=" << experiment.timeout;
        command << " __name:=mip_recorder &";
        std::cout << command.str() << std::endl;
        std::system(command.str().c_str());

        // Wait X ms for rosbag record to initialize (found experimentally), then start demo
        std::this_thread::sleep_for(std::chrono::milliseconds(m_rosbag_record_delay_ms));
        start_demo();

        end_demo_signal sig = wait_for_space_or_demo_stop(experiment.timeout);

        // Stop recording
        command.str("");
        command << "rosnode kill /mip_recorder";
        std::system(command.str().c_str());
        std::cout << "Finished recording!\n";

        end_demo_or_test(sig);
    }
}

void InteractionApplication::train_export_rosbag_multiple_automated(const Experiment& experiment)
{
    // NOTE: reset should be implemented for automated demonstrations to work properly (unless robots always end in original state after interaction).
    set_sim_time(false);

    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "How many automated demonstrations do you want to run?\n";
    int num_trains;
    std::cin >> num_trains;
    if(!std::cin)
    {
        std::cout << "Bad input! Returning to main menu.\n";
        return;
    }
    for(int i = 0; i < num_trains; i++)
    {
        for(const auto& sub_action : experiment.sub_actions)
        {
            std::cout << sub_action.message << "\n";

            std::cout << "Executing demonstration in 1 second...\n\n";
            std::this_thread::sleep_for(std::chrono::seconds(1));

            std::stringstream command;
            command << "rosbag record -a -O " << get_path("", sub_action.prefix, ".bag");
            m_last_demonstration_name.assign(command.str().substr(20));
            std::cout << "Last demonstration name: " << m_last_demonstration_name << std::endl;
            command << " --duration=" << experiment.timeout;
            command << " __name:=mip_recorder &";
            std::cout << command.str() << std::endl;
            std::system(command.str().c_str());

            // Wait X ms for rosbag record to initialize (found experimentally), then start demo
            std::this_thread::sleep_for(std::chrono::milliseconds(m_rosbag_record_delay_ms));
            start_demo();

            end_demo_signal sig = wait_for_space_or_demo_stop(experiment.timeout);

            // Stop recording
            command.str("");
            command << "rosnode kill /mip_recorder";
            std::system(command.str().c_str());
            std::cout << "Finished recording!\n";

            end_demo_or_test(sig);

            // If the person typed space or 'q' to quit, break and return.
            if(sig == user_quit)
            {
                return;
            }
        }
    }
}

void InteractionApplication::train_export_csv_from_rosbag(const Experiment& experiment)
{
    set_sim_time(true);

    std::string dir_path;
    std::cout << "Please enter the path to the directory with the scenario rosbags or leave blank to use the default.\n";
    getline(std::cin, dir_path);

    if(dir_path.empty())
    {
        dir_path = m_default_playback_dir;
        std::cout << "Using default directory: " << m_default_playback_dir << std::endl;
    }

    for(const auto& sub_action : experiment.sub_actions)
    {
        std::string prefix(sub_action.prefix);

        std::size_t num_files = 0;
        for(const auto& name : std::experimental::filesystem::directory_iterator(dir_path))
        {
            if(check_path(name.path(), prefix, ".bag"))
            {
                num_files += 1;
            }
        }

        std::size_t current_file = 1;
        for(const auto& name : std::experimental::filesystem::directory_iterator(dir_path))
        {
            if(check_path(name.path(), prefix, ".bag"))
            {
                std::cout << name.path().string() << ". File " << current_file << "/" << num_files << ".\n";

                // Purposely re-create the IC object everytime so previous states are wiped out.
                InteractionCore mip(*m_handle, create_devices(experiment), create_predictor(experiment));

                std::stringstream command;
                // command << "rosbag play --clock --hz=100 --delay=2 --duration=3 --quiet ";
                command << "rosbag play --clock --hz=100 --delay=2 --quiet ";
                command << name.path().string();

                mip.begin_demonstration(m_observation_frequency, m_max_observation_length, true, false, true);

                std::system(command.str().c_str());

                mip.end_demonstration();

                mip.export_demonstration(name.path().parent_path() / name.path().stem());

                current_file += 1;
            }
        }
    }

    set_sim_time(false);
}

void InteractionApplication::train_export_csv(const Experiment& experiment)
{
    set_sim_time(false);

    InteractionCore mip(*m_handle, create_devices(experiment), create_predictor(experiment));

    for(const auto& sub_action : experiment.sub_actions)
    {
        std::cout << sub_action.message << "\n";
        std::cout << "Please get ready and press [space] to begin demonstration.\n\n";

        if(!wait_for_space())
        {
            return;
        }

        std::cout << "Executing demonstration in 3 seconds. Get ready...\n\n";

        std::this_thread::sleep_for(std::chrono::seconds(3));

        start_demo();
        mip.begin_demonstration(m_observation_frequency, m_max_observation_length);

        std::cout << "Please press [space] to end demonstration or wait for the time-out.\n";

        // If we received a space before the timeout, then end early.
        end_demo_signal sig = wait_for_space_or_demo_stop(experiment.timeout);

        // Export demonstration
        mip.end_demonstration();
        mip.export_demonstration(get_path("", sub_action.prefix, ""));

        end_demo_or_test(sig);
    }
}

bool InteractionApplication::check_path(std::experimental::filesystem::path file_name, std::string prefix, std::string extension)
{
    if(file_name.stem().string().substr(0, prefix.size()) == prefix && file_name.extension().string() == extension)
    {
        return true;
    }
    return false;
}

std::string InteractionApplication::get_path(std::string input, std::string prefix, std::string extension)
{
    std::experimental::filesystem::path path;

    if(input.empty())
    {
        path = m_default_record_dir;
    }
    else
    {
        path = input;
    }

    path /= prefix;

    std::time_t timestamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    struct tm* timeinfo = localtime(&timestamp);

    std::stringstream file_name;
    file_name << path.string() << "_" << std::put_time(timeinfo, "%Y-%m-%d_%H-%M-%S") << extension;

    return file_name.str();
}

void InteractionApplication::perform_action(const Experiment& experiment, unsigned int option_idx, unsigned int action_idx)
{
    if(option_idx == 0)
    {
        if(action_idx == 0)
        {
            train_export_rosbag(experiment);
        }
        else if(action_idx == 2)
        {
            train_export_csv_from_rosbag(experiment);
        }
        else if(action_idx == 3)
        {
            if(m_last_demonstration_name.size() > 0)
            {
                std::cout << "Removing " << m_last_demonstration_name << std::endl;
                std::remove(m_last_demonstration_name.c_str());
                m_last_demonstration_name.clear();
            }
            else
            {
                std::cout << "Nothing to remove!" << std::endl;
            }
        }
        else if(action_idx == 1)
        {
            train_export_csv(experiment);
        }
        else if(action_idx == 4){
            train_export_rosbag_multiple_automated(experiment);
        }
    }
    else if(option_idx == 1)
    {
        if(action_idx == 0)
        {
            test_export_rosbag(experiment);
        }
        else if(action_idx == 1)
        {
            test_no_export(experiment);
        }
        else if(action_idx == 2)
        {
            test_from_rosbag(experiment);
        }
        else if(action_idx == 3)
        {
            test_from_csv(experiment);
        }
        else if(action_idx == 4)
        {
            test_no_export_multiple_automated(experiment);
        }
    }
}

void InteractionApplication::display_scenario_submenu(const Experiment& experiment, unsigned int option_idx)
{
    std::cout << "\n\nPlease select a scenario action:\n";

    unsigned int action_idx = 0;
    for(const auto& action : m_scenario_actions[option_idx])
    {
        std::cout << "  [" << action_idx++ << "] " << action << "\n";
    }

    char input = get_keyboard_input();

    if(input == 'q')
    {
        return;
    }
    else
    {
        for(action_idx = 0; action_idx < m_scenario_actions[option_idx].size(); action_idx++)
        {
            if(std::to_string(action_idx).c_str()[0] == input)
            {
                perform_action(experiment, option_idx, action_idx);
                break;
            }
        }
    }
}

void InteractionApplication::display_scenario_menu(const Experiment& experiment)
{
    std::cout << "\n\nPlease select a scenario category:\n";

    unsigned int option_idx = 0;
    for(const auto& option : m_scenario_options)
    {
        std::cout << "  [" << option_idx++ << "] " << option << "\n";
    }

    char input = get_keyboard_input();

    if(input == 'q')
    {
        return;
    }
    else
    {
        for(option_idx = 0; option_idx < m_scenario_options.size(); option_idx++)
        {
            if(std::to_string(option_idx).c_str()[0] == input)
            {
                display_scenario_submenu(experiment, option_idx);
                break;
            }
        }
    }
}

void InteractionApplication::display_main_menu()
{
    while(ros::ok())
    {
        std::cout << "Welcome to the MIP Interactive Application!\n\n";

        std::cout << "You may press [q] at any menu in order to cancel your selection and return to the previous menu.\n";
        std::cout << "You may press [d] at the main menu in order to open the dashboard.\n\n";

        std::cout << "Please select which scenario you would like to work with:\n";

        unsigned int scenario_idx = 0;
        for(const auto& experiment : m_experiments)
        {
            std::cout << "  [" << experiment.id << "] " << experiment.name << "\n";
        }

        char input = get_keyboard_input();

        if(input == 'q')
        {
            return;
        }
        else if(input == 'd')
        {
            std::stringstream command;
            command << "rosrun intprim_framework_ros dashboard_main.py &";
            std::system(command.str().c_str());
        }
        else
        {
            for(const auto& experiment : m_experiments)
            {
                if(std::to_string(experiment.id).c_str()[0] == input)
                {
                    display_scenario_menu(experiment);
                    break;
                }
            }
        }
    }
}

void InteractionApplication::set_sim_time(bool sim_time)
{
    m_handle->setParam("/use_sim_time", sim_time);

    // ROS is ridiculous and requires use_sim_time to be set before the node is initialized.
    // So kill the node and re-initialize.
    std::stringstream command;
    command << "rosnode kill /intprim_service_node";
    std::system(command.str().c_str());

    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "\n\n\n";
}

InteractionApplication::end_demo_signal InteractionApplication::wait_for_space_or_demo_stop(unsigned int timeout)
{
    char input = 'z';
    do
    {
        input = get_keyboard_input(timeout, true);
    }
    while(input != 'q' && input != ' ' && input != 'x' && input != CHAR_MIN);
    // if demo stops naturally (input set to x)
    if(input == 'x')
    {
        return demo_ended_okay;
    }
    // if user enters a ' ' or 'q' to manually kill
    else if(input == ' ' || input == 'q')
    {
        return user_quit;
    }
    else
    {
        return timed_out;
    }
}

bool InteractionApplication::wait_for_space(unsigned int timeout)
{
    char input = 'z';
    do
    {
        input = get_keyboard_input(timeout);
    }
    while(input != 'q' && input != ' ' && input != CHAR_MIN);

    return input == ' ';
}

bool InteractionApplication::wait_for_quit(unsigned int timeout)
{
    char input = 'z';
    do
    {
        input = get_keyboard_input(timeout);
    }
    while(input != 'q' && input != CHAR_MIN);

    return input == 'q';
}

void InteractionApplication::send_demo_reset()
{
    if(m_reset_after_demo)
    {
        m_demo_done_reset = false;
        std_msgs::Int32 message;
        message.data = 1;
        m_demo_reset_publisher.publish(message);
        std::cout << "Sent Demo Reset signal. Waiting for demo to reset... \n";
    }
    else
    {
        throw std::invalid_argument("Parameter \"interaction/reset_after_demo\" is missing or set to false. Please set to true to use this feature.");
    }

}

bool InteractionApplication::wait_for_kill_to_finish(int wait_seconds)
{
    float rate_hz = 2.0;
    float count = 0.0;
    ros::WallRate rate(rate_hz);
    while(ros::ok() && count < wait_seconds)
    {
        if(!m_demo_active)
        {
            break;
        }
        rate.sleep();
        count += 1.0/rate_hz;
    }
    return count < wait_seconds;
}

bool InteractionApplication::wait_for_demo_reset(int wait_seconds)
{
    float rate_hz = 2.0;
    if(m_reset_after_demo)
    {
        float count = 0.0;
        ros::WallRate rate(rate_hz);
        while(ros::ok() && count < wait_seconds)
        {
            if(m_demo_done_reset)
            {
                break;
            }
            rate.sleep();
            count += 1.0/rate_hz;
        }
        return count < wait_seconds;
    }
    else
    {
        throw std::invalid_argument("Parameter \"interaction/reset_after_demo\" is missing or set to false. Please set to true to use this feature.");
    }

}


char InteractionApplication::get_keyboard_input(unsigned int timeout, bool check_demo_status)
{
        char buf = 0;
        struct termios old = {0};

        // Disable terminal update
        if(tcgetattr(0, &old) < 0)
        {
            perror("tcsetattr()");
        }

        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 1;
        old.c_cc[VTIME] = 0;

        if(tcsetattr(0, TCSANOW, &old) < 0)
        {
            perror("tcsetattr ICANON");
        }

        auto start = std::chrono::system_clock::now();

        struct timeval* timeout_ptr = NULL;
        struct timeval timeout_struct;

        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> duration = end - start;

        do
        {
            // Perform read. This must also be initialized before each call as it gets modified by select.
            fd_set set;
            FD_ZERO(&set);
            FD_SET(0, &set);

            // The time structure must be set inside the while loop, as select apparently modifies this structure.
            if(timeout != 0)
            {
                timeout_struct.tv_sec = 0;
                timeout_struct.tv_usec = 100000; // 500 milliseconds

                timeout_ptr = &timeout_struct;
            }

            int status = select(1, &set, NULL, NULL, timeout_ptr);
            if(status == -1)
            {
                perror("Error in select.");
            }
            else if(status == 0)
            {
                // std::cout << "Are we hitting this constantly?" << std::endl;
                buf = CHAR_MIN;
            }
            else if(read(0, &buf, 1) < 0)
            {
                perror ("Error in read.");
            }

            end = std::chrono::system_clock::now();
            duration = end - start;

            // std::cout << "Input = \"" << buf << "\". Timeout is: " << timeout_struct.tv_sec << std::endl;
        } while(ros::ok() && (!check_demo_status || (check_demo_status && m_demo_active.load())) && (timeout != 0 && buf == CHAR_MIN && duration.count() < timeout));

        // Return x if the demo ends naturally (received 'done' signal)
        if(check_demo_status && !m_demo_active.load() && buf == CHAR_MIN)
        {
            buf = 'x';
        }

        // Re-enable terminal update
        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;

        if(tcsetattr(0, TCSADRAIN, &old) < 0)
        {
            perror ("tcsetattr ~ICANON");
        }

        return (buf);
}
