/* @author Joseph Campbell <jacampb1@asu.edu>, Interactive Robotics Lab, Arizona State University */
#include "interaction_core.h"

#include <fstream>
#include <functional>
#include <sstream>
#include <time.h>
#include <thread>

InteractionCore::InteractionCore(ros::NodeHandle handle, std::vector<std::unique_ptr<DeviceInterface>> device_interfaces, std::unique_ptr<PredictorInterface> predictor_interface) :
    m_handle(handle),
    m_device_interfaces(std::move(device_interfaces)),
    m_predictor_interface(std::move(predictor_interface)),
    m_clock_listener(),
    m_control_listener(),
    m_robot_talker(),
    m_trajectory(),
    m_trajectory_mutex(),
    m_observation_thread(),
    m_response_thread(),
    m_respond(false),
    m_clock_received(false),
    m_control_received(false),
    m_observe(false),
    m_state_size(0),
    m_observation_noise_path(),
    m_control_topic()
{
    handle.getParam("control/control_topic", m_control_topic);

    m_robot_talker = handle.advertise<intprim_framework_ros::Trajectory>(m_control_topic, 1);

    XmlRpc::XmlRpcValue bip_list;
    if(handle.getParam("bip", bip_list))
    {
        for(size_t bip_idx = 0; bip_idx < bip_list.size(); bip_idx++)
        {
            if(bip_list[bip_idx]["primary"])
            {
                m_observation_noise_path = std::string(bip_list[bip_idx]["observation_noise"]);
            }
        }
    }
}

void InteractionCore::clock_callback(const rosgraph_msgs::Clock::ConstPtr& message)
{
    m_clock_received.store(true);
}

void InteractionCore::control_callback(const intprim_framework_ros::Trajectory::ConstPtr& message)
{
   m_control_received.store(true);
}

void InteractionCore::update_trajectory(unsigned int max_observation_length, bool check_valid)
{
    std::lock_guard<std::mutex> lock_trajectory(m_trajectory_mutex);

    // Make sure all the current device states are valid before we start populating trajectories.
    if(check_valid)
    {
        for(auto& interface : m_device_interfaces)
        {
            if(!interface->get_state().valid())
            {
                return;
            }
        }
    }

    for(auto& interface : m_device_interfaces)
    {
        interface->get_state().to_matrix(m_trajectory);
    }

    if(m_state_size == 0)
    {
        m_state_size = m_trajectory.size();
    }
    if(m_trajectory.size() / m_state_size > max_observation_length)
    {
        m_trajectory.erase(m_trajectory.begin(), m_trajectory.begin() + m_state_size);
    }
}

void InteractionCore::begin_demonstration(unsigned int observation_rate, unsigned int max_observation_length, bool require_clock, bool wait_for_control, bool check_valid)
{
    //if(m_observation_thread.get_id() == std::thread::id)
    if(m_observe.load())
    {
        std::cout << "Error: attempting to begin demonstration twice." << std::endl;
        return;
    }

    {
        // Clear any existing trajectories.
        std::lock_guard<std::mutex> lock(m_trajectory_mutex);

        // Clear current trajectory
        m_trajectory.clear();
    }

    m_observe.store(true);
    m_observation_thread = std::thread(&InteractionCore::observe, this, observation_rate, max_observation_length, require_clock, wait_for_control, check_valid);
}

void InteractionCore::end_demonstration()
{
    m_observe.store(false);
    m_observation_thread.join();
}

void InteractionCore::export_demonstration(std::string file_name)
{
    std::lock_guard<std::mutex> lock(m_trajectory_mutex);

    std::ofstream out_file;
    out_file.open(file_name + ".csv");

    auto trajectory_idx = 0;
    for(const auto& state : m_trajectory)
    {
        out_file << state;

        if(++trajectory_idx % m_state_size == 0)
        {
            out_file << "\n";
        }
        else
        {
            out_file << ",";
        }
    }

    out_file.close();

    std::cout << "Export complete. File location: " << file_name << ".csv" << std::endl;

    // Clear current trajectory
    m_trajectory.clear();
}

float InteractionCore::evaluate_demonstration(std::string file_name, unsigned int minimum_observation_length)
{
    m_trajectory.clear();

    m_predictor_interface->initialize();

    // Initialize covariance matrix
    std::ifstream noise_file;
    std::vector<float> noise_matrix;
    noise_file.open(m_observation_noise_path);

    while(noise_file)
    {
        std::string       line;
        std::getline(noise_file, line);
        std::stringstream lineStream(line);
        std::string       cell;
        while(std::getline(lineStream, cell, ','))
        {
            noise_matrix.push_back(std::stof(cell));
        }
    }

    noise_file.close();

    intprim_framework_ros::EvaluateTrajectory message;
    message.request.covariance = noise_matrix;

    std::ifstream in_file;
    in_file.open(file_name);

    float mse = 0.0;
    float num_mse = 0;

    std::string line;
    while(std::getline(in_file, line))
    {
        auto idx = 0;
        std::stringstream  lineStream(line);
        std::string        cell;
        while(std::getline(lineStream,cell,','))
        {
            // Convert everything to float
            m_trajectory.push_back(std::stof(cell));
            idx++;
        }
        if(m_state_size == 0)
        {
            m_state_size = idx;
        }

        if(m_trajectory.empty() || m_trajectory.size() / m_state_size < minimum_observation_length)
        {
            continue;
        }

        message.request.observed_trajectory.stride = m_state_size;
        // Add observed trajectory
        message.request.observed_trajectory.data.assign(m_trajectory.begin(), m_trajectory.end());

        m_trajectory.clear();

        m_predictor_interface->evaluate(message);

        mse += message.response.mse;
        num_mse += 1.0;
    }

    in_file.close();

    return mse / num_mse;
}

void InteractionCore::begin_response(unsigned int observation_rate, unsigned int max_observation_length, unsigned int response_rate, unsigned int minimum_observation_length, bool require_clock, bool wait_for_control, bool check_valid)
{
    if(m_observe.load() || m_respond.load())
    {
        std::cout << "Error: attempting to begin demonstration twice." << std::endl;
        return;
    }

    {
        // Clear any existing trajectories.
        std::lock_guard<std::mutex> lock(m_trajectory_mutex);

        // Clear current trajectory
        m_trajectory.clear();
    }

    m_predictor_interface->initialize();

    m_observe.store(true);
    m_respond.store(true);
    m_observation_thread = std::thread(&InteractionCore::observe, this, observation_rate, max_observation_length, require_clock, wait_for_control, check_valid);
    m_response_thread = std::thread(&InteractionCore::respond, this, response_rate, minimum_observation_length);
}

void InteractionCore::end_response()
{
    m_observe.store(false);
    m_respond.store(false);
    m_observation_thread.join();
    m_response_thread.join();
}

void InteractionCore::get_statistics(std::string bag_file)
{
    intprim_framework_ros::GetStatistics message;
    message.request.bag_file = bag_file;
    m_predictor_interface->get_statistics(message);
}

void InteractionCore::send_control_command(const intprim_framework_ros::Trajectory& trajectory)
{
    m_robot_talker.publish(trajectory);
}

void InteractionCore::respond(unsigned int response_rate, unsigned int minimum_observation_length)
{
    ros::WallRate loop_rate(response_rate);

    // Initialize covariance matrix
    std::ifstream noise_file;
    std::vector<float> noise_matrix;
    noise_file.open(m_observation_noise_path);

    while(noise_file)
    {
        std::string       line;
        std::getline(noise_file, line);
        std::stringstream lineStream(line);
        std::string       cell;
        while(std::getline(lineStream, cell, ','))
        {
            noise_matrix.push_back(std::stof(cell));
        }
    }

    noise_file.close();

    intprim_framework_ros::GenerateTrajectory message;
    message.request.covariance = noise_matrix;

    while(ros::ok() && m_respond.load())
    {

        {
            // Retrieve trajectory and send it to IP
            std::lock_guard<std::mutex> lock(m_trajectory_mutex);

            if(m_trajectory.empty() || m_trajectory.size() / m_state_size < minimum_observation_length)
            {
                continue;
            }

            message.request.observed_trajectory.stride = m_state_size;
            // Add observed trajectory
            message.request.observed_trajectory.data.assign(m_trajectory.begin(), m_trajectory.end());

            m_trajectory.clear();
        }

        m_predictor_interface->predict(message);

        send_control_command(message.response.generated_trajectory);

        loop_rate.sleep();
    }
}

void InteractionCore::observe(unsigned int observation_rate, unsigned int max_observation_length, bool require_clock, bool wait_for_control, bool check_valid)
{
    ros::WallRate loop_rate(observation_rate);

    if(require_clock)
    {
        m_clock_received.store(false);
        m_clock_listener = m_handle.subscribe("/clock", 1, &InteractionCore::clock_callback, this);
    }

    if(wait_for_control)
    {
        m_control_received.store(false);
        m_control_listener = m_handle.subscribe(m_control_topic, 1, &InteractionCore::control_callback, this);

    }

    while(ros::ok() && m_observe.load())
    {
        // Get latest states.
        ros::spinOnce();

        if((wait_for_control && m_control_received.load()) || !wait_for_control)
        {
            // Collect current state and append it to trajectory.
            if(require_clock && m_clock_received.load())
            {
                update_trajectory(max_observation_length, check_valid);
                m_clock_received.store(false);
            }
            else if(!require_clock)
            {
                update_trajectory(max_observation_length, check_valid);
            }
        }
        loop_rate.sleep();
    }

    if(require_clock)
    {
        m_clock_listener.shutdown();
    }
    if(wait_for_control)
    {
        m_control_listener.shutdown();
    }
}
