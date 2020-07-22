/* @author Joseph Campbell <jacampb1@asu.edu>, Interactive Robotics Lab, Arizona State University */
#include "devices/ur5.h"

#include <boost/bind.hpp>
#include <chrono>
#include <regex>
#include <sstream>
#include <string>
#include <thread>

std::string UR5State::JOINT_NAMES[UR5State::NUM_JOINTS];
double UR5State::JOINT_THRESHOLD = 0.0;

UR5State::UR5State() :
    m_message(),
    m_valid(false)
{

}

void UR5State::to_matrix(std::vector<float>& trajectory) const
{
    if(m_message)
    {
        for(std::size_t joint_idx = 0; joint_idx < NUM_JOINTS; ++joint_idx)
        {
            trajectory.push_back(m_message->position[joint_idx]);
        }
    }
    else
    {
        for(std::size_t joint_idx = 0; joint_idx < NUM_JOINTS; ++joint_idx)
        {
            trajectory.push_back(0.0);
        }
    }
}

const sensor_msgs::JointState::ConstPtr& UR5State::get_message()
{
    return m_message;
}

void UR5State::set_message(const sensor_msgs::JointState::ConstPtr& message)
{
    m_message = message;
    m_valid = true;
}

bool UR5State::valid() const
{
    return m_valid;
}

bool UR5State::within_threshold(const std::vector<float>& trajectory, std::size_t trajectory_idx) const
{
    if(m_message)
    {
        for(std::size_t joint_idx = 0; joint_idx < NUM_JOINTS; ++joint_idx)
        {
            if(std::abs(m_message->position[joint_idx] - trajectory[joint_idx + trajectory_idx]) > JOINT_THRESHOLD)
            {
                return false;
            }
        }
        return true;
    }
    return false;
}

UR5Interface::UR5Interface(ros::NodeHandle handle, std::string robot_type) :
    m_state_subscriber(),
    m_state_publisher(),
    m_current_state(),
    m_publish_message()
{
    for(int joint = 0; joint < UR5State::NUM_JOINTS; joint++)
    {
        std::string name;
        if(robot_type.compare("control_sim") == 0)
        {
             name = "UR5c_joint" + std::to_string(joint+1);
             UR5State::JOINT_NAMES[joint] = name;
        }
        else if(robot_type.compare("observe_sim") == 0)
        {
             name = "UR5l_joint" + std::to_string(joint+1);
             UR5State::JOINT_NAMES[joint] = name;
        }
        else
        {
             name = "UR5_joint" + std::to_string(joint+1);
             UR5State::JOINT_NAMES[joint] = name;
        }
        m_publish_message.name.push_back(name);
        m_publish_message.position.push_back(0.0);
    }
    if(robot_type.compare("control_sim") == 0)
    {
        handle.getParam("control/ur5c/joint_distance_threshold", UR5State::JOINT_THRESHOLD);
        m_state_publisher = handle.advertise<sensor_msgs::JointState>("/robot/ur5c/control", 1);
        m_state_subscriber = handle.subscribe("/robot/ur5c/state", 1, &UR5Interface::state_callback, this);
    }
    else if(robot_type.compare("observe_sim") == 0)
    {
        handle.getParam("control/ur5l/joint_distance_threshold", UR5State::JOINT_THRESHOLD);
        m_state_publisher = handle.advertise<sensor_msgs::JointState>("/robot/ur5l/control", 1);
        m_state_subscriber = handle.subscribe("/robot/ur5l/state", 1, &UR5Interface::state_callback, this);
    }
    else{
        handle.getParam("control/ur5/joint_distance_threshold", UR5State::JOINT_THRESHOLD);
        m_state_publisher = handle.advertise<sensor_msgs::JointState>("/robot/ur5/control", 1);
        m_state_subscriber = handle.subscribe("/robot/ur5/state", 1, &UR5Interface::state_callback, this);
    }
}

const UR5State& UR5Interface::get_state()
{
    return m_current_state;
}

void UR5Interface::publish_state(const std::vector<float>& trajectory, std::size_t trajectory_idx)
{
    for(std::size_t joint_idx = 0; joint_idx < UR5State::NUM_JOINTS; ++joint_idx)
    {
        m_publish_message.position[joint_idx] = trajectory[trajectory_idx + joint_idx];
    }

    m_state_publisher.publish(m_publish_message);
}

void UR5Interface::state_callback(const sensor_msgs::JointState::ConstPtr& message)
{
    m_current_state.set_message(message);
}
