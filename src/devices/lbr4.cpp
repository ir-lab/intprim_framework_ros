/* @author Joseph Campbell <jacampb1@asu.edu>, Interactive Robotics Lab, Arizona State University */
#include "devices/lbr4.h"

#include <boost/bind.hpp>
#include <chrono>
#include <regex>
#include <sstream>
#include <string>
#include <thread>

const std::string LBR4State::JOINT_NAMES[LBR4State::NUM_JOINTS] = {
    "LBR4p_joint1",
    "LBR4p_joint2",
    "LBR4p_joint3",
    "LBR4p_joint4",
    "LBR4p_joint5",
    "LBR4p_joint6",
    "LBR4p_joint7"
};

double LBR4State::JOINT_THRESHOLD = 0.0;


LBR4State::LBR4State() :
    m_message(),
    m_valid(false)
{

}

void LBR4State::to_matrix(std::vector<float>& trajectory) const
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

const sensor_msgs::JointState::ConstPtr& LBR4State::get_message()
{
    return m_message;
}

void LBR4State::set_message(const sensor_msgs::JointState::ConstPtr& message)
{
    m_message = message;
    m_valid = true;
}

bool LBR4State::valid() const
{
    return m_valid;
}

bool LBR4State::within_threshold(const std::vector<float>& trajectory, std::size_t trajectory_idx) const
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

LBR4Interface::LBR4Interface(ros::NodeHandle handle) :
    m_state_subscriber(),
    m_state_publisher(),
    m_current_state(),
    m_publish_message()
{
    handle.getParam("control/lbr4/joint_distance_threshold", LBR4State::JOINT_THRESHOLD);

    for(const auto& name : LBR4State::JOINT_NAMES)
    {
        m_publish_message.name.push_back(name);
        m_publish_message.position.push_back(0.0);
    }

    m_state_publisher = handle.advertise<sensor_msgs::JointState>("/robot/lbr4/control", 1);
    m_state_subscriber = handle.subscribe("/robot/lbr4/state", 1, &LBR4Interface::state_callback, this);
}

const LBR4State& LBR4Interface::get_state()
{
    return m_current_state;
}

void LBR4Interface::publish_state(const std::vector<float>& trajectory, std::size_t trajectory_idx)
{
    for(std::size_t joint_idx = 0; joint_idx < LBR4State::NUM_JOINTS; ++joint_idx)
    {
        m_publish_message.position[joint_idx] = trajectory[trajectory_idx + joint_idx];
    }

    m_state_publisher.publish(m_publish_message);
}

void LBR4Interface::state_callback(const sensor_msgs::JointState::ConstPtr& message)
{
    m_current_state.set_message(message);
}
