/* @author Joseph Campbell <jacampb1@asu.edu>, Interactive Robotics Lab, Arizona State University */
#include "robot_controller.h"

RobotController::RobotController(ros::NodeHandle handle, unsigned int control_frequency, std::unique_ptr<RobotInterface> robot_interface) :
    m_controller_listener(),
    m_robot_interface(std::move(robot_interface)),
    m_current_trajectory(),
    m_control_thread(),
    m_active(false),
    m_trajectory_index(0),
    m_control_frequency(control_frequency)
{
    std::string control_topic;
    handle.getParam("control/control_topic", control_topic);

    m_controller_listener = handle.subscribe(control_topic, 1, &RobotController::control_callback, this);
}

void RobotController::control_callback(const intprim_framework_ros::Trajectory::ConstPtr& message)
{
    m_current_trajectory = message;
    m_trajectory_index = 0;
}

void RobotController::control_robot()
{
    m_active.store(true);
    m_control_thread = std::thread(&RobotController::control, this);
    m_control_thread.join();
}

void RobotController::control()
{
    ros::WallRate rate(m_control_frequency);

    while(ros::ok() && m_active.load())
    {
        ros::spinOnce();

        if(m_current_trajectory && m_current_trajectory->data.size() > 0)
        {
            const DeviceState& current_state = m_robot_interface->get_state();

            bool update_index = current_state.within_threshold(m_current_trajectory->data, m_current_trajectory->stride * m_trajectory_index);

            if(update_index && m_trajectory_index + 1 < m_current_trajectory->data.size() / m_current_trajectory->stride)
            {
                m_trajectory_index += 1;
            }

            std::cout << "Trajectory index " << m_trajectory_index << std::endl;

            m_robot_interface->publish_state(m_current_trajectory->data, m_current_trajectory->stride * m_trajectory_index);
        }

        rate.sleep();
    }
}

void RobotController::start_control()
{
    m_active.store(true);
    m_control_thread = std::thread(&RobotController::control, this);
}

void RobotController::end_control()
{
    m_active.store(false);
    m_control_thread.join();
}
