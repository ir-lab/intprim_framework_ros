/* @author Joseph Campbell <jacampb1@asu.edu>, Interactive Robotics Lab, Arizona State University */
#include "robot_controller.h"
#include "devices/robot_interface.h"

#ifdef IRL_ROBOTS_AVAILABLE
    #include "devices/ur5.h"
    #include "devices/lbr4.h"
#endif


#include "ros/ros.h"

#include <stdexcept>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_controller");

    ros::NodeHandle handle;

    int control_frequency = 1;
    std::unique_ptr<RobotInterface> robot_interface;

    std::string controller_type;
    if(!handle.getParam("control/controller", controller_type))
    {
        throw std::invalid_argument("Missing \"control/controller\" ROS parameter. Must be defined to run robot_controller as stand-alone node.");
    }

    XmlRpc::XmlRpcValue param_list;
    handle.getParam("control", param_list);

    if(controller_type == "ur5")
    {
        #ifdef IRL_ROBOTS_AVAILABLE
            robot_interface = std::unique_ptr<UR5Interface>(new UR5Interface(handle, "regular"));
            control_frequency = param_list[controller_type]["control_frequency"];
        #else
            throw std::runtime_error("Experiment uses UR5Interface but IRL robots unavailable.");
        #endif
    }
    if(controller_type == "ur5c")
    {
        #ifdef IRL_ROBOTS_AVAILABLE
                robot_interface = std::unique_ptr<UR5Interface>(new UR5Interface(handle, "control_sim"));
                control_frequency = param_list[controller_type]["control_frequency"];
        #else
                throw std::runtime_error("Experiment uses UR5CInterface but IRL robots unavailable.");
        #endif
    }
    else if(controller_type == "lbr4")
    {
        #ifdef IRL_ROBOTS_AVAILABLE
            robot_interface = std::unique_ptr<LBR4Interface>(new LBR4Interface(handle));
            control_frequency = param_list[controller_type]["control_frequency"];
        #else
            throw std::runtime_error("Experiment uses LBR4Interface but IRL robots unavailable.");
        #endif
    }
    else
    {
        throw std::invalid_argument("Unsupported \"control/controller\".");
    }

    RobotController controller(handle, control_frequency, std::move(robot_interface));

    controller.control_robot();
}
