/* @author Joseph Campbell <jacampb1@asu.edu>, Interactive Robotics Lab, Arizona State University */
#include "interaction_application.h"
#include "ros/ros.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "interaction_application");

    ros::NodeHandle handle;

    InteractionApplication app(&handle);

    app.run();
}
