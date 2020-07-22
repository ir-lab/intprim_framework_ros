/* @author Joseph Campbell <jacampb1@asu.edu>, Interactive Robotics Lab, Arizona State University */
#pragma once

#include "predictor_interface.h"
#include "ros/ros.h"

class SinglePredictorInterface : public PredictorInterface
{
    public:
        SinglePredictorInterface(ros::NodeHandle* handle);

        bool add_demonstration(intprim_framework_ros::AddDemonstration& message);
        bool initialize();
        bool evaluate(intprim_framework_ros::EvaluateTrajectory& message);
        bool predict(intprim_framework_ros::GenerateTrajectory& message);
        bool get_statistics(intprim_framework_ros::GetStatistics& message);

    private:
        ros::ServiceClient              m_demonstrationClient;
        ros::ServiceClient              m_initializeClient;
        ros::ServiceClient              m_evaluateClient;
        ros::ServiceClient              m_responseClient;
        ros::ServiceClient              m_statisticsClient;
        int8_t                          m_intprim_id;
};
