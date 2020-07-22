/* @author Joseph Campbell <jacampb1@asu.edu>, Interactive Robotics Lab, Arizona State University */
#include "devices/single_predictor.h"

#include <intprim_framework_ros/InitializeState.h>

SinglePredictorInterface::SinglePredictorInterface(ros::NodeHandle* handle) :
    m_demonstrationClient(),
    m_initializeClient(),
    m_evaluateClient(),
    m_responseClient(),
    m_statisticsClient(),
    m_intprim_id(-1)
{
    m_demonstrationClient = handle->serviceClient<intprim_framework_ros::AddDemonstration>("/ip/addDemonstration");
    m_initializeClient    = handle->serviceClient<intprim_framework_ros::InitializeState>("/ip/initializeState");
    m_evaluateClient      = handle->serviceClient<intprim_framework_ros::EvaluateTrajectory>("/ip/evaluateTrajectory");
    m_responseClient      = handle->serviceClient<intprim_framework_ros::GenerateTrajectory>("/ip/generateTrajectory");
    m_statisticsClient    = handle->serviceClient<intprim_framework_ros::GetStatistics>("/ip/getStatistics");
}

bool SinglePredictorInterface::add_demonstration(intprim_framework_ros::AddDemonstration& message)
{
    message.request.interaction_id = m_intprim_id;
    return m_demonstrationClient.call(message);
}

bool SinglePredictorInterface::initialize()
{
    intprim_framework_ros::InitializeState message;
    return m_initializeClient.call(message);
}

bool SinglePredictorInterface::evaluate(intprim_framework_ros::EvaluateTrajectory& message)
{
    message.request.interaction_id = m_intprim_id;
    return m_evaluateClient.call(message);
}

bool SinglePredictorInterface::predict(intprim_framework_ros::GenerateTrajectory& message)
{
    message.request.interaction_id = m_intprim_id;
    return m_responseClient.call(message);
}

bool SinglePredictorInterface::get_statistics(intprim_framework_ros::GetStatistics& message)
{
    message.request.interaction_id = m_intprim_id;
    m_statisticsClient.call(message);
}
