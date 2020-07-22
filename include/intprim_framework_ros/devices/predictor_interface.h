/* @author Joseph Campbell <jacampb1@asu.edu>, Interactive Robotics Lab, Arizona State University */
#pragma once

/*!
    @addtogroup interaction_core
    @{
*/

#include <intprim_framework_ros/AddDemonstration.h>
#include <intprim_framework_ros/EvaluateTrajectory.h>
#include <intprim_framework_ros/GenerateTrajectory.h>
#include <intprim_framework_ros/GetStatistics.h>

/*!
    A PredictorInterface is a polymorphic interface and represents a communication bridge to IntPrim.
    Currently, the only predictor that is implemented is for a single BIP prediction model, however, this interface allows for future versions that can switch between multiple models.
    This class is a purely abstract interface and must be inherited from in order to create new predictors.

    It may seem like poor architecture design to have the APIs directly pass the ROS service messages by reference, but there is a trade off to be had:
    1) The original buffer in InteractionCore can be immediately copied into the message and unlocked, so as to prevent a needless secondary copy (it's possible move semantics can negate this).
    2) The response can be directly utilized without having to make another copy (or move, if supported) to return it.
*/
class PredictorInterface
{
public:
    /*!
        Pure virtual destructor. Prevents direct instantiation of this class.
    */
    virtual ~PredictorInterface() = 0;

    /*!
        Add the given demonstration to the IntPrim model.

        @param message The AddDemonstration service message to use. The observed trajectory must be already populated and response will be populated upon completion.

        @returns True if successful, false if not.
    */
    virtual bool add_demonstration(intprim_framework_ros::AddDemonstration& message) = 0;

    /*!
        Initializes the IntPrim model. Needs to be called before a new interaction is started.

        @returns True if successful, false if not.
    */
    virtual bool initialize() = 0;

    /*!
        Evaluates the given demonstration (requires a trained IntPrim model).

        @param message The EvaluateTrajectory service message to use. The observed trajectory must be already populated and response will be populated upon completion.

        @returns True if successful, false if not.
    */
    virtual bool evaluate(intprim_framework_ros::EvaluateTrajectory& message) = 0;

    /*!
        Performs inference and returns the predicted response, given the observed trajectory.

        @param message The GenerateTrajectory service message to use. The observed trajectory must be already populated and response will be populated upon completion.

        @returns True if successful, false if not.
    */
    virtual bool predict(intprim_framework_ros::GenerateTrajectory& message) = 0;

    /*!
        Exports the statistics for the previous interaction. Must be called after an interaction but before intialize is called again, or will result in failure.

        @param message The GetStatistics message to use.

        @returns True if succcessful, false if not.
    */
    virtual bool get_statistics(intprim_framework_ros::GetStatistics& message) = 0;
};

/** @} */

inline PredictorInterface::~PredictorInterface()
{
}
