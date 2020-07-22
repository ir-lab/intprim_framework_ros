/* @author Joseph Campbell <jacampb1@asu.edu>, Interactive Robotics Lab, Arizona State University */
#pragma once

/*!
    @addtogroup interaction_core
    @{
*/

#include "device_interface.h"

#include <fstream>
#include <vector>

/*!
    A RobotInterface is a polymorphic interface and represents a single controllable robot.
    This doesn't necessarily have to be a robot but can be any controllable device that is to be used by RobotController as long as it fulfills this interface.
    This class is a purely abstract interface and must be inherited from in order to create new robots.
*/
class RobotInterface : public DeviceInterface
{
public:
    /*!
        Pure virtual destructor. Prevents direct instantiation of this class.
    */
    virtual ~RobotInterface() = 0;

    /*!
        Returns the most current state from this device.

        @returns DeviceState This is a covariant return type. The polymorphic type corresponding to this Device will be returned in actuality.
    */
    virtual const DeviceState& get_state() = 0;

    /*!
        Publishes a state to the controllable device. The state in the given trajectory should serve as a reference signal for the underlying robot controller.

        @param trajectory The vector of states to serve as a reference trajectory
        @param trajectory_idx The index of the specific state in the reference trajectory to target.
    */
    virtual void publish_state(const std::vector<float>& trajectory, std::size_t trajectory_idx) = 0;
};

/** @} */

inline RobotInterface::~RobotInterface()
{
}
