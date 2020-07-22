/* @author Joseph Campbell <jacampb1@asu.edu>, Interactive Robotics Lab, Arizona State University */
#pragma once

/*!
    @addtogroup interaction_core
    @{
*/

#include <fstream>
#include <vector>

/*!
    A DeviceState is a polymorphic interface and represents a single observation (state) of a single device.
    This can correspond to a measurement from a sensor, e.g., camera, accelerometer, or any other input source.
    This class is a purely abstract interface and must be inherited from in order to create new devices.
*/
class DeviceState
{
public:
    /*!
        Pure virtual destructor. Prevents direct instantiation of this class.
    */
    virtual ~DeviceState() = 0;

    /*!
        Append the values in the current state to the end of the given trajectory.
        This method incrementally builds up an observation vector in a modular way by having each DeviceInterface append its observation values to a single vector in a deterministic manner.

        @param trajectory The vector to append to
    */
    virtual void to_matrix(std::vector<float>& trajectory) const = 0;

    /*!
        Determines whether the current state is valid, i.e., it has received actual state values from the device.

        @returns True if valid, false if not.
    */
    virtual bool valid() const = 0;

    /*!
        Checks if the specified state in the given trajectory is within a pre-specified threshold of the current state.

        @param trajectory A trajectory of states.
        @param trajectory_idx The specific state to compare against.

        @returns True if within the threshold, false if not.
    */
    virtual bool within_threshold(const std::vector<float>& trajectory, std::size_t trajectory_idx) const = 0;
};

/*!
    A DeviceInterface is a polymorphic interface and represents a single device.
    This can correspond to a sensor, e.g., camera, accelerometer, or any other input source.
    This class is a purely abstract interface and must be inherited from in order to create new devices.
*/
class DeviceInterface
{
public:
    /*!
        Pure virtual destructor. Prevents direct instantiation of this class.
    */
    virtual ~DeviceInterface() = 0;

    /*!
        Returns the most current state from this device.

        @returns DeviceState This is a covariant return type. The polymorphic type corresponding to this Device will be returned in actuality.
    */
    virtual const DeviceState& get_state() = 0;
};

/** @} */

inline DeviceState::~DeviceState()
{
}

inline DeviceInterface::~DeviceInterface()
{
}
