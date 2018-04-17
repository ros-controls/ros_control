///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of PAL Robotics S.L. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/// \author Adolfo Rodriguez Tsouroukdissian

#ifndef TRANSMISSION_INTERFACE_TRANSMISSION_H
#define TRANSMISSION_INTERFACE_TRANSMISSION_H

#include <cstddef>
#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>


namespace transmission_interface
{

/**
 * @defgroup transmission_types Transmission types
 */

/**
 * \brief Contains pointers to raw data representing the position, velocity and acceleration of a transmission's
 * actuators.
 */
struct ActuatorData
{
  std::vector<double*> position;
  std::vector<double*> velocity;
  std::vector<double*> effort;
};

/**
 * \brief Contains pointers to raw data representing the position, velocity and acceleration of a transmission's
 * joints.
 */
struct JointData
{
  std::vector<double*> position;
  std::vector<double*> velocity;
  std::vector<double*> effort;
};

/**
 * \brief Abstract base class for representing mechanical transmissions.
 *
 * Mechanical transmissions transform effort/flow variables such that their product (power) remains constant.
 * Effort variables for linear and rotational domains are \e force and \e torque; while the flow variables are
 * respectively linear velocity and angular velocity.
 *
 * In robotics it is customary to place transmissions between actuators and joints. This interface adheres to this
 * naming to identify the input and output spaces of the transformation.
 * The provided interfaces allow bidirectional mappings between actuator and joint spaces for effort, velocity and
 * position. Position is not a power variable, but the mappings can be implemented using the velocity map plus an
 * integration constant representing the offset between actuator and joint zeros.
 *
 * \par Credit
 * This interface was inspired by similar existing implementations by PAL Robotics, S.L. and Willow Garage Inc.
 *
 * \note Implementations of this interface must take care of realtime-safety if the code is to be run in realtime
 * contexts, as is often the case in robot control.
 */
class Transmission
{
public:
  virtual ~Transmission() {}

  /**
   * \brief Transform \e effort variables from actuator to joint space.
   * \param[in]  act_data Actuator-space variables.
   * \param[out] jnt_data Joint-space variables.
   * \pre All non-empty vectors must contain valid data and their size should be consistent with the number of
   * transmission actuators and joints.
   * Data vectors not used in this map can remain empty.
   */
  virtual void actuatorToJointEffort(const ActuatorData& act_data,
                                           JointData&    jnt_data) = 0;

  /**
   * \brief Transform \e velocity variables from actuator to joint space.
   * \param[in]  act_data Actuator-space variables.
   * \param[out] jnt_data Joint-space variables.
   * \pre All non-empty vectors must contain valid data and their size should be consistent with the number of
   * transmission actuators and joints.
   * Data vectors not used in this map can remain empty.
   */
  virtual void actuatorToJointVelocity(const ActuatorData& act_data,
                                             JointData&    jnt_data) = 0;

  /**
   * \brief Transform \e position variables from actuator to joint space.
   * \param[in]  act_data Actuator-space variables.
   * \param[out] jnt_data Joint-space variables.
   * \pre All non-empty vectors must contain valid data and their size should be consistent with the number of
   * transmission actuators and joints.
   * Data vectors not used in this map can remain empty.
   */
  virtual void actuatorToJointPosition(const ActuatorData& act_data,
                                             JointData&    jnt_data) = 0;

  /**
   * \brief Transform \e effort variables from joint to actuator space.
   * \param[in]  jnt_data Joint-space variables.
   * \param[out] act_data Actuator-space variables.
   * \pre All non-empty vectors must contain valid data and their size should be consistent with the number of
   * transmission actuators and joints.
   * Data vectors not used in this map can remain empty.
   */
  virtual void jointToActuatorEffort(const JointData&    jnt_data,
                                           ActuatorData& act_data) = 0;

  /**
   * \brief Transform \e velocity variables from joint to actuator space.
   * \param[in]  jnt_data Joint-space variables.
   * \param[out] act_data Actuator-space variables.
   * \pre All non-empty vectors must contain valid data and their size should be consistent with the number of
   * transmission actuators and joints.
   * Data vectors not used in this map can remain empty.
   */
  virtual void jointToActuatorVelocity(const JointData&    jnt_data,
                                             ActuatorData& act_data) = 0;

  /**
   * \brief Transform \e position variables from joint to actuator space.
   * \param[in]  jnt_data Joint-space variables.
   * \param[out] act_data Actuator-space variables.
   * \pre All non-empty vectors must contain valid data and their size should be consistent with the number of
   * transmission actuators and joints.
   * Data vectors not used in this map can remain empty.
   */
  virtual void jointToActuatorPosition(const JointData&    jnt_data,
                                             ActuatorData& act_data) = 0;

  /** \return Number of actuators managed by transmission, ie. the dimension of the actuator space. */
  virtual std::size_t numActuators() const = 0;

  /** \return Number of joints managed by transmission, ie. the dimension of the joint space. */
  virtual std::size_t numJoints()    const = 0;
};

typedef boost::shared_ptr<Transmission> TransmissionSharedPtr;

} // transmission_interface

#endif // TRANSMISSION_INTERFACE_TRANSMISSION_H
