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
//   * Neither the name of hiDOF, Inc. nor the names of its
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

namespace transmission_interface
{

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
 * positon. Position is not a power variable, but the mappings can be implemented using the velocity map plus an
 * integration constant representing the offset between actuator and joint zeros.
 *
 * \par Credit
 * This interface was inspired by similar existing implemenations by PAL Robotics, S.L. and Willow Garage Inc.
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
   * \param[in]  actuator_eff Vector of \e pointers to actuator-space effort variables.
   * \param[out] joint_eff    Vector of \e pointers to joint-space effort variables.
   * \pre All pointers must be valid, and the parameter dimensions must be consistent with those of the
   * actuator and joint spaces.
   */
  virtual void actuatorToJointEffort(const std::vector<double*> actuator_eff,
                                           std::vector<double*> joint_eff) = 0;

  /**
   * \brief Transform \e velocity variables from actuator to joint space.
   * \param[in]  actuator_vel Vector of \e pointers to actuator-space velocity variables.
   * \param[out] joint_vel    Vector of \e pointers to joint-space velocity variables.
   * \pre All pointers must be valid, and the parameter dimensions must be consistent with those of the
   * actuator and joint spaces.
   */
  virtual void actuatorToJointVelocity(const std::vector<double*> actuator_vel,
                                             std::vector<double*> joint_vel) = 0;

  /**
   * \brief Transform \e position variables from actuator to joint space.
   * \param[in]  actuator_pos Vector of \e pointers to actuator-space position variables.
   * \param[out] joint_pos    Vector of \e pointers to joint-space position variables.
   * \pre All pointers must be valid, and the parameter dimensions must be consistent with those of the
   * actuator and joint spaces.
   */
  virtual void actuatorToJointPosition(const std::vector<double*> actuator_pos,
                                             std::vector<double*> joint_pos) = 0;

  /**
   * \brief Transform \e effort variables from joint to actuator space.
   * \param[in]  joint_eff    Vector of \e pointers to joint-space effort variables.
   * \param[out] actuator_eff Vector of \e pointers to actuator-space effort variables.
   * \pre All pointers must be valid, and the parameter dimensions must be consistent with those of the
   * actuator and joint spaces.
   */
  virtual void jointToActuatorEffort(const std::vector<double*> joint_eff,
                                           std::vector<double*> actuator_eff) = 0;

  /**
   * \brief Transform \e velocity variables from joint to actuator space.
   * \param[in]  joint_vel    Vector of \e pointers to joint-space velocity variables.
   * \param[out] actuator_vel Vector of \e pointers to actuator-space velocity variables.
   * \pre All pointers must be valid, and the parameter dimensions must be consistent with those of the
   * actuator and joint spaces.
   */
  virtual void jointToActuatorVelocity(const std::vector<double*> joint_vel,
                                             std::vector<double*> actuator_vel) = 0;

  /**
   * \brief Transform \e position variables from joint to actuator space.
   * \param[in]  joint_pos    Vector of \e pointers to joint-space position variables.
   * \param[out] actuator_pos Vector of \e pointers to actuator-space position variables.
   * \pre All pointers must be valid, and the parameter dimensions must be consistent with those of the
   * actuator and joint spaces.
   */
  virtual void jointToActuatorPosition(const std::vector<double*> joint_pos,
                                             std::vector<double*> actuator_pos) = 0;

  /** \return Number of actuators managed by transmission, ie. the dimension of the actuator space. */
  virtual std::size_t numActuators() const = 0;

  /** \return Number of joints managed by transmission, ie. the dimension of the joint space. */
  virtual std::size_t numJoints()    const = 0;
};

} // transmission_interface

#endif // TRANSMISSION_INTERFACE_TRANSMISSION_H
