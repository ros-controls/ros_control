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

#ifndef TRANSMISSION_INTERFACE_SIMPLE_TRANSMISSION_H
#define TRANSMISSION_INTERFACE_SIMPLE_TRANSMISSION_H

#include <cassert>
#include <string>
#include <vector>

#include <transmission_interface/transmission.h>
#include <transmission_interface/transmission_exception.h>

namespace transmission_interface
{

/**
 * \brief Implementation of a simple reducer transmission.
 *
 * The following figure illustrates the transmission and the expressions that govern it.
 *
 * TODO: Insert figure
 *
 * - The transmission couples one actuator to one joint.
 * - The transmission ratio, or reduction can take any real value \e except zero. In particular:
 *     - If its absolute value is greater than one, it's a velocity reducer, while if its absolute value lies in
 *       \f$ (0, 1) \f$ it's a velocity amplifier.
 *     - Negative values represent a direction flip, ie. actuator and joint move in opposite directions.
 */
class SimpleTransmission : public Transmission
{
public:
  /**
   * \param reduction Reduction ratio.
   * \param joint_offset Joint position offset used in the position mappings.
   * \pre Nonzero reduction value.
   */
  SimpleTransmission(const double reduction,
                     const double joint_offset = 0.0);

  virtual ~SimpleTransmission() {}

  virtual void actuatorToJointEffort(const std::vector<double*> actuator_eff,
                                           std::vector<double*> joint_eff);

  virtual void actuatorToJointVelocity(const std::vector<double*> actuator_vel,
                                             std::vector<double*> joint_vel);

  virtual void actuatorToJointPosition(const std::vector<double*> actuator_pos,
                                             std::vector<double*> joint_pos);

  virtual void jointToActuatorEffort(const std::vector<double*> joint_eff,
                                           std::vector<double*> actuator_eff);

  virtual void jointToActuatorVelocity(const std::vector<double*> joint_vel,
                                             std::vector<double*> actuator_vel);

  virtual void jointToActuatorPosition(const std::vector<double*> joint_pos,
                                             std::vector<double*> actuator_pos);

  virtual std::size_t numActuators() const {return 1;}
  virtual std::size_t numJoints()    const {return 1;}

protected:
  double reduction_;
  double joint_offset_;
  std::vector<double*> joint_pos_vec_; ///< Workspace vector used to modify otherwise const parameters.
};

inline SimpleTransmission::SimpleTransmission(const double reduction,
                                              const double joint_offset)
  : Transmission(),
    reduction_(reduction),
    joint_offset_(joint_offset),
    joint_pos_vec_(std::vector<double*>(1))
{
  if (0.0 == reduction_)
  {
    throw TransmissionException("Transmission reduction ratio cannot be zero.");
  }
}

inline void SimpleTransmission::actuatorToJointEffort(const std::vector<double*> actuator_eff,
                                                            std::vector<double*> joint_eff)
{
  assert(1 == actuator_eff.size() && 1 == joint_eff.size());
  *joint_eff[0] = *actuator_eff[0] * reduction_;
}

inline void SimpleTransmission::actuatorToJointVelocity(const std::vector<double*> actuator_vel,
                                                              std::vector<double*> joint_vel)
{
  assert(1 == actuator_vel.size() && 1 == joint_vel.size());
  *joint_vel[0] = *actuator_vel[0] / reduction_;
}

inline void SimpleTransmission::actuatorToJointPosition(const std::vector<double*> actuator_pos,
                                                              std::vector<double*> joint_pos)
{
  assert(1 == actuator_pos.size() && 1 == joint_pos.size());
  actuatorToJointVelocity(actuator_pos, joint_pos); // Apply flow map...
  *joint_pos[0] += joint_offset_;                   // ...and add integration constant
}

inline void SimpleTransmission::jointToActuatorEffort(const std::vector<double*> joint_eff,
                                                            std::vector<double*> actuator_eff)
{
  assert(1 == actuator_eff.size() && 1 == joint_eff.size());
  *actuator_eff[0] = *joint_eff[0] / reduction_;
}

inline void SimpleTransmission::jointToActuatorVelocity(const std::vector<double*> joint_vel,
                                                              std::vector<double*> actuator_vel)
{
  assert(1 == actuator_vel.size() && 1 == joint_vel.size());
  *actuator_vel[0] = *joint_vel[0] * reduction_;
}

inline void SimpleTransmission::jointToActuatorPosition(const std::vector<double*> joint_pos,
                                                              std::vector<double*> actuator_pos)
{
  assert(1 == actuator_pos.size() && 1 == joint_pos.size());
  double joint_pos_with_offset = *joint_pos[0] - joint_offset_;
  joint_pos_vec_[0] = &joint_pos_with_offset;            // Remove integration constant in workspace vector...
  jointToActuatorVelocity(joint_pos_vec_, actuator_pos); // ...and apply flow map to _workspace_ vector
}

} // transmission_interface

#endif // TRANSMISSION_INTERFACE_SIMPLE_TRANSMISSION_H