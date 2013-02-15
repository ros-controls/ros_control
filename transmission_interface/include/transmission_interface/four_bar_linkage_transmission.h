
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

#ifndef TRANSMISSION_INTERFACE_FOUR_BAR_LINKAGE_TRANSMISSION_H
#define TRANSMISSION_INTERFACE_FOUR_BAR_LINKAGE_TRANSMISSION_H

#include <cassert>
#include <string>
#include <vector>

#include <transmission_interface/transmission.h>
#include <transmission_interface/transmission_exception.h>

namespace transmission_interface
{

class FourBarLinkageTransmission : public Transmission
{
public:
  FourBarLinkageTransmission(const std::vector<double>& actuator_reduction,
                             const std::vector<double>& joint_offset = std::vector<double>(2, 0.0));

  virtual ~FourBarLinkageTransmission() {}

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

  virtual std::size_t numActuators() const {return 2;}
  virtual std::size_t numJoints()    const {return 2;}

protected:
  std::vector<double>  actuator_reduction_;
  std::vector<double>  joint_offset_;
  std::vector<double*> joint_pos_vec_; ///< Workspace vector used to modify otherwise const parameters.
};

inline FourBarLinkageTransmission::FourBarLinkageTransmission(const std::vector<double>& actuator_reduction,
                                                              const std::vector<double>& joint_offset)
  : Transmission(),
    actuator_reduction_(actuator_reduction),
    joint_offset_(joint_offset),
    joint_pos_vec_(2)
{
  if (2 != actuator_reduction.size() ||
      2 != joint_offset_.size())
  {
    throw TransmissionException("Reduction and offset vectors of a four-bar linkage transmission must have size 2.");
  }
  if (0.0 == actuator_reduction[0] ||
      0.0 == actuator_reduction[1])
  {
    throw TransmissionException("Transmission reduction ratios cannot be zero.");
  }
}

inline void FourBarLinkageTransmission::actuatorToJointEffort(const std::vector<double*> actuator_eff,
                                                                    std::vector<double*> joint_eff)
{
  assert(2 == actuator_eff.size() && 2 == joint_eff.size());
  std::vector<double>& ar = actuator_reduction_;
  *joint_eff[0] = *actuator_eff[0] * ar[0];
  *joint_eff[1] = *actuator_eff[1] * ar[1] - *actuator_eff[0] * ar[0];
}

inline void FourBarLinkageTransmission::actuatorToJointVelocity(const std::vector<double*> actuator_vel,
                                                                      std::vector<double*> joint_vel)
{
  assert(2 == actuator_vel.size() && 2 == joint_vel.size());
  std::vector<double>& ar = actuator_reduction_;
  *joint_vel[0] = *actuator_vel[0] / ar[0];
  *joint_vel[1] = *actuator_vel[1] / ar[1] - *actuator_vel[0] / ar[0];
}

inline void FourBarLinkageTransmission::actuatorToJointPosition(const std::vector<double*> actuator_pos,
                                                                      std::vector<double*> joint_pos)
{
  assert(2 == actuator_pos.size() && 2 == joint_pos.size());
  actuatorToJointVelocity(actuator_pos, joint_pos); // Apply flow map...
  *joint_pos[0] += joint_offset_[0];                // ...and add integration constant
  *joint_pos[1] += joint_offset_[1];                // ...to each joint
}

inline void FourBarLinkageTransmission::jointToActuatorEffort(const std::vector<double*> joint_eff,
                                                                    std::vector<double*> actuator_eff)
{
  assert(2 == actuator_eff.size() && 2 == joint_eff.size());
  std::vector<double>& ar = actuator_reduction_;
  *actuator_eff[0] = *joint_eff[0] / ar[0];
  *actuator_eff[1] = (*joint_eff[0] + *joint_eff[1]) / ar[1];
}

inline void FourBarLinkageTransmission::jointToActuatorVelocity(const std::vector<double*> joint_vel,
                                                                      std::vector<double*> actuator_vel)
{
  assert(2 == actuator_vel.size() && 2 == joint_vel.size());
  std::vector<double>& ar = actuator_reduction_;
  *actuator_vel[0] = *joint_vel[0] * ar[0];
  *actuator_vel[1] = (*joint_vel[0] + *joint_vel[1]) * ar[1];
}

inline void FourBarLinkageTransmission::jointToActuatorPosition(const std::vector<double*> joint_pos,
                                                                      std::vector<double*> actuator_pos)
{
  assert(2 == actuator_pos.size() && 2 == joint_pos.size());
  double joint_pos_with_offset[2] = {*joint_pos[0] - joint_offset_[0],
                                     *joint_pos[1] - joint_offset_[1]};
  joint_pos_vec_[0] = &joint_pos_with_offset[0];         // Remove integration constant in workspace vector...
  joint_pos_vec_[1] = &joint_pos_with_offset[1];         // ...from each joint
  jointToActuatorVelocity(joint_pos_vec_, actuator_pos); // ...and apply flow map to _workspace_ vector
}

} // transmission_interface

#endif // TRANSMISSION_INTERFACE_FOUR_BAR_LINKAGE_TRANSMISSION_H
