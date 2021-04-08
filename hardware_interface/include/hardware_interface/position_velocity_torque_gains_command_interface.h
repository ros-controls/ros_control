///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2021, University of Oxford
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

/// \author Wolfgang Merkt

#pragma once


#include <cassert>
#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/posvel_command_interface.h>

namespace hardware_interface
{

/** \brief A handle used to read and command a single joint. */
class PositionVelocityTorqueGainsJointHandle : public PosVelJointHandle
{
public:
  PositionVelocityTorqueGainsJointHandle() = default;

  /**
   * \param js This joint's state handle
   * \param cmd_pos A pointer to the storage for this joint's output command position
   * \param cmd_vel A pointer to the storage for this joint's output command velocity
   * \param cmd_eff A pointer to the storage for this joint's output command feed-forward torque
   * \param cmd_kp A pointer to the storage for this joint's position tracking gain
   * \param cmd_kd A pointer to the storage for this joint's velocity tracking gain
   */
  PositionVelocityTorqueGainsJointHandle(const JointStateHandle& js, double* cmd_pos, double* cmd_vel, double* cmd_eff, double* cmd_kp, double* cmd_kd)
    : PosVelJointHandle(js, cmd_pos, cmd_vel), cmd_eff_(cmd_eff), cmd_kp_(cmd_kp), cmd_kd_(cmd_kd)
  {
    if (!cmd_eff)
    {
      throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command effort data pointer is null.");
    }

    if (!cmd_kp)
    {
      throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command position gain data pointer is null.");
    }

    if (!cmd_kd)
    {
      throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command derivative gain data pointer is null.");
    }
  }

  void setCommand(double cmd_pos, double cmd_vel, double cmd_eff, double cmd_kp, double cmd_kd)
  {
    setCommandPosition(cmd_pos);
    setCommandVelocity(cmd_vel);
    setCommandEffort(cmd_eff);
    setCommandKp(cmd_kp);
    setCommandKd(cmd_kd);
  }

  void setCommandEffort(double cmd_eff) {assert(cmd_eff_); *cmd_eff_ = cmd_eff;}
  double getCommandEffort() const {assert(cmd_eff_); return *cmd_eff_;}

  void setCommandKp(double cmd_kp) {assert(cmd_kp_); *cmd_kp_ = cmd_kp;}
  double getCommandKp() const {assert(cmd_kp_); return *cmd_kp_;}

  void setCommandKd(double cmd_kd) {assert(cmd_kd_); *cmd_kd_ = cmd_kd;}
  double getCommandKd() const {assert(cmd_kd_); return *cmd_kd_;}

private:
  double* cmd_eff_ = {nullptr};
  double* cmd_kp_ = {nullptr};
  double* cmd_kd_ = {nullptr};
};


/** \brief Hardware interface to support commanding an array of joints.
 *
 * This \ref HardwareInterface supports commanding joints by position, velocity,
 * torque and PD gains together in one command.
 *
 * \note Getting a joint handle through the getHandle() method \e will claim that resource.
 *
 */
class PositionVelocityTorqueGainsJointInterface : public HardwareResourceManager<PositionVelocityTorqueGainsJointHandle, ClaimResources> {};

}
