///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012, hiDOF INC.
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

/// \author Igor Kalevatykh

#ifndef HARDWARE_INTERFACE_POSVEL_COMMAND_INTERFACE_H
#define HARDWARE_INTERFACE_POSVEL_COMMAND_INTERFACE_H

#include <cassert>
#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_state_interface.h>

namespace hardware_interface
{

/** \brief A handle used to read and command a single joint. */
class PosVelJointHandle : public JointStateHandle
{
public:
  PosVelJointHandle() : JointStateHandle(), cmd_pos_(0), cmd_vel_(0) {}

  /**
   * \param js This joint's state handle
   * \param cmd_pos A pointer to the storage for this joint's output command position
   * \param cmd_vel A pointer to the storage for this joint's output command velocity
   */
  PosVelJointHandle(const JointStateHandle& js, double* cmd_pos, double* cmd_vel)
    : JointStateHandle(js), cmd_pos_(cmd_pos), cmd_vel_(cmd_vel)
  {
    if (!cmd_pos)
    {
      throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command position pointer is null.");
    }
    if (!cmd_vel)
    {
      throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command velocity  pointer is null.");
    }
  }

  void setCommand(double cmd_pos, double cmd_vel)
  {
    setCommandPosition(cmd_pos);
    setCommandVelocity(cmd_vel);
  }

  void setCommandPosition(double cmd_pos)     {assert(cmd_pos_); *cmd_pos_ = cmd_pos;}
  void setCommandVelocity(double cmd_vel)     {assert(cmd_vel_); *cmd_vel_ = cmd_vel;}

  double getCommandPosition()     const {assert(cmd_pos_); return *cmd_pos_;}
  double getCommandVelocity()     const {assert(cmd_vel_); return *cmd_vel_;}

private:
  double* cmd_pos_;
  double* cmd_vel_;
};

/** \brief Hardware interface to support commanding an array of joints.
 *
 * This \ref HardwareInterface supports commanding joints by position, velocity
 * together in one command.
 *
 * \note Getting a joint handle through the getHandle() method \e will claim that resource.
 *
 */
class PosVelJointInterface : public HardwareResourceManager<PosVelJointHandle, ClaimResources> {};

}

#endif /*HARDWARE_INTERFACE_POSVEL_COMMAND_INTERFACE_H*/
