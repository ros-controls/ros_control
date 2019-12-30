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

/// \author Wim Meeussen

#pragma once


#include <cassert>
#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_state_interface.h>

namespace hardware_interface
{

/** \brief A handle used to read and command a single joint. */
class JointHandle : public JointStateHandle
{
public:
  JointHandle() = default;

  /**
   * \param js This joint's state handle
   * \param cmd A pointer to the storage for this joint's output command
   */
  JointHandle(const JointStateHandle& js, double* cmd)
    : JointStateHandle(js), cmd_(cmd)
  {
    if (!cmd_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command data pointer is null.");
    }
  }

  void setCommand(double command) {assert(cmd_); *cmd_ = command;}
  double getCommand() const {assert(cmd_); return *cmd_;}
  const double* getCommandPtr() const {assert(cmd_); return cmd_;}

private:
  double* cmd_ = {nullptr};
};

/** \brief Hardware interface to support commanding an array of joints.
 *
 * This \ref HardwareInterface supports commanding the output of an array of
 * named joints. Note that these commands can have any semantic meaning as long
 * as they each can be represented by a single double, they are not necessarily
 * effort commands. To specify a meaning to this command, see the derived
 * classes like \ref EffortJointInterface etc.
 *
 * \note Getting a joint handle through the getHandle() method \e will claim that resource.
 *
 */
class JointCommandInterface : public HardwareResourceManager<JointHandle, ClaimResources> {};

/// \ref JointCommandInterface for commanding effort-based joints.
class EffortJointInterface : public JointCommandInterface {};

/// \ref JointCommandInterface for commanding velocity-based joints.
class VelocityJointInterface : public JointCommandInterface {};

/// \ref JointCommandInterface for commanding position-based joints.
class PositionJointInterface : public JointCommandInterface {};

}
