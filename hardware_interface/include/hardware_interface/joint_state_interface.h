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

#ifndef HARDWARE_INTERFACE_JOINT_STATE_INTERFACE_H
#define HARDWARE_INTERFACE_JOINT_STATE_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <cassert>
#include <string>

namespace hardware_interface
{

/** A handle used to read the state of a single joint. */
class JointStateHandle
{
public:
  JointStateHandle() : name_(), pos_(0), vel_(0), eff_(0) {}

  /**
   * \param name The name of the joint
   * \param pos A pointer to the storage for this joint's position
   * \param vel A pointer to the storage for this joint's velocity
   * \param eff A pointer to the storage for this joint's effort (force or torque)
   */
  JointStateHandle(const std::string& name, const double* pos, const double* vel, const double* eff)
    : name_(name), pos_(pos), vel_(vel), eff_(eff)
  {
    if (!pos)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. Position data pointer is null.");
    }
    if (!vel)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. Velocity data pointer is null.");
    }
    if (!eff)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. Effort data pointer is null.");
    }
  }

  std::string getName() const {return name_;}
  double getPosition()  const {assert(pos_); return *pos_;}
  double getVelocity()  const {assert(vel_); return *vel_;}
  double getEffort()    const {assert(eff_); return *eff_;}

private:
  std::string name_;
  const double* pos_;
  const double* vel_;
  const double* eff_;
};

/** \brief Hardware interface to support reading the state of an array of joints
 *
 * This \ref HardwareInterface supports reading the state of an array of named
 * joints, each of which has some position, velocity, and effort (force or
 * torque).
 *
 */
class JointStateInterface : public HardwareResourceManager<JointStateHandle> {};

}

#endif
