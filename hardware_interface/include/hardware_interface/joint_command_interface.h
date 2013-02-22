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

#ifndef HARDWARE_INTERFACE_JOINT_COMMAND_INTERFACE_H
#define HARDWARE_INTERFACE_JOINT_COMMAND_INTERFACE_H

#include <hardware_interface/joint_state_interface.h>


namespace hardware_interface
{

/** \brief A handle used to read and command a single joint
 */
class JointHandle : public JointStateHandle
{
public:
  JointHandle() {};
  JointHandle(const JointStateHandle& js, double* cmd)
    : JointStateHandle(js), cmd_(cmd)
  {}
  void setCommand(double command) {*cmd_ = command;};

private:
  double* cmd_;
};


/** \brief Hardware interface to support commanding an array of joints
 *
 * This \ref HardwareInterface supports commanding the output of an array of
 * named joints. Note that these commands can have any semantic meaning as long
 * as they each can be represented by a single double, they are not necessarily
 * effort commands. To specify a meaning to this command, see the derived
 * classes like \ref EffortJointInterface etc.
 *
 */
class JointCommandInterface : public hardware_interface::HardwareInterface
{
public:
  /// Get the vector of joint names registered to this interface.
  std::vector<std::string> getJointNames() const
  {
    std::vector<std::string> out;
    out.reserve(handle_map_.size());
    for( HandleMap::const_iterator it = handle_map_.begin(); it != handle_map_.end(); ++it)
    {
      out.push_back(it->first);
    }
    return out;
  }

  /** \brief Register a new joint with this interface.
   *
   * \param name The name of the new joint
   * \param cmd A pointer to the storage for this joint's output command
   */
  void registerJoint(const JointStateHandle& js, double* cmd)
  {
    JointHandle handle(js, cmd);
    HandleMap::iterator it = handle_map_.find(js.getName());
    if (it == handle_map_.end())
      handle_map_.insert(std::make_pair(js.getName(), handle));
    else
      it->second = handle;
  }

  /** \brief Get a \ref JointHandle for accessing a joint's state and setting
   * its output command.
   *
   * When a \ref JointHandle is acquired, this interface will claim the joint
   * as a resource.
   *
   * \param name The name of the joint
   *
   * \returns A \ref JointHandle corresponding to the joint identified by \c name
   *
   */
  JointHandle getJointHandle(const std::string& name)
  {
    HandleMap::const_iterator it = handle_map_.find(name);

    if (it == handle_map_.end())
      throw HardwareInterfaceException("Could not find joint [" + name + "] in JointCommandInterface");

    HardwareInterface::claim(name);
    return it->second;
  }

protected:
  typedef std::map<std::string, JointHandle> HandleMap;
  HandleMap handle_map_;
};

/// \ref JointCommandInterface for commanding effort-based joints
class EffortJointInterface : public JointCommandInterface
{

};

/// \ref JointCommandInterface for commanding velocity-based joints
class VelocityJointInterface : public JointCommandInterface
{

};

/// \ref JointCommandInterface for commanding position-based joints
class PositionJointInterface : public JointCommandInterface
{

};


}

#endif
