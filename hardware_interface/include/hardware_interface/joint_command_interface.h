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

/*
 * Author: Wim Meeussen
 */

#ifndef HARDWARE_INTERFACE_JOINT_COMMAND_INTERFACE_H
#define HARDWARE_INTERFACE_JOINT_COMMAND_INTERFACE_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/resource.h>

namespace hardware_interface{


class EffortJointHandle : public JointStateHandle
{
public:
  EffortJointHandle(const JointStateHandle& js, double* cmd)
    : JointStateHandle(js), cmd_(cmd)
  {}
  void setEffortCommand(double command) {*cmd_ = command;};

private:
  double* cmd_;
};


class EffortJointInterface: virtual public JointStateInterface
{
public:
  EffortJointInterface()
  {
    registerType(typeid(EffortJointInterface).name());
  }

  // get the joint to command
  EffortJointHandle getEffortJointHandle(const std::string& name)
  {
    claim(Resource("Joint", name));
    double* cmd = getEffortCommand(name);
    if (!cmd)
      throw HardwareInterfaceException("Failed to construct JointEffortCommand for joint [" + name + "]");
    return EffortJointHandle(getJointStateHandle(name), cmd);
  }

protected:
  // Virtual function to give access to command for joint
  virtual double* getEffortCommand(const std::string& name) = 0;
};







class VelocityJointHandle : public JointStateHandle
{
public:
  VelocityJointHandle(const JointStateHandle& js, double* cmd)
    : JointStateHandle(js), cmd_(cmd)
  {}

  void setVelocityCommand(double command) {*cmd_ = command;};

private:
  double* cmd_;
};


class VelocityJointInterface: virtual public JointStateInterface
{
public:
  VelocityJointInterface()
  {
    registerType(typeid(VelocityJointInterface).name());
  }

  // get the joint to command
  VelocityJointHandle getVelocityJointHandle(const std::string& name)
  {
    claim(Resource("Joint", name));
    double* cmd = getVelocityCommand(name);
    if (!cmd)
      throw HardwareInterfaceException("Failed to construct JointVelocityCommand for joint [" + name + "]");
    return VelocityJointHandle(getJointStateHandle(name), cmd);
  }

protected:
  // Virtual function to give access to command for joint
  virtual double* getVelocityCommand(const std::string& name) = 0;
};








class PositionJointHandle : public JointStateHandle
{
public:
  PositionJointHandle(const JointStateHandle& js, double* cmd)
    : JointStateHandle(js), cmd_(cmd)
  {}

  void setPositionCommand(double command) {*cmd_ = command;};

private:
  double* cmd_;
};


class PositionJointInterface: virtual public JointStateInterface
{
public:
  PositionJointInterface()
  {
    registerType(typeid(PositionJointInterface).name());
  }

  // get the joint to command
  PositionJointHandle getPositionJointHandle(const std::string& name)
  {
    claim(Resource("Joint", name));
    double* cmd = getPositionCommand(name);
    if (!cmd)
      throw HardwareInterfaceException("Failed to construct JointPositionCommand for joint [" + name + "]");
    return PositionJointHandle(getJointStateHandle(name), cmd);
  }

protected:
  // Virtual function to give access to command for joint
  virtual double* getPositionCommand(const std::string& name) = 0;
};




}

#endif
