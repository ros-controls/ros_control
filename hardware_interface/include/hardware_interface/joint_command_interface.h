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
//   * Neither the name of Stanford University nor the names of its
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

#include "hardware_interface/joint_state_interface.h"


namespace hardware_interface{


class JointCommand
{
public:
  JointCommand(const JointState& js, double& cmd)
    : js_(js), cmd_(cmd)
  {}

  double getPosition() const {return js_.getPosition();}
  double getVelocity() const {return js_.getVelocity();}
  double getEffort()   const {return js_.getEffort();}
  double setCommand(double command) {cmd = command};

private:
  const JointState js_;
  double& cmd_;
};



class JointCommandInterface: public class JointStateInterface
{
public:
  // get the joint to command
  JointCommand& getJoint(const std::string& name)
  {
    return JointCommand(getJointState(name), getJointCommand(name));    
  }


protected:  
  // Virtual function to give access to command for joint
  virtual double& getJointCommand(const std::string& name) = 0;
  
};


class JointEffortInterface: public class JointCommandInterface {};
class JointPositionInterface: public class JointCommandInterface {};
class JointVelocityInterface: public class JointCommandInterface {};

}

#endif
