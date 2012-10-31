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

#ifndef HARDWARE_INTERFACE_JOINT_STATE_INTERFACE_H
#define HARDWARE_INTERFACE_JOINT_STATE_INTERFACE_H

#include "hardware_interface/hardware_interface.h"


namespace hardware_interface{

class JointState
{
public:
  JointState(const std::string name, double& pos, double& vel, double& eff)
    : name_(name), pos_(pos), vel_(vel), eff_(eff)
  {}

  double getName() const {return name_;}
  double getPosition() const {return pos_;}
  double getVelocity() const {return vel_;}
  double getEffort()   const {return eff_;}


private:
  std::string name_;
  double& pos_;
  double& vel_;
  double& eff_;
};



class JointStateInterface: public class HardwareInterface
{
public:
  const JointState getJointState(const std::string& name) const
  {
    return JointState(name, 
                      getJointPosition(name),
                      getJointVelocity(name),
                      getJointEffort(name));
  }

  virtual const std::vector<std::string>& getJointNames() const = 0;

protected:
  virtual double& getJointPosition(const std::string& name) = 0;
  virtual double& getJointVelocity(const std::string& name) = 0;
  virtual double& getJointEffort(const std::string& name) = 0;

};

}

#endif
