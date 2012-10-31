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


#include "hardware_interface/joint_command_interface.h"


namespace hardware_interface_dummy
{


class DummyJointCommand: public JointEffortInterface
{
  DummyJointCommand()
  {
    joint_position_.push_back(1);
    joint_velocity_.push_back(2);
    joint_effort_.push_back(3);
    joint_name_.push_back("hiDOF_joint1");

    joint_position_.push_back(4);
    joint_velocity_.push_back(5);
    joint_effort_.push_back(6);
    joint_name_.push_back("hiDOF_joint2");
  }



  virtual std::vector<std::string>& getJointNames() const
  {
    return joint_name_;
  }


  virtual double& getJointCommand(const std::string& name)
  {
    for (unsigned i=0; i<joint_name_.size(); i++)
      if (joint_name_[i] == name)
        return joint_command_[i];

    throw HardwareInterfaceException("Could not find joint "+name+" in Dummy hardware interface");
    return 0;
  }

  
  virtual double& getJointPosition(const std::string& name)
  {
    for (unsigned i=0; i<joint_name_.size(); i++)
      if (joint_name_[i] == name)
        return joint_position_[i];

    throw HardwareInterfaceException("Could not find joint "+name+" in Dummy hardware interface");
    return 0;
  }


  virtual double& getJointVelocity(const std::string& name)
  {
    for (unsigned i=0; i<joint_name_.size(); i++)
      if (joint_name_[i] == name)
        return joint_velocity_[i];

    throw HardwareInterfaceException("Could not find joint "+name+" in Dummy hardware interface");
    return 0;
  }


  virtual double& getJointEffort(const std::string& name)
  {
    for (unsigned i=0; i<joint_name_.size(); i++)
      if (joint_name_[i] == name)
        return joint_effort_[i];

    throw HardwareInterfaceException("Could not find joint "+name+" in Dummy hardware interface");
    return 0;
  }


  void read()
  {
    for (unsigned i=0; i<joint_position_.size(); i++)
    {
      joint_effort_[i] = joint_command_[i];
      joint_velocity_[i] += joint_effort_[i]*0.001;
      joint_position_[i] += 0.01;
      joint_position_[i] += 0.01;
    }
  }

  void write()
  {
  }


private:
  std::vector<double> joint_command_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<std::string> joint_name_;
};
}
