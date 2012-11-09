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


#include <controller_manager_tests/my_robot_hw.h>

namespace controller_manager_tests
{

  MyRobotHW::MyRobotHW()
  {
    joint_position_.resize(2);
    joint_velocity_.resize(2);
    joint_effort_.resize(2);
    joint_effort_command_.resize(2);
    joint_velocity_command_.resize(2);
    joint_name_.resize(2);

    joint_name_[0] = "hiDOF_joint1";
    joint_position_[0] = 1.0;
    joint_velocity_[0] = 0.0;
    joint_effort_[0] = 0.1;
    joint_effort_command_[0] = 0.0;
    joint_velocity_command_[0] = 0.0;

    joint_name_[1] = "hiDOF_joint2";
    joint_position_[1] = 1.0;
    joint_velocity_[1] = 0.0;
    joint_effort_[1] = 0.1;
    joint_effort_command_[1] = 0.0;
    joint_velocity_command_[1] = 0.0;
  }



  const std::vector<std::string>& MyRobotHW::getJointNames() const
  {
    return joint_name_;
  }


  double* MyRobotHW::getEffortCommand(const std::string& name)
  {
    for (unsigned i=0; i<joint_name_.size(); i++)
      if (joint_name_[i] == name)
        return &joint_effort_command_[i];

    throw hardware_interface::HardwareInterfaceException("Could not find joint "+name+" in Dummy hardware interface");
  }

  double* MyRobotHW::getVelocityCommand(const std::string& name)
  {
    for (unsigned i=0; i<joint_name_.size(); i++)
      if (joint_name_[i] == name)
        return &joint_velocity_command_[i];

    throw hardware_interface::HardwareInterfaceException("Could not find joint "+name+" in Dummy hardware interface");
  }

  const double* MyRobotHW::getPosition(const std::string& name) const
  {
    for (unsigned i=0; i<joint_name_.size(); i++)
      if (joint_name_[i] == name)
        return &joint_position_[i];

    throw hardware_interface::HardwareInterfaceException("Could not find joint "+name+" in Dummy hardware interface");
  }


  const double* MyRobotHW::getVelocity(const std::string& name) const
  {
    for (unsigned i=0; i<joint_name_.size(); i++)
      if (joint_name_[i] == name)
        return &joint_velocity_[i];

    throw hardware_interface::HardwareInterfaceException("Could not find joint "+name+" in Dummy hardware interface");
  }


  const double* MyRobotHW::getEffort(const std::string& name) const
  {
    for (unsigned i=0; i<joint_name_.size(); i++)
      if (joint_name_[i] == name)
        return &joint_effort_[i];

    throw hardware_interface::HardwareInterfaceException("Could not find joint "+name+" in Dummy hardware interface");
  }


  void MyRobotHW::read()
  {

  }

  void MyRobotHW::write()
  {
  }

}
