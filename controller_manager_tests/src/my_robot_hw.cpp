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
//   * Neither the name of hiDOF Inc nor the names of its
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
  using namespace hardware_interface;

  // Initialize raw data
  joint_position_.resize(3);
  joint_velocity_.resize(3);
  joint_effort_.resize(3);
  joint_effort_command_.resize(3);
  joint_velocity_command_.resize(3);
  joint_name_.resize(3);

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

  joint_name_[2] = "hiDOF_joint3";
  joint_position_[2] = 1.0;
  joint_velocity_[2] = 0.0;
  joint_effort_[2] = 0.1;
  joint_effort_command_[2] = 0.0;
  joint_velocity_command_[2] = 0.0;

  // Populate hardware interfaces
  js_interface_.registerHandle(JointStateHandle(joint_name_[0], &joint_position_[0], &joint_velocity_[0], &joint_effort_[0]));
  js_interface_.registerHandle(JointStateHandle(joint_name_[1], &joint_position_[1], &joint_velocity_[1], &joint_effort_[1]));
  js_interface_.registerHandle(JointStateHandle(joint_name_[2], &joint_position_[2], &joint_velocity_[2], &joint_effort_[2]));

  ej_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[0]), &joint_effort_command_[0]));
  ej_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[1]), &joint_effort_command_[1]));
  ej_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[2]), &joint_effort_command_[2]));

  vj_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[0]), &joint_velocity_command_[0]));
  vj_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[1]), &joint_velocity_command_[1]));
  vj_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[2]), &joint_velocity_command_[2]));

  registerInterface(&js_interface_);
  registerInterface(&ej_interface_);
  registerInterface(&vj_interface_);
}


void MyRobotHW::read()
{

}

void MyRobotHW::write()
{
}

}
