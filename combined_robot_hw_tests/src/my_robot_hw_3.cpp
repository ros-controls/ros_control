///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2015, Shadow Robot Company Ltd.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Shadow Robot Company Ltd. nor the names of its
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


#include <combined_robot_hw_tests/my_robot_hw_3.h>

namespace combined_robot_hw_tests
{

bool MyRobotHW3::init(ros::NodeHandle& /*root_nh*/, ros::NodeHandle &robot_hw_nh)
{
  using namespace hardware_interface;

  std::string robot_description;
  if (!robot_hw_nh.getParam("robot_description", robot_description)) {return false;}

  // E.g. Read URDF and parse joint names.
  // Then read joint_name_filter param to know what joints belong to this RobotHW

  std::vector<std::string> joints;
  joints.push_back("right_arm_joint_1");

  // Initialize raw data
  size_t nb_joints = joints.size();
  joint_position_.resize(nb_joints);
  joint_velocity_.resize(nb_joints);
  joint_effort_.resize(nb_joints);
  joint_effort_command_.resize(nb_joints);
  joint_velocity_command_.resize(nb_joints);
  joint_name_.resize(nb_joints);

  for (size_t i = 0; i < nb_joints; i++)
  {
    joint_name_[i] = joints[i];
    joint_position_[i] = 1.0;
    joint_velocity_[i] = 0.0;
    joint_effort_[i] = 0.1;
    joint_effort_command_[i] = 1.5;
    joint_velocity_command_[i] = 0.0;
    // Populate hardware interfaces
    js_interface_.registerHandle(JointStateHandle(joint_name_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]));
    ej_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[i]), &joint_effort_command_[i]));
    vj_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[i]), &joint_velocity_command_[i]));
  }

  registerInterface(&js_interface_);
  registerInterface(&ej_interface_);
  registerInterface(&vj_interface_);

  return true;
}


void MyRobotHW3::read(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{

}

void MyRobotHW3::write(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
}

}

PLUGINLIB_EXPORT_CLASS( combined_robot_hw_tests::MyRobotHW3, hardware_interface::RobotHW)
