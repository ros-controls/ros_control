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


#include <controller_manager_tests/effort_test_controller.h>

using namespace controller_manager_tests;

bool EffortTestController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& n)
{
  // get all joint states from the hardware interface
  // const std::vector<std::string>& joint_names = hw->getJointNames();
  // for (unsigned i=0; i<joint_names.size(); i++)
  //  ROS_INFO("Got joint %s", joint_names[i].c_str());
  std::vector<std::string> joint_names;

  if (!n.getParam("joints", joint_names))
  {
    joint_names.push_back("hiDOF_joint1");
    joint_names.push_back("hiDOF_joint2");
  }

  for (const auto& joint_name : joint_names)
    joint_effort_commands_.push_back(hw->getHandle(joint_name));

  return true;
}

void EffortTestController::starting(const ros::Time& /*time*/)
{
  ROS_INFO("Starting JointState Controller");
}

void EffortTestController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{}

void EffortTestController::stopping(const ros::Time& /*time*/)
{
  ROS_INFO("Stopping JointState Controller");
}

PLUGINLIB_EXPORT_CLASS( controller_manager_tests::EffortTestController, controller_interface::ControllerBase)
