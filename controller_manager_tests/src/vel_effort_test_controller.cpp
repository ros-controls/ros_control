// Author: Kelsey Hawkins
// Based on code by:
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


#include <controller_manager_tests/vel_effort_test_controller.h>

using namespace controller_manager_tests;

bool VelocityEffortTestController::init(hardware_interface::VelocityJointInterface* hw_vel, 
                                        hardware_interface::EffortJointInterface* hw_eff, 
                                        ros::NodeHandle &n)
{
  // get all joint states from the hardware interface
  // const std::vector<std::string>& all_vel_joint_names = hw_vel->getNames();
  // for (unsigned i=0; i<all_vel_joint_names.size(); i++)
  //   ROS_INFO("Got vel joint %s", all_vel_joint_names[i].c_str());
  // const std::vector<std::string>& all_eff_joint_names = hw_eff->getNames();
  // for (unsigned i=0; i<all_eff_joint_names.size(); i++)
  //   ROS_INFO("Got eff joint %s", all_eff_joint_names[i].c_str());

  std::vector<std::string> velocity_joint_names;
  velocity_joint_names.push_back("vel_joint1");
  velocity_joint_names.push_back("vel_joint2");
  velocity_joint_names.push_back("vel_derived_joint1");
  velocity_joint_names.push_back("vel_derived_joint2");

  std::vector<std::string> effort_joint_names;
  effort_joint_names.push_back("eff_joint1");
  effort_joint_names.push_back("eff_joint2");
  effort_joint_names.push_back("eff_derived_joint1");
  effort_joint_names.push_back("eff_derived_joint2");

  for (unsigned i=0; i<4; i++) {
    joint_velocity_commands_.push_back(hw_vel->getHandle(velocity_joint_names[i]));
    joint_effort_commands_.push_back(hw_eff->getHandle(effort_joint_names[i]));
  }

  return true;
}

void VelocityEffortTestController::starting(const ros::Time& time)
{
  ROS_INFO("Starting VelocityEffortTestController Controller");
}

void VelocityEffortTestController::update(const ros::Time& time, const ros::Duration& period)
{
  for (unsigned int i=0; i < joint_effort_commands_.size(); i++)
  {

  }
}

void VelocityEffortTestController::stopping(const ros::Time& time)
{
  ROS_INFO("Stopping VelocityEffortTestController Controller");
}

PLUGINLIB_EXPORT_CLASS( controller_manager_tests::VelocityEffortTestController, controller_interface::ControllerBase)
