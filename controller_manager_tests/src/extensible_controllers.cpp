///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2017, Clearpath Robotics Inc.
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

#include <controller_manager_tests/extensible_controllers.h>
#include <pluginlib/class_list_macros.hpp>

using namespace controller_manager_tests;

bool ExtensibleController::init(hardware_interface::RobotHW* robot_hw,
          ros::NodeHandle& /*root_nh*/, ros::NodeHandle& controller_nh)
{
  std::string vel_joint_name;
  controller_nh.getParam("velocity_joint", vel_joint_name);

  hardware_interface::VelocityJointInterface* joint_iface = robot_hw->get<hardware_interface::VelocityJointInterface>();
  joint_iface->getHandle(vel_joint_name);
  return true;
}

int ExtensibleController::helper()
{
  return 1;
}

void ExtensibleController::update(const ros::Time&, const ros::Duration&)
{
  // TODO: Publish this return value so that the test can validate it?
  int foo = helper();
}


bool DerivedController::initRequest(hardware_interface::RobotHW* hw, ros::NodeHandle& nh, ros::NodeHandle& pnh,
    controller_interface::ControllerBase::ClaimedResources& cr)
{
  return DerivedControllerInterface::initRequest(hw, nh, pnh, cr);
}


bool DerivedController::init(hardware_interface::RobotHW* robot_hw,
                             ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  // First initialize the base controller.
  if (!ExtensibleController::init(robot_hw, root_nh, controller_nh))
  {
    return false;
  }

  std::string eff_joint_name;
  controller_nh.getParam("effort_joint", eff_joint_name);

  hardware_interface::EffortJointInterface* joint_iface = robot_hw->get<hardware_interface::EffortJointInterface>();
  joint_iface->getHandle(eff_joint_name);
  return true;
}

int DerivedController::helper()
{
  return 2;
}

PLUGINLIB_EXPORT_CLASS(controller_manager_tests::ExtensibleController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(controller_manager_tests::DerivedController, controller_interface::ControllerBase)
