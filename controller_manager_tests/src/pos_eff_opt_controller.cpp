///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2015, PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the names of PAL Robotics S.L. nor the names of its
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

#include <controller_manager_tests/pos_eff_opt_controller.h>

using namespace controller_manager_tests;

bool PosEffOptController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &n)
{
  std::vector<std::string> pos_joints;
  if (!n.getParam("position_joints", pos_joints)) {return false;}

  std::vector<std::string> eff_joints;
  if (!n.getParam("effort_joints", eff_joints)) {return false;}

  typedef hardware_interface::PositionJointInterface PosIface;
  typedef hardware_interface::EffortJointInterface EffIface;

  // In this controller interfaces are optional, so initRequest did not check interface existence
  // The position interface is OPTIONAL
  // The effort interface is REQUIRED
  PosIface* pos_iface = robot_hw->get<PosIface>();
  EffIface* eff_iface = robot_hw->get<EffIface>();

  // populate command handles (claimed resources)
  if (pos_iface)
  {
    for (const auto& pos_joint : pos_joints)
    {
      pos_cmd_.push_back(pos_iface->getHandle(pos_joint));
    }
  }
  else
  {
    ROS_WARN("Optional interface not found: 'hardware_interface::PositionJointInterface'");
  }

  if (eff_iface)
  {
    for (const auto& eff_joint : eff_joints)
    {
      eff_cmd_.push_back(eff_iface->getHandle(eff_joint));
    }
  }
  else
  {
    ROS_ERROR("Required interface not found: 'hardware_interface::EffortJointInterface'");
    return false;
  }

  return true;
}

void PosEffOptController::starting(const ros::Time& /*time*/)
{
  ROS_INFO("Starting PosEffOptController");
}

void PosEffOptController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{}

void PosEffOptController::stopping(const ros::Time& /*time*/)
{
  ROS_INFO("Stopping PosEffOptController");
}

PLUGINLIB_EXPORT_CLASS(controller_manager_tests::PosEffOptController, controller_interface::ControllerBase)
