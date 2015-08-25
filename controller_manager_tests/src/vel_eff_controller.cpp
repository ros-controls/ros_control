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

#include <controller_manager_tests/vel_eff_controller.h>

using namespace controller_manager_tests;

bool VelEffController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &n)
{
  std::vector<std::string> vel_joints;
  if (!n.getParam("velocity_joints", vel_joints)) {return false;}

  std::vector<std::string> eff_joints;
  if (!n.getParam("effort_joints", eff_joints)) {return false;}

  typedef std::vector<std::string>::const_iterator NamesIterator;
  typedef hardware_interface::VelocityJointInterface VelIface;
  typedef hardware_interface::EffortJointInterface EffIface;

  // should not fail, as initRequest should have checked interface existence
  VelIface* vel_iface = robot_hw->get<VelIface>();
  EffIface* eff_iface = robot_hw->get<EffIface>();

  // populate command handles (claimed resources)
  for (NamesIterator it = vel_joints.begin(); it != vel_joints.end(); it++)
  {
    vel_cmd_.push_back(vel_iface->getHandle(*it));
  }
  for (NamesIterator it = eff_joints.begin(); it != eff_joints.end(); it++)
  {
    eff_cmd_.push_back(eff_iface->getHandle(*it));
  }


  return true;
}

void VelEffController::starting(const ros::Time& /*time*/)
{
  ROS_INFO("Starting VelEffController");
}

void VelEffController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{}

void VelEffController::stopping(const ros::Time& /*time*/)
{
  ROS_INFO("Stopping VelEffController");
}

PLUGINLIB_EXPORT_CLASS( controller_manager_tests::VelEffController, controller_interface::ControllerBase)
