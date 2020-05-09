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


#include <algorithm>
#include <combined_robot_hw_tests/my_robot_hw_1.h>

namespace combined_robot_hw_tests
{

bool MyRobotHW1::init(ros::NodeHandle& /*root_nh*/, ros::NodeHandle &/*robot_hw_nh*/)
{
  using namespace hardware_interface;

  // Initialize raw data
  joint_position_.resize(3);
  joint_velocity_.resize(3);
  joint_effort_.resize(3);
  joint_effort_command_.resize(3);
  joint_velocity_command_.resize(3);
  joint_name_.resize(3);

  joint_name_[0] = "test_joint1";
  joint_position_[0] = 1.0;
  joint_velocity_[0] = 0.0;
  joint_effort_[0] = 0.1;
  joint_effort_command_[0] = 3.0;
  joint_velocity_command_[0] = 0.0;

  joint_name_[1] = "test_joint2";
  joint_position_[1] = 1.0;
  joint_velocity_[1] = 0.0;
  joint_effort_[1] = 0.1;
  joint_effort_command_[1] = 0.0;
  joint_velocity_command_[1] = 0.0;

  joint_name_[2] = "test_joint3";
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

  return true;
}


void MyRobotHW1::read(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  joint_position_[0] = 2.7;
}

void MyRobotHW1::write(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  // Just to test that write() is called
  joint_effort_command_[1] = joint_effort_command_[0];
}

bool MyRobotHW1::prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                               const std::list<hardware_interface::ControllerInfo>& /*stop_list*/)
{
  for (const auto& controller : start_list)
  {
    if (controller.claimed_resources.empty())
    {
      continue;
    }
    for (const auto& res_it : controller.claimed_resources)
    {
      std::vector<std::string> r_hw_ifaces = this->getNames();

      std::vector<std::string>::iterator if_name = std::find(r_hw_ifaces.begin(), r_hw_ifaces.end(), res_it.hardware_interface);
      if (if_name == r_hw_ifaces.end()) // this hardware_interface is not registered on this RobotHW
      {
        ROS_ERROR_STREAM("Bad interface: " << res_it.hardware_interface);
        std::cout << res_it.hardware_interface;
        return false;
      }

      std::vector<std::string> r_hw_iface_resources = this->getInterfaceResources(res_it.hardware_interface);
      for (const auto& resource : res_it.resources)
      {
        std::vector<std::string>::iterator res_name = std::find(r_hw_iface_resources.begin(), r_hw_iface_resources.end(), resource);
        if (res_name == r_hw_iface_resources.end()) // this resource is not registered on this RobotHW
        {
          ROS_ERROR_STREAM("Bad resource: " << resource);
          std::cout << resource;
          return false;
        }
      }
    }
  }
  return true;
}

void MyRobotHW1::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                          const std::list<hardware_interface::ControllerInfo>& /*stop_list*/)
{
  for (const auto& controller : start_list)
  {
    if (controller.claimed_resources.empty())
    {
      continue;
    }
    for (const auto& claimed_resource : controller.claimed_resources)
    {
      std::vector<std::string> r_hw_ifaces = this->getNames();

      std::vector<std::string>::iterator if_name = std::find(r_hw_ifaces.begin(), r_hw_ifaces.end(), claimed_resource.hardware_interface);
      if (if_name == r_hw_ifaces.end()) // this hardware_interface is not registered on this RobotHW
      {
        throw hardware_interface::HardwareInterfaceException("Hardware_interface " + claimed_resource.hardware_interface + " is not registered");
      }

      std::vector<std::string> r_hw_iface_resources = this->getInterfaceResources(claimed_resource.hardware_interface);
      for (const auto& resource : claimed_resource.resources)
      {
        std::vector<std::string>::iterator res_name = std::find(r_hw_iface_resources.begin(), r_hw_iface_resources.end(), resource);
        if (res_name == r_hw_iface_resources.end()) // this resource is not registered on this RobotHW
        {
          throw hardware_interface::HardwareInterfaceException("Resource " + resource + " is not registered");
        }
      }
    }
  }
}

}

PLUGINLIB_EXPORT_CLASS( combined_robot_hw_tests::MyRobotHW1, hardware_interface::RobotHW)
