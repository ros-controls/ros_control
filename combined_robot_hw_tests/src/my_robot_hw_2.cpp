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
#include <combined_robot_hw_tests/my_robot_hw_2.h>

namespace combined_robot_hw_tests
{

bool MyRobotHW2::init(ros::NodeHandle& /*root_nh*/, ros::NodeHandle &robot_hw_nh)
{
  using namespace hardware_interface;

  std::vector<std::string> joints;
  if (!robot_hw_nh.getParam("joints", joints)) {return false;}

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
    joint_effort_command_[i] = 0.0;
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


void MyRobotHW2::read(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{

}

void MyRobotHW2::write(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
}

bool MyRobotHW2::prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                               const std::list<hardware_interface::ControllerInfo>& /*stop_list*/)
{
  for (const auto& controller : start_list)
  {
    if (controller.name == "ctrl_without_my_robot_hw_2_resources")
    {
      ROS_ERROR_STREAM("Controller should have been filtered out: " << controller.name);
      return false;
    }

    if (controller.name == "ctrl_without_my_robot_hw_2_resources_in_one_of_two_ifaces")
    {
      if (controller.claimed_resources.size() > 1)
      {
        ROS_ERROR_STREAM("One of the interfaces should have been filtered out");
        return false;
      }
    }

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
        return false;
      }

      std::vector<std::string> r_hw_iface_resources = this->getInterfaceResources(res_it.hardware_interface);
      for (const auto& resource : res_it.resources)
      {
        std::vector<std::string>::iterator res_name = std::find(r_hw_iface_resources.begin(), r_hw_iface_resources.end(), resource);
        if (res_name == r_hw_iface_resources.end()) // this resource is not registered on this RobotHW
        {
          ROS_ERROR_STREAM("Bad resource: " << resource);
          return false;
        }
      }
    }
  }
  return true;
}

void MyRobotHW2::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                          const std::list<hardware_interface::ControllerInfo>& /*stop_list*/)
{
  for (const auto& controller : start_list)
  {
    if (controller.name == "ctrl_without_my_robot_hw_2_resources")
    {
      throw hardware_interface::HardwareInterfaceException("Controller " + controller.name + " should have been filtered out");
    }

    if (controller.name == "ctrl_without_my_robot_hw_2_resources_in_one_of_two_ifaces")
    {
      if (controller.claimed_resources.size() > 1)
      {
        throw hardware_interface::HardwareInterfaceException("One of the interfaces should have been filtered out");
      }
    }

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

PLUGINLIB_EXPORT_CLASS( combined_robot_hw_tests::MyRobotHW2, hardware_interface::RobotHW)
