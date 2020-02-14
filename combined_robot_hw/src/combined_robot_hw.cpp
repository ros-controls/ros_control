///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2015, Shadow Robot Company Ltd.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
// * Neither the name of Shadow Robot Company Ltd. nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
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
#include "combined_robot_hw/combined_robot_hw.h"

namespace combined_robot_hw
{
  bool CombinedRobotHW::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
  {
    root_nh_ = root_nh;
    robot_hw_nh_ = robot_hw_nh;

    std::vector<std::string> robots;
    std::string param_name = "robot_hardware";
    if (!robot_hw_nh.getParam(param_name, robots))
    {
      ROS_ERROR_STREAM("Could not find '" << param_name << "' parameter (namespace: " << robot_hw_nh.getNamespace() << ").");
      return false;
    }

    for (const auto& robot : robots)
    {
      if (!loadRobotHW(robot))
      {
        return false;
      }
    }
    return true;
  }

  bool CombinedRobotHW::prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                             const std::list<hardware_interface::ControllerInfo>& stop_list)
  {
    // Call the prepareSwitch method of the single RobotHW objects.
    for (const auto& robot_hw : robot_hw_list_)
    {
      std::list<hardware_interface::ControllerInfo> filtered_start_list;
      std::list<hardware_interface::ControllerInfo> filtered_stop_list;

      // Generate a filtered version of start_list and stop_list for each RobotHW before calling prepareSwitch
      filterControllerList(start_list, filtered_start_list, robot_hw);
      filterControllerList(stop_list, filtered_stop_list, robot_hw);

      if (!robot_hw->prepareSwitch(filtered_start_list, filtered_stop_list))
        return false;
    }
    return true;
  }

  void CombinedRobotHW::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                        const std::list<hardware_interface::ControllerInfo>& stop_list)
  {
    // Call the doSwitch method of the single RobotHW objects.
    for (const auto& robot_hw : robot_hw_list_)
    {
      std::list<hardware_interface::ControllerInfo> filtered_start_list;
      std::list<hardware_interface::ControllerInfo> filtered_stop_list;

      // Generate a filtered version of start_list and stop_list for each RobotHW before calling doSwitch
      filterControllerList(start_list, filtered_start_list, robot_hw);
      filterControllerList(stop_list, filtered_stop_list, robot_hw);

      robot_hw->doSwitch(filtered_start_list, filtered_stop_list);
    }
  }

  bool CombinedRobotHW::loadRobotHW(const std::string& name)
  {
    ROS_DEBUG("Will load robot HW '%s'", name.c_str());

    ros::NodeHandle c_nh;
    // Constructs the robot HW
    try
    {
      c_nh = ros::NodeHandle(robot_hw_nh_, name);
    }
    catch(std::exception &e)
    {
      ROS_ERROR("Exception thrown while constructing nodehandle for robot HW with name '%s':\n%s", name.c_str(), e.what());
      return false;
    }
    catch(...)
    {
      ROS_ERROR("Exception thrown while constructing nodehandle for robot HW with name '%s'", name.c_str());
      return false;
    }

    hardware_interface::RobotHWSharedPtr robot_hw;
    std::string type;
    if (c_nh.getParam("type", type))
    {
      ROS_DEBUG("Constructing robot HW '%s' of type '%s'", name.c_str(), type.c_str());
      try
      {
        for (const auto& cur_type : robot_hw_loader_.getDeclaredClasses())
        {
          if (type == cur_type)
          {
            robot_hw = robot_hw_loader_.createUniqueInstance(type);
          }
        }
      }
      catch (const std::runtime_error &ex)
      {
        ROS_ERROR("Could not load class %s: %s", type.c_str(), ex.what());
      }
    }
    else
    {
      ROS_ERROR("Could not load robot HW '%s' because the type was not specified. Did you load the robot HW configuration on the parameter server (namespace: '%s')?", name.c_str(), c_nh.getNamespace().c_str());
      return false;
    }

    // checks if robot HW was constructed
    if (!robot_hw)
    {
      ROS_ERROR("Could not load robot HW '%s' because robot HW type '%s' does not exist.",  name.c_str(), type.c_str());
      return false;
    }

    // Initializes the robot HW
    ROS_DEBUG("Initializing robot HW '%s'", name.c_str());
    bool initialized;
    try
    {
      initialized = robot_hw->init(root_nh_, c_nh);
    }
    catch(std::exception &e)
    {
      ROS_ERROR("Exception thrown while initializing robot HW %s.\n%s", name.c_str(), e.what());
      initialized = false;
    }
    catch(...)
    {
      ROS_ERROR("Exception thrown while initializing robot HW %s", name.c_str());
      initialized = false;
    }

    if (!initialized)
    {
      ROS_ERROR("Initializing robot HW '%s' failed", name.c_str());
      return false;
    }
    ROS_DEBUG("Initialized robot HW '%s' successful", name.c_str());

    robot_hw_list_.push_back(robot_hw);

    this->registerInterfaceManager(robot_hw.get());

    ROS_DEBUG("Successfully load robot HW '%s'", name.c_str());
    return true;
  }

  void CombinedRobotHW::read(const ros::Time& time, const ros::Duration& period)
  {
    // Call the read method of the single RobotHW objects.
    for (const auto& robot_hw : robot_hw_list_)
    {
      robot_hw->read(time, period);
    }
  }


  void CombinedRobotHW::write(const ros::Time& time, const ros::Duration& period)
  {
    // Call the write method of the single RobotHW objects.
    for (const auto& robot_hw : robot_hw_list_)
    {
      robot_hw->write(time, period);
    }
  }

  void CombinedRobotHW::filterControllerList(const std::list<hardware_interface::ControllerInfo>& list,
                                             std::list<hardware_interface::ControllerInfo>& filtered_list,
                                             hardware_interface::RobotHWSharedPtr robot_hw)
  {
    filtered_list.clear();
    for (const auto& controller : list)
    {
      hardware_interface::ControllerInfo filtered_controller;
      filtered_controller.name = controller.name;
      filtered_controller.type = controller.type;

      if (controller.claimed_resources.empty())
      {
        filtered_list.push_back(filtered_controller);
        continue;
      }
      for (const auto& claimed_resource : controller.claimed_resources)
      {
        hardware_interface::InterfaceResources filtered_iface_resources;
        filtered_iface_resources.hardware_interface = claimed_resource.hardware_interface;
        std::vector<std::string> r_hw_ifaces = robot_hw->getNames();

        auto const if_name = std::find(r_hw_ifaces.begin(), r_hw_ifaces.end(), filtered_iface_resources.hardware_interface);
        if (if_name == r_hw_ifaces.end()) // this hardware_interface is not registered in r_hw, so we filter it out
        {
          continue;
        }

        std::vector<std::string> r_hw_iface_resources = robot_hw->getInterfaceResources(filtered_iface_resources.hardware_interface);
        std::set<std::string> filtered_resources;
        for (const auto& resource : claimed_resource.resources)
        {
          std::vector<std::string>::iterator res_name = std::find(r_hw_iface_resources.begin(), r_hw_iface_resources.end(), resource);
          if (res_name != r_hw_iface_resources.end())
          {
            filtered_resources.insert(resource);
          }
        }
        if (!filtered_resources.empty())
        {
          filtered_iface_resources.resources = filtered_resources;
          filtered_controller.claimed_resources.push_back(filtered_iface_resources);
        }
      }
      if (!filtered_controller.claimed_resources.empty())
      {
        filtered_list.push_back(filtered_controller);
      }
    }
  }
}
