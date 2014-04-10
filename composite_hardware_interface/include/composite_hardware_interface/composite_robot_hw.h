/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2014, Igor Kalevatykh, Bauman Moscow State Technical University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the Bauman Moscow State Technical University,
 *      nor the names of its contributors may be used to endorse or promote
 *      products derived from this software without specific prior written
 *      permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/// \author Igor Kalevatykh <kalevatykhia@gmail.com>

#ifndef COMPOSITE_HARDWARE_INTERFACE__COMPOSITE_ROBOT_HW_H
#define COMPOSITE_HARDWARE_INTERFACE__COMPOSITE_ROBOT_HW_H

#include <ros/ros.h>
#include <ros/console.h>
#include <urdf/model.h>
#include <hardware_interface/robot_hw.h>
#include <composite_hardware_interface/internal/device_loader.h>
#include <composite_hardware_interface/device_hw.h>

namespace composite_hardware_interface
{

/** \brief Composite Robot Hardware Interface and Resource Manager
 *
 * This class provides a standardized interface to a set of robot hardware
 * interfaces to the controller manager. This class can load, configure, start,
 * run and stop a set of robot devices.
 *
 */
class CompositeRobotHW : public hardware_interface::RobotHW
{
public:
  CompositeRobotHW(const ros::NodeHandle& nh) :
    node_(nh), configured_(false), started_(false), interfaces_(*this), urdf_model_(nh)
  {
  }

  virtual bool configure()
  {
    // Read device list
    XmlRpc::XmlRpcValue device_list;
    if(!node_.getParam("device_list", device_list))
    {
      ROS_ERROR("Could not load %s/device_list parameter.", node_.getNamespace().c_str());
      return false;
    }

    if(device_list.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_ERROR("Device list should be a map with a name as key.");
      return false;
    }

    // Load and configure each device
    loader_.reset(new DeviceLoader<DeviceHW>("composite_hardware_interface", "composite_hardware_interface::DeviceHW"));

    for (XmlRpc::XmlRpcValue::iterator it = device_list.begin(); it != device_list.end(); ++it)
    {
      DeviceHwPtr dev;
      std::string dev_name = it->first;
      ros::NodeHandle dev_node(node_, "device_list/" + dev_name);

      std::string dev_type;
      if(!dev_node.getParam("type", dev_type))
      {
        ROS_ERROR("Could not find a type attribute for the '%s' hardware interface.", dev_name.c_str());
        return false;
      }

      try
      {
        dev = loader_->createInstance(it->second["type"]);
      }
      catch (const std::exception &e)
      {
        ROS_ERROR("Could not load class %s: %s.", dev_type.c_str(), e.what());
        return false;
      }

      if(!dev->configure(interfaces_, resources_, urdf_model_, dev_name, dev_node))
      {
        ROS_ERROR("Could not configure '%s' hardware interface.", dev_name.c_str());
        return false;
      }

      devices_[dev_name] = dev;
    }

    configured_ = true;
    return true;
  }

  virtual bool start()
  {
    if(!configured_)
      return false;

    for (DeviceIt it = devices_.begin(); it != devices_.end(); ++it)
    {
      if (!it->second->start())
      {
        ROS_ERROR("Could not start '%s' hardware interface.", it->first.c_str());
        return false;
      }
    }

    started_ = true;
    return true;
  }

  virtual bool read(const ros::Time time, const ros::Duration period)
  {
    if(!started_)
      return false;

    bool ok = true;
    for (DeviceIt it = devices_.begin(); it != devices_.end(); ++it)
    {
      if (!it->second->read(time, period))
        ok = false;
    }

    return ok;
  }

  virtual bool write(const ros::Time time, const ros::Duration period)
  {
    if(!started_)
      return false;

    bool ok = true;
    for (DeviceIt it = devices_.begin(); it != devices_.end(); ++it)
    {
      if (!it->second->write(time, period))
        ok = false;
    }

    return ok;
  }

  virtual void stop()
  {
    if (started_)
    {
      for (DeviceIt it = devices_.begin(); it != devices_.end(); ++it)
      {
        it->second->stop();
      }
      started_ = false;
    }
  }

  virtual ~CompositeRobotHW()
  {
    stop();
  }

protected:
  typedef std::map<std::string, DeviceHwPtr> DeviceMap;
  typedef DeviceMap::iterator DeviceIt;

  ros::NodeHandle node_;
  bool configured_;
  bool started_;
  DeviceMap devices_;
  SharedInterfaceManager interfaces_;
  SharedResourceManager resources_;
  SharedUrdfModel urdf_model_;

  boost::shared_ptr<DeviceLoaderInterface> loader_;
};

}

#endif

