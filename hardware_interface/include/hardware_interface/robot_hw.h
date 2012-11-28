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

#ifndef HARDWARE_INTERFACE_ROBOT_HW_H
#define HARDWARE_INTERFACE_ROBOT_HW_H

#include <map>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/controller_info.h>
#include <ros/ros.h>

namespace hardware_interface
{

class RobotHW
{
public:
  RobotHW()
  {

  }

  // ****** Resource Management Check ******
  virtual bool checkForConflict(const std::list<ControllerInfo>& info) const
  {
    // See if any two controllers use the same resource

    // Figure out which resources have multiple users
    typedef std::map<std::string, std::list<ControllerInfo> > ResourceMap;
    ResourceMap resource_map;
    for (std::list<ControllerInfo>::const_iterator info_it = info.begin(); info_it != info.end(); info_it++)
      for (std::set<std::string>::const_iterator resource_it = info_it->resources.begin(); resource_it != info_it->resources.end(); resource_it++)
        resource_map[*resource_it].push_back(*info_it);

    bool in_conflict = false;
    for (ResourceMap::iterator it = resource_map.begin(); it != resource_map.end(); it++)
    {
      if (it->second.size() > 1)
      {
        std::string controller_list;
        for (std::list<ControllerInfo>::const_iterator controller_it = it->second.begin(); controller_it != it->second.end(); controller_it++)
          controller_list += controller_it->name + ", ";
        ROS_WARN("Resource conflict on [%s].  Controllers = [%s]", it->first.c_str(), controller_list.c_str());
        in_conflict = true;
      }
    }

    return in_conflict;
  }

  // ****** Hardware Interface registration and getter ******
  template<class T>
  void registerInterface(T* hw)
  {
    interfaces_[typeid(T).name()] = hw;
  }

  template<class T>
  HardwareInterface* get()
  {
    InterfaceMap::iterator it = interfaces_.find(typeid(T).name());
    if (it == interfaces_.end())
      return NULL;
    return it->second;
  }

private:
  typedef std::map<std::string, HardwareInterface*> InterfaceMap;
  InterfaceMap interfaces_;
};

}

#endif

