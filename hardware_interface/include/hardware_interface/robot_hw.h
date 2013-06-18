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

#include <list>
#include <map>
#include <typeinfo>
#include <hardware_interface/internal/demangle_symbol.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/controller_info.h>
#include <ros/console.h>

namespace hardware_interface
{

/** \brief Robot Hardware Interface and Resource Manager
 *
 * This class provides a standardized interface to a set of robot hardware
 * interfaces to the controller manager. It performs resource conflict checking
 * for a given set of controllers and maintains a map of hardware interfaces.
 * It is meant to be used as a base class for abstracting custom robot
 * hardware.
 *
 * The hardware interface map (\ref interfaces_) is a 1-to-1 map between
 * the nams of interface types derived from \ref HardwareInterface  and
 * instances of those interface types.
 *
 */
class RobotHW
{
public:
  RobotHW()
  {

  }

  /** \name Resource Management
   *\{*/

  /** Check (in non-realtime) if the given set of controllers is allowed
   * to run simultaneously.
   *
   * This default implementation simply checks if any two controllers use the
   * same resource.
   */
  virtual bool checkForConflict(const std::list<ControllerInfo>& info) const
  {
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

  /*\}*/

  /** \name Hardware Interface Management
   *\{*/

  /**
   * \brief Register a hardware interface.
   *
   * This associates the name of the type of interface to be registered with
   * the given pointer.
   *
   * \tparam T The hardware interface type
   * \param hw A pointer to a hardware interface
   */
  template<class T>
  void registerInterface(T* hw)
  {
    const std::string iface_name = internal::demangledTypeName<T>();
    if (interfaces_.find(iface_name) != interfaces_.end())
    {
      ROS_WARN_STREAM("Replacing previously registered interface '" << iface_name << "'.");
    }
    interfaces_[internal::demangledTypeName<T>()] = hw;
  }

  /**
   * \brief Get a hardware interface.
   *
   * Since \ref RobotHW only stores one interface per type, this returns a
   * pointer to the requested interface type. If the interface type is not
   * registered, it will return \c NULL.
   *
   * \tparam T The hardware interface type
   * \return A pointer to a hardware interface or \c NULL
   */
  template<class T>
  T* get()
  {
    InterfaceMap::iterator it = interfaces_.find(internal::demangledTypeName<T>());
    if (it == interfaces_.end())
      return NULL;

    T* hw = dynamic_cast<T*>(it->second);
    if (!hw)
    {
      ROS_ERROR("Failed on dynamic_cast<T>(hw) for T = [%s]. This should never happen",
                internal::demangledTypeName<T>().c_str());
      return NULL;
    }
    return hw;
  }

  /*\}*/

private:
  typedef std::map<std::string, HardwareInterface*> InterfaceMap;
  InterfaceMap interfaces_;
};

}

#endif

