///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2015, PAL Robotics S.L.
// Copyright (C) 2017, Clearpath Robotics
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

/** \author Adolfo Rodríguez Tsouroukdissian */

#pragma once


#include <algorithm>
#include <sstream>

#include <controller_interface/controller_base.h>
#include <hardware_interface/robot_hw.h>

namespace controller_interface
{

/** \cond HIDDEN_SYMBOLS */
namespace internal
{

template <class T>
inline std::string enumerateElements(const T&           val,
                                     const std::string& delimiter,
                                     const std::string& prefix,
                                     const std::string& suffix)
{
  std::string ret;
  if (val.empty()) {return ret;}

  const std::string sdp = suffix+delimiter+prefix;
  std::stringstream ss;
  ss << prefix;
  std::copy(val.begin(), val.end(), std::ostream_iterator<typename T::value_type>(ss, sdp.c_str()));
  ret = ss.str();
  if (!ret.empty()) {ret.erase(ret.size() - delimiter.size() - prefix.size());}
  return ret;
}


template <typename T>
inline bool hasInterfaces(hardware_interface::RobotHW* robot_hw)
{
  T* hw = robot_hw->get<T>();
  if (!hw)
  {
    const std::string hw_name = hardware_interface::internal::demangledTypeName<T>();
    ROS_ERROR_STREAM("This controller requires a hardware interface of type '" << hw_name << "', " <<
                     "but is not exposed by the robot. Available interfaces in robot:\n" <<
                     enumerateElements(robot_hw->getNames(), "\n", "- '", "'")); // delimiter, prefix, suffux
    return false;
  }
  return true;
}

template <typename T1, typename T2, typename... More>
inline bool hasInterfaces(hardware_interface::RobotHW* robot_hw)
{
  return hasInterfaces<T1>(robot_hw) && hasInterfaces<T2, More...>(robot_hw);
}


template <typename T>
void clearClaims(hardware_interface::RobotHW* robot_hw)
{
  T* hw = robot_hw->get<T>();
  if (hw)
  {
    hw->clearClaims();
  }
}

template <typename T1, typename T2, typename... More>
void clearClaims(hardware_interface::RobotHW* robot_hw)
{
  clearClaims<T1>(robot_hw);
  clearClaims<T2, More...>(robot_hw);
}


template <typename T>
inline void extractInterfaceResources(hardware_interface::RobotHW* robot_hw_in,
                                      hardware_interface::RobotHW* robot_hw_out)
{
  T* hw = robot_hw_in->get<T>();
  if (hw) {robot_hw_out->registerInterface(hw);}
}

template <typename T1, typename T2, typename... More>
inline void extractInterfaceResources(hardware_interface::RobotHW* robot_hw_in,
                                      hardware_interface::RobotHW* robot_hw_out)
{
  extractInterfaceResources<T1>(robot_hw_in, robot_hw_out);
  extractInterfaceResources<T2, More...>(robot_hw_in, robot_hw_out);
}


template <typename T>
inline void populateClaimedResources(hardware_interface::RobotHW*      robot_hw,
                                     ControllerBase::ClaimedResources& claimed_resources)
{
  T* hw = robot_hw->get<T>();
  if (hw)
  {
    hardware_interface::InterfaceResources iface_res;
    iface_res.hardware_interface = hardware_interface::internal::demangledTypeName<T>();
    iface_res.resources = hw->getClaims();
    claimed_resources.push_back(iface_res);
  }
}

template <typename T1, typename T2, typename... More>
inline void populateClaimedResources(hardware_interface::RobotHW*      robot_hw,
                                     ControllerBase::ClaimedResources& claimed_resources)
{
  populateClaimedResources<T1>(robot_hw, claimed_resources);
  populateClaimedResources<T2, More...>(robot_hw, claimed_resources);
}

} // namespace
/** \endcond */

} // namespace
