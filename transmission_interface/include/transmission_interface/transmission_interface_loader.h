
///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
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

/// \author Adolfo Rodriguez Tsouroukdissian

#ifndef TRANSMISSION_INTERFACE_TRANSMISSION_INTERFACE_LOADER_H
#define TRANSMISSION_INTERFACE_TRANSMISSION_INTERFACE_LOADER_H

// C++ standard
#include <algorithm>
#include <string>
#include <utility>
#include <vector>

// Boost
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

// ROS
#include <ros/console.h>

// pluginlib
#include <pluginlib/class_loader.h>

// ros_control
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/internal/demangle_symbol.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

#include <transmission_interface/robot_transmissions.h>
#include <transmission_interface/transmission.h>
#include <transmission_interface/transmission_interface.h>
#include <transmission_interface/transmission_info.h>
#include <transmission_interface/transmission_loader.h>

namespace transmission_interface
{

namespace internal
{
// NOTE: Adapted to C++03 from http://www.cplusplus.com/reference/algorithm/is_permutation
// TODO: Use Boost's or C++11's implementation, once they become widespread
template<class ForwardIt1, class ForwardIt2>
bool is_permutation(ForwardIt1 first, ForwardIt1 last,
                    ForwardIt2 d_first)
{
  // skip common prefix
  std::pair<ForwardIt1, ForwardIt2> pair = std::mismatch(first, last, d_first);
  first = pair.first; d_first = pair.second;

  // iterate over the rest, counting how many times each element
  // from [first, last) appears in [d_first, d_last)
  if (first != last) {
    ForwardIt2 d_last = d_first;
    std::advance(d_last, std::distance(first, last));
    for (ForwardIt1 i = first; i != last; ++i) {
      if (i != std::find(first, i, *i)) continue; // already counted this *i

      unsigned int m = std::count(d_first, d_last, *i);
      if (m==0 || std::count(i, last, *i) != m) {
          return false;
      }
    }
  }
  return true;
}

} // namespace


/**
 * \brief Raw data for a set of joints.
 *
 * For simplicity, every joint has read/write position, velocity and effort variables, but not all of them will
 * necessarily be used.
 */
// TODO: Don't assume interfaces?
struct RawJointData
{
  std::vector<std::string> names;
  std::vector<double>      position;
  std::vector<double>      velocity;
  std::vector<double>      effort;
  std::vector<double>      position_cmd;
  std::vector<double>      velocity_cmd;
  std::vector<double>      effort_cmd;
};

/**
 * \brief Joint interfaces of a robot. Only used interfaces need to be populated.
 */
struct JointInterfaces
{
  hardware_interface::JointStateInterface    joint_state_interface;
  hardware_interface::PositionJointInterface position_joint_interface;
  hardware_interface::VelocityJointInterface velocity_joint_interface;
  hardware_interface::EffortJointInterface   effort_joint_interface;
};

struct ForwardTransmissionInterfaces
{
  ActuatorToJointStateInterface    act_to_jnt_state;
  JointToActuatorPositionInterface jnt_to_act_pos_cmd;
  JointToActuatorVelocityInterface jnt_to_act_vel_cmd;
  JointToActuatorEffortInterface   jnt_to_act_eff_cmd;
};

/**
 * \brief TODO
 */
struct TransmissionLoaderData
{
  typedef boost::shared_ptr<Transmission> TransmissionPtr;

  TransmissionLoaderData()
    : robot_hw(0),
      joint_interfaces(0),
      raw_joint_data(0),
      robot_transmissions(0),
      transmission_interfaces(0),
      transmission_data()
  {}

  hardware_interface::RobotHW*  robot_hw;
  JointInterfaces*              joint_interfaces;
  RawJointData*                 raw_joint_data;

  RobotTransmissions*            robot_transmissions;
  ForwardTransmissionInterfaces* transmission_interfaces;
  std::vector<TransmissionPtr>   transmission_data;
};

class RequisiteProvider // TODO: There must be a more descriptive name for this class!
{
public:
  typedef TransmissionLoaderData::TransmissionPtr TransmissionPtr;

  virtual ~RequisiteProvider() {}

  /**
   * \brief Update a robot's joint interfaces with joint information contained in a transmission.
   *
   * \param[in] transmission_info Structure containing information of which joints to add. Only new, non-previously
   * registered joints will be added.
   *
   * \param[out] joint_ifaces Joint interfaces where new joints will be added. It may already contain data, which
   * will not be overwritten; only new joints will be added.
   *
   * \param raw_joint_data[out] Structure where the raw data of new joints will reside.  It may already contain data,
   * which will not be overwritten; only new data will be added.
   *
   * \return true if succesful.
   */
  virtual bool updateJointInterfaces(const TransmissionInfo& transmission_info,
                                     JointInterfaces&        joint_interfaces,
                                     RawJointData&           raw_joint_data) = 0;

  bool loadTransmissionMaps(const TransmissionInfo& transmission_info,
                            TransmissionLoaderData& loader_data,
                            TransmissionPtr         transmission);
protected:
  struct TransmissionHandleData
  {
    std::string     name;
    ActuatorData    act_state_data;
    ActuatorData    act_cmd_data;
    JointData       jnt_state_data;
    JointData       jnt_cmd_data;
    TransmissionPtr transmission;
  };

  virtual bool getJointStateData(const TransmissionInfo& transmission_info,
                                 const RawJointData&     raw_joint_data,
                                 JointData&              jnt_state_data) = 0;

  virtual bool getJointCommandData(const TransmissionInfo& transmission_info,
                                   const RawJointData&     raw_joint_data,
                                   JointData&              jnt_cmd_data) = 0;

  virtual bool getActuatorStateData(const TransmissionInfo&      transmission_info,
                                    hardware_interface::RobotHW* robot_hw,
                                    ActuatorData&                act_state_data) = 0;

  virtual bool getActuatorCommandData(const TransmissionInfo&      transmission_info,
                                      hardware_interface::RobotHW* robot_hw,
                                      ActuatorData&                act_cmd_data) = 0;

  virtual bool registerTransmission(TransmissionLoaderData& loader_data,
                                    TransmissionHandleData& handle_data) = 0;

  template <class Interface>
  static bool hasResource(const std::string& name, const Interface& iface)
  {
    using hardware_interface::internal::demangledTypeName;

    // Do nothing if resource already exists on the interface
    const std::vector<std::string>& existing_resources = iface.getNames();
    if (existing_resources.end() != std::find(existing_resources.begin(), existing_resources.end(), name))
    {
      ROS_DEBUG_STREAM_NAMED("parser", "Resource '" << name << "' already exists on interface '" <<
                             demangledTypeName<Interface>());
      return true;
    }
    else
    {
      ROS_DEBUG_STREAM_NAMED("parser", "Resource '" << name << "' does not exist on interface '" <<
                             demangledTypeName<Interface>());
      return false;
    }
  }

  static unsigned int addJoint(const std::string& name, RawJointData& raw_joint_data);


  template <class HardwareInterface, class Handle>
  bool getActuatorHandles(const std::vector<ActuatorInfo>& actuators_info,
                          hardware_interface::RobotHW*     robot_hw,
                          std::vector<Handle>&             handles)
  {
    using hardware_interface::RobotHW;
    using hardware_interface::HardwareInterfaceException;
    using hardware_interface::internal::demangledTypeName;

    HardwareInterface* hw_iface = robot_hw->get<HardwareInterface>();

    // Check required hardware interface
    if (!hw_iface)
    {
      ROS_ERROR_STREAM_NAMED("parser", "Robot does not have the required hardware interface '" <<
                             demangledTypeName<HardwareInterface>() << "'.");
      return false;
    }

    // Get handles to all required resource
    BOOST_FOREACH(const ActuatorInfo& info, actuators_info)
    {
      try
      {
        handles.push_back(hw_iface->getHandle(info.name_));
      }
      catch(const HardwareInterfaceException& ex)
      {
        ROS_ERROR_STREAM_NAMED("parser", "Actuator '" << info.name_ <<
                               "' does not expose the required hardware interface '" <<
                               demangledTypeName<HardwareInterface>() << "'.");
        return false;
      }
    }
    return true;
  }
};

class TransmissionInterfaceLoader
{
public:
  TransmissionInterfaceLoader();

  bool load(const TransmissionInfo& transmission_info,
            TransmissionLoaderData& loader_data);

private:
  typedef pluginlib::ClassLoader<TransmissionLoader>      TransmissionClassLoader;
  typedef boost::shared_ptr<TransmissionClassLoader>      TransmissionClassLoaderPtr;
  typedef pluginlib::ClassLoader<RequisiteProvider>       RequisiteProviderClassLoader;
  typedef boost::shared_ptr<RequisiteProviderClassLoader> RequisiteProviderClassLoaderPtr;

  typedef boost::shared_ptr<Transmission>                 TransmissionPtr;
  typedef boost::shared_ptr<TransmissionLoader>           TransmissionLoaderPtr;
  typedef boost::shared_ptr<RequisiteProvider>            RequisiteProviderPtr;

  TransmissionClassLoaderPtr transmission_class_loader_;
  RequisiteProviderClassLoaderPtr req_provider_loader_;

protected:
  bool isValid(const TransmissionLoaderData& loader_data);
};

class JointStateInterfaceProvider : public RequisiteProvider
{
public:
  bool updateJointInterfaces(const TransmissionInfo& transmission_info,
                             JointInterfaces&        joint_interfaces,
                             RawJointData&           raw_joint_data);

protected:
  bool getJointStateData(const TransmissionInfo& transmission_info,
                         const RawJointData&     raw_joint_data,
                         JointData&              jnt_state_data);

  bool getJointCommandData(const TransmissionInfo& transmission_info,
                           const RawJointData&     raw_joint_data,
                           JointData&              jnt_cmd_data) {return true;}

  bool getActuatorStateData(const TransmissionInfo&      transmission_info,
                            hardware_interface::RobotHW* robot_hw,
                            ActuatorData&                act_state_data);

  bool getActuatorCommandData(const TransmissionInfo&      transmission_info,
                              hardware_interface::RobotHW* robot_hw,
                              ActuatorData&                act_cmd_data) {return true;}

  bool registerTransmission(TransmissionLoaderData& loader_data,
                            TransmissionHandleData& handle_data);
};

class PositionJointInterfaceProvider : public JointStateInterfaceProvider
{
public:
  bool updateJointInterfaces(const TransmissionInfo& transmission_info,
                             JointInterfaces&        joint_interfaces,
                             RawJointData&           raw_joint_data);

protected:

  bool getJointCommandData(const TransmissionInfo& transmission_info,
                           const RawJointData&     raw_joint_data,
                           JointData&              jnt_cmd_data);

  bool getActuatorCommandData(const TransmissionInfo&      transmission_info,
                              hardware_interface::RobotHW* robot_hw,
                              ActuatorData&                act_cmd_data);

  bool registerTransmission(TransmissionLoaderData& loader_data,
                            TransmissionHandleData& handle_data);
};

class VelocityJointInterfaceProvider : public JointStateInterfaceProvider
{
public:
  bool updateJointInterfaces(const TransmissionInfo& transmission_info,
                             JointInterfaces&        joint_interfaces,
                             RawJointData&           raw_joint_data);

protected:

  bool getJointCommandData(const TransmissionInfo& transmission_info,
                           const RawJointData&     raw_joint_data,
                           JointData&              jnt_cmd_data);

  bool getActuatorCommandData(const TransmissionInfo&      transmission_info,
                              hardware_interface::RobotHW* robot_hw,
                              ActuatorData&                act_cmd_data);

  bool registerTransmission(TransmissionLoaderData& loader_data,
                            TransmissionHandleData& handle_data);
};

class EffortJointInterfaceProvider : public JointStateInterfaceProvider
{
public:
  bool updateJointInterfaces(const TransmissionInfo& transmission_info,
                             JointInterfaces&        joint_interfaces,
                             RawJointData&           raw_joint_data);

protected:

  bool getJointCommandData(const TransmissionInfo& transmission_info,
                           const RawJointData&     raw_joint_data,
                           JointData&              jnt_cmd_data);

  bool getActuatorCommandData(const TransmissionInfo&      transmission_info,
                              hardware_interface::RobotHW* robot_hw,
                              ActuatorData&                act_cmd_data);

  bool registerTransmission(TransmissionLoaderData& loader_data,
                            TransmissionHandleData& handle_data);
};

} // namespace

#endif // header guard
