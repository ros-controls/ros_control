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

//#include <iterator>

#include <transmission_interface/transmission_loader.h>


namespace transmission_interface
{

//namespace
//{

///**
// * \brief Update a \ref hardware_interface::JointStateInterface "JointStateInterface" with a new joint.
// *
// * Initializes the joint's raw data, and registers it in the interface. If the joint already exists in the interface,
// * this function results in a no-op.
// */
//void updateJointStateInterface(const std::string& name,
//                               JointInterfaces&   joint_ifaces,
//                               RawJointData&      raw_joint_data)
//{
//  // Do nothing if joint already exists on the hardware interface
//  if (!internal::hasJoint(name, joint_ifaces.joint_state_interface)) {return;}

//  // Update hardware interface
//  using hardware_interface::JointStateHandle;
//  const unsigned int id = addJoint(name, raw_joint_data);
//  JointStateHandle handle(raw_joint_data.names[id],
//                          &raw_joint_data.position[id],
//                          &raw_joint_data.velocity[id],
//                          &raw_joint_data.effort[id]);
//  joint_ifaces.joint_state_interface.registerHandle(handle);
//}

///**
// * \brief Update a \ref hardware_interface::PositionJointInterface "PositionJointInterface" with a new joint.
// *
// * Initializes the joint's raw data, and registers it in the interface. If the joint already exists in the interface,
// * this function results in a no-op.
// */
//void updatePositionJointInterface(const std::string& name,
//                                  JointInterfaces&   joint_ifaces,
//                                  RawJointData&      raw_joint_data)
//{
//  // Do nothing if joint already exists on the hardware interface
//  if (!internal::hasJoint(name, joint_ifaces.position_joint_interface)) {return;}

//  // Update joint states interface first
//  updateJointStateInterface(name, joint_ifaces, raw_joint_data);

//  // Update command interface
//  const std::vector<std::string>& names = raw_joint_data.names;
//  const unsigned int id = std::distance(names.begin(), std::find(names.begin(), names.end(), name));

//  using hardware_interface::JointHandle;
//  JointHandle handle(joint_ifaces.joint_state_interface.getHandle(name), &raw_joint_data.position_cmd[id]);
//  joint_ifaces.position_joint_interface.registerHandle(handle);
//}

///**
// * \brief Update a \ref hardware_interface::VelocityJointInterface "VelocityJointInterface" with a new joint.
// *
// * Initializes the joint's raw data, and registers it in the interface. If the joint already exists in the interface,
// * this function results in a no-op.
// */
//void updateVelocityJointInterface(const std::string& name,
//                                  JointInterfaces&   joint_ifaces,
//                                  RawJointData&      raw_joint_data)
//{
//  // Do nothing if joint already exists on the hardware interface
//  if (!internal::hasJoint(name, joint_ifaces.velocity_joint_interface)) {return;}

//  // Update joint states interface first
//  updateJointStateInterface(name, joint_ifaces, raw_joint_data);

//  // Update command interface
//  const std::vector<std::string>& names = raw_joint_data.names;
//  const unsigned int id = std::distance(names.begin(), std::find(names.begin(), names.end(), name));

//  using hardware_interface::JointHandle;
//  JointHandle handle(joint_ifaces.joint_state_interface.getHandle(name), &raw_joint_data.velocity_cmd[id]);
//  joint_ifaces.velocity_joint_interface.registerHandle(handle);
//}

///**
// * \brief Update a \ref hardware_interface::EffortJointInterface "EffortJointInterface" with a new joint.
// *
// * Initializes the joint's raw data, and registers it in the interface. If the joint already exists in the interface,
// * this function results in a no-op.
// */
//void updateEffortJointInterface(const std::string& name,
//                                JointInterfaces&   joint_ifaces,
//                                RawJointData&      raw_joint_data)
//{
//  // Do nothing if joint already exists on the hardware interface
//  if (!internal::hasJoint(name, joint_ifaces.effort_joint_interface)) {return;}

//  // Update joint states interface first
//  updateJointStateInterface(name, joint_ifaces, raw_joint_data);

//  // Update command interface
//  const std::vector<std::string>& names = raw_joint_data.names;
//  const unsigned int id = std::distance(names.begin(), std::find(names.begin(), names.end(), name));

//  using hardware_interface::JointHandle;
//  JointHandle handle(joint_ifaces.joint_state_interface.getHandle(name), &raw_joint_data.effort_cmd[id]);
//  joint_ifaces.effort_joint_interface.registerHandle(handle);
//}

///**
// * \brief TODO
// */
//bool getActuatorStateData(const std::vector<std::string>&    names,
//                          const hardware_interface::RobotHW& robot_hw,
//                          ActuatorData&                      actuator_data)
//{
//  using hardware_interface::ActuatorStateInterface;
//  using hardware_interface::ActuatorStateHandle;
//  using internal::updateHandleList;

//  // Get handles to all required actuators
//  std::vector<ActuatorStateHandle> handles;
//  if (!updateHandleList<ActuatorStateInterface, ActuatorStateHandle>(names, robot_hw, handles)) {return false;}

//  // Populate actuator data
//  const unsigned int dim = names.size();
//  actuator_data.position.resize(dim);
//  actuator_data.velocity.resize(dim);
//  actuator_data.effort.resize(dim);

//  for (unsigned int i = 0; i < dim; ++i)
//  {
//    // TODO: Get rid of these const casts!
//    actuator_data.position[i] = const_cast<double*>(handles[i].getPositionPtr());
//    actuator_data.velocity[i] = const_cast<double*>(handles[i].getVelocityPtr());
//    actuator_data.effort[i]   = const_cast<double*>(handles[i].getEffortPtr());
//  }
//  return true;
//}

//bool getActuatorStateData(const std::vector<ActuatorInfo>&   actuators_info,
//                          const hardware_interface::RobotHW& robot_hw,
//                          ActuatorData&                      actuator_data)
//{
//  std::vector<std::string> state_actuators;

//  BOOST_FOREACH(const ActuatorInfo& actuator_info, actuators_info)
//  {
//    BOOST_FOREACH(const std::string& hw_iface_name, actuator_info.hardware_interfaces_)
//    {
//      if ("ActuatorStateInterface"    == hw_iface_name ||
//          "PositionActuatorInterface" == hw_iface_name ||
//          "VelocityActuatorInterface" == hw_iface_name ||
//          "EffortActuatorInterface"   == hw_iface_name)
//      {
//        state_actuators.push_back(actuator_info.name_);
//      }
//      else
//      {
//        ROS_ERROR_STREAM_NAMED("parser", "Don't know how to get actuator state data from a hardware interface of type '"
//                               << hw_iface_name << "'.");
//        return false;
//      }
//    }
//  }

//  if (state_actuators.empty())
//  {
//    ROS_ERROR_STREAM_NAMED("parser", "Transmission has no actuators exposing their state.");
//    return false;
//  }

//  if (!getActuatorStateData(state_actuators, robot_hw, actuator_data)) {return false;}

//  return true;
//}

//bool getActuatorPositionCommandData(const std::vector<std::string>&    names,
//                                    const hardware_interface::RobotHW& robot_hw,
//                                    ActuatorData&                      actuator_data)
//{
//  using hardware_interface::PositionActuatorInterface;
//  using hardware_interface::ActuatorHandle;
//  using internal::updateHandleList;

//  // Get handles to all required actuators
//  std::vector<ActuatorHandle> handles;
//  if (!updateHandleList<PositionActuatorInterface, ActuatorHandle>(names, robot_hw, handles)) {return false;}

//  // Populate actuator data
//  const unsigned int dim = names.size();
//  actuator_data.position.resize(dim);
//  for (unsigned int i = 0; i < dim; ++i)
//  {
//    actuator_data.position[i] = const_cast<double*>(handles[i].getCommandPtr()); // TODO: Get rid of these const casts!
//  }

//  return true;
//}

//bool getActuatorVelocityCommandData(const std::vector<std::string>&    names,
//                                    const hardware_interface::RobotHW& robot_hw,
//                                    ActuatorData&                      actuator_data)
//{
//  using hardware_interface::VelocityActuatorInterface;
//  using hardware_interface::ActuatorHandle;
//  using internal::updateHandleList;

//  // Get handles to all required actuators
//  std::vector<ActuatorHandle> handles;
//  if (!updateHandleList<VelocityActuatorInterface, ActuatorHandle>(names, robot_hw, handles)) {return false;}

//  // Populate actuator data
//  const unsigned int dim = names.size();
//  actuator_data.velocity.resize(dim);
//  for (unsigned int i = 0; i < dim; ++i)
//  {
//    actuator_data.velocity[i] = const_cast<double*>(handles[i].getCommandPtr()); // TODO: Get rid of these const casts!
//  }

//  return true;
//}

//bool getActuatorEffortCommandData(const std::vector<std::string>&    names,
//                                  const hardware_interface::RobotHW& robot_hw,
//                                  ActuatorData&                      actuator_data)
//{
//  using hardware_interface::EffortActuatorInterface;
//  using hardware_interface::ActuatorHandle;
//  using internal::updateHandleList;

//  // Get handles to all required actuators
//  std::vector<ActuatorHandle> handles;
//  if (!updateHandleList<EffortActuatorInterface, ActuatorHandle>(names, robot_hw, handles)) {return false;}

//  // Populate actuator data
//  const unsigned int dim = names.size();
//  actuator_data.effort.resize(dim);
//  for (unsigned int i = 0; i < dim; ++i)
//  {
//    actuator_data.effort[i] = const_cast<double*>(handles[i].getCommandPtr()); // TODO: Get rid of these const casts!
//  }

//  return true;
//}

//bool getActuatorCommandData(const std::vector<ActuatorInfo>&   actuators_info,
//                            const hardware_interface::RobotHW& robot_hw,
//                            ActuatorData&                      actuator_data)
//{
//  std::vector<std::string> pos_cmd_actuators;
//  std::vector<std::string> vel_cmd_actuators;
//  std::vector<std::string> eff_cmd_actuators;

//  BOOST_FOREACH(const ActuatorInfo& actuator_info, actuators_info)
//  {
//    BOOST_FOREACH(const std::string& hw_iface_name, actuator_info.hardware_interfaces_)
//    {
//      if      ("PositionActuatorInterface" == hw_iface_name) {pos_cmd_actuators.push_back(actuator_info.name_);}
//      else if ("VelocityActuatorInterface" == hw_iface_name) {vel_cmd_actuators.push_back(actuator_info.name_);}
//      else if ("EffortActuatorInterface"   == hw_iface_name) {eff_cmd_actuators.push_back(actuator_info.name_);}
//    }
//  }

//  // We only support having command interfaces span all transmission joints (or none)
//  const unsigned int dim = actuators_info.size();
//  if (!pos_cmd_actuators.empty() && dim != pos_cmd_actuators.size() ||
//      !vel_cmd_actuators.empty() && dim != vel_cmd_actuators.size() ||
//      !eff_cmd_actuators.empty() && dim != eff_cmd_actuators.size())
//  {
//    ROS_ERROR_STREAM_NAMED("parser", "Actuator command interfaces should be defined for all joints in a transmission.");
//    return false;
//  }

//  if (!getActuatorPositionCommandData(pos_cmd_actuators,
//                                      robot_hw,
//                                      actuator_data)) {return false;}

//  if (!getActuatorVelocityCommandData(vel_cmd_actuators,
//                                      robot_hw,
//                                      actuator_data)) {return false;}

//  if (!getActuatorEffortCommandData(eff_cmd_actuators,
//                                    robot_hw,
//                                    actuator_data)) {return false;}

//  return true;
//}



//} // namespace

//bool updateJointInterfaces(const TransmissionInfo& transmission_info,
//                           JointInterfaces&        joint_ifaces,
//                           RawJointData&           raw_joint_data)
//{
//  BOOST_FOREACH(const JointInfo& jnt_info, transmission_info.joints_)
//  {
//    // Hardware interfaces
//    std::vector<std::string> hw_iface_names = jnt_info.hardware_interfaces_;

//    // If unspecified, try to infer the hardware interface from the actuators interface
//    if (hw_iface_names.empty())
//    {
//      const std::vector<ActuatorInfo>& act_infos = transmission_info.actuators_;
//      if (act_infos.empty())
//      {
//        ROS_ERROR_STREAM_NAMED("parser", "Can't infer hardware interface for joints in transmission '" <<
//                               transmission_info.name_ << "'. There are no actuators specifed.");
//        return false;
//      }

//      typedef typename std::vector<ActuatorInfo>::const_iterator ActuatorInfoConstIter;
//      hw_iface_names = act_infos.front().hardware_interfaces_; // Joint has hardware interfaces of first actuator
//      BOOST_FOREACH(const ActuatorInfo& act_info, act_infos)
//      {
//        // Error out if at least one actuator has a different hardware interface
//        if (!internal::is_permutation(hw_iface_names.begin(), hw_iface_names.end(),
//                                      act_info.hardware_interfaces_.begin()))
//        {
//          ROS_ERROR_STREAM_NAMED("parser", "Can't infer hardware interface for joints in transmission '" <<
//                                 transmission_info.name_ <<
//                                 "'. Inferring is only possible when all actuators have the same hardware interfaces.");
//          return false;
//        }
//      }
//    }

//    // Register joint on all its hardware interfaces
//    BOOST_FOREACH(const std::string& hw_iface_name, hw_iface_names)
//    {
//      if ("JointStateInterface" == hw_iface_name)
//      {
//        updateJointStateInterface(jnt_info.name_, joint_ifaces, raw_joint_data);
//      }
//      else if ("PositionJointInterface" == hw_iface_name)
//      {
//        updatePositionJointInterface(jnt_info.name_, joint_ifaces, raw_joint_data);
//      }
//      else if ("VelocityJointInterface" == hw_iface_name)
//      {
//        updateVelocityJointInterface(jnt_info.name_, joint_ifaces, raw_joint_data);
//      }
//      else if ("EffortJointInterface" == hw_iface_name)
//      {
//        updateEffortJointInterface(jnt_info.name_, joint_ifaces, raw_joint_data);
//      }
//      else
//      {
//        ROS_ERROR_STREAM_NAMED("parser", "Don't know how to add a joint to a hardware interface of type '" <<
//                               hw_iface_name << "'.");
//        return false;
//      }
//    }

//    return true;
//  }
//}

//bool getActuatorData(const std::vector<ActuatorInfo>&   actuators_info,
//                     const hardware_interface::RobotHW& robot_hw,
//                     ActuatorData&                      actuator_state_data,
//                     ActuatorData&                      actuator_cmd_data)
//{
//  const bool state_data_ok = getActuatorStateData(actuators_info,
//                                                  robot_hw,
//                                                  actuator_state_data);
//  if (!state_data_ok) {return false;}

//  const bool command_data_ok = getActuatorCommandData(actuators_info,
//                                                      robot_hw,
//                                                      actuator_cmd_data);
//  if (!command_data_ok) {return false;}

//  return true;
//}

} // namespace
