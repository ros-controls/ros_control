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
//   * Neither the name of PAL Robotics S.L. nor the names of its
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


// Pluginlib
#include <pluginlib/class_list_macros.hpp>

// ros_control
#include <transmission_interface/velocity_joint_interface_provider.h>

namespace transmission_interface
{

bool VelocityJointInterfaceProvider::updateJointInterfaces(const TransmissionInfo&      transmission_info,
                                                           hardware_interface::RobotHW* robot_hw,
                                                           JointInterfaces&             joint_interfaces,
                                                           RawJointDataMap&             raw_joint_data_map)
{
  // Setup joint state interface first
  if (!JointStateInterfaceProvider::updateJointInterfaces(transmission_info,
                                                          robot_hw,
                                                          joint_interfaces,
                                                          raw_joint_data_map)) {return false;}

  // If interface does not yet exist in robot hardware abstraction, add it and use internal data structures
  using hardware_interface::VelocityJointInterface;
  if (!robot_hw->get<VelocityJointInterface>())
  {
    robot_hw->registerInterface(&joint_interfaces.velocity_joint_interface);
  }
  VelocityJointInterface& interface = *(robot_hw->get<VelocityJointInterface>());

  // Register joints on the hardware interface
  for (const auto& joint_info : transmission_info.joints_)
  {
    const std::string& name = joint_info.name_;

    // Do nothing if joint already exists on the hardware interface
    if (hasResource(name, interface)) {continue;}

    // Update hardware interface
    using hardware_interface::JointHandle;
    RawJointData& raw_joint_data = raw_joint_data_map[name];
    JointHandle handle(joint_interfaces.joint_state_interface.getHandle(joint_info.name_),
                       &raw_joint_data.velocity_cmd);
    interface.registerHandle(handle);
  }
  return true;
}

bool VelocityJointInterfaceProvider::getJointCommandData(const TransmissionInfo& transmission_info,
                                                         const RawJointDataMap&  raw_joint_data_map,
                                                         JointData&              jnt_cmd_data)
{
  const unsigned int dim = transmission_info.joints_.size();
  jnt_cmd_data.velocity.resize(dim);

  for (unsigned int i = 0; i < dim; ++i)
  {
    const std::string& joint_name = transmission_info.joints_[i].name_;
    RawJointDataMap::const_iterator raw_joint_data_it = raw_joint_data_map.find(joint_name);
    if (raw_joint_data_it == raw_joint_data_map.end()) {return false;} // Joint name not found!
    const RawJointData& raw_joint_data = raw_joint_data_it->second;

    // TODO: Get rid of these const casts!
    jnt_cmd_data.velocity[i] = const_cast<double*>(&(raw_joint_data.velocity_cmd));
  }

  return true;
}

bool VelocityJointInterfaceProvider::getActuatorCommandData(const TransmissionInfo&      transmission_info,
                                                            hardware_interface::RobotHW* robot_hw,
                                                            ActuatorData&                act_cmd_data)
{
  using hardware_interface::VelocityActuatorInterface;
  using hardware_interface::ActuatorHandle;

  // Get handles to all required actuators
  std::vector<ActuatorHandle> handles;
  if (!this->getActuatorHandles<VelocityActuatorInterface, ActuatorHandle>(transmission_info.actuators_,
                                                                           robot_hw,
                                                                           handles)) {return false;}

  // Populate actuator data
  const unsigned int dim = transmission_info.actuators_.size();
  act_cmd_data.velocity.resize(dim);

  for (unsigned int i = 0; i < dim; ++i)
  {
    // TODO: Get rid of these const casts!
    act_cmd_data.velocity[i] = const_cast<double*>(handles[i].getCommandPtr());
  }
  return true;
}

bool VelocityJointInterfaceProvider::registerTransmission(TransmissionLoaderData& loader_data,
                                                          TransmissionHandleData& handle_data)
{
  // Setup joint state interface first (if not yet done)
  if (!hasResource(handle_data.name, loader_data.transmission_interfaces.act_to_jnt_state))
  {
    if (!JointStateInterfaceProvider::registerTransmission(loader_data, handle_data)) {return false;}
  }

  // If command interface does not yet exist in the robot transmissions, add it and use internal data structures
  if (!loader_data.robot_transmissions->get<JointToActuatorVelocityInterface>())
  {
    loader_data.robot_transmissions->registerInterface(&loader_data.transmission_interfaces.jnt_to_act_vel_cmd);
  }
  JointToActuatorVelocityInterface& interface = *(loader_data.robot_transmissions->get<JointToActuatorVelocityInterface>());

  // Setup command interface
  JointToActuatorVelocityHandle handle(handle_data.name,
                                       handle_data.transmission.get(),
                                       handle_data.act_cmd_data,
                                       handle_data.jnt_cmd_data);
  interface.registerHandle(handle);
  return true;
}

} // namespace

PLUGINLIB_EXPORT_CLASS(transmission_interface::VelocityJointInterfaceProvider,
                       transmission_interface::RequisiteProvider)
