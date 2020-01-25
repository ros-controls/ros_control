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
#include <transmission_interface/joint_state_interface_provider.h>

namespace transmission_interface
{

bool JointStateInterfaceProvider::updateJointInterfaces(const TransmissionInfo&      transmission_info,
                                                        hardware_interface::RobotHW* robot_hw,
                                                        JointInterfaces&             joint_interfaces,
                                                        RawJointDataMap&             raw_joint_data_map)
{
  // If interface does not yet exist in robot hardware abstraction, add it and use internal data structures
  using hardware_interface::JointStateInterface;
  if (!robot_hw->get<JointStateInterface>())
  {
    robot_hw->registerInterface(&joint_interfaces.joint_state_interface);
  }
  JointStateInterface& interface = *(robot_hw->get<JointStateInterface>());

  // Register joints on the hardware interface
  for (const auto& joint_info : transmission_info.joints_)
  {
    const std::string& name = joint_info.name_;

    // Do nothing if joint already exists on the hardware interface
    if (hasResource(name, interface)) {continue;}

    // Update hardware interface
    using hardware_interface::JointStateHandle;
    RawJointData& raw_joint_data = raw_joint_data_map[name]; // Add joint if it does not yet exist
    if(raw_joint_data.hasAbsolutePosition && raw_joint_data.hasTorqueSensor)
    {
      JointStateHandle handle(name,
                              &raw_joint_data.position,
                              &raw_joint_data.velocity,
                              &raw_joint_data.effort,
                              &raw_joint_data.absolute_position,
                              &raw_joint_data.torque_sensor);
      interface.registerHandle(handle);
    }
    else if(raw_joint_data.hasAbsolutePosition)
    {
      JointStateHandle handle(name,
                              &raw_joint_data.position,
                              &raw_joint_data.velocity,
                              &raw_joint_data.effort,
                              &raw_joint_data.absolute_position);
      interface.registerHandle(handle);
    }
    else if(raw_joint_data.hasTorqueSensor)
    {
      JointStateHandle handle(name,
                              &raw_joint_data.position,
                              &raw_joint_data.velocity,
                              &raw_joint_data.effort,
                              &raw_joint_data.torque_sensor, true);
      interface.registerHandle(handle);
    }
    else
    {
      JointStateHandle handle(name,
                              &raw_joint_data.position,
                              &raw_joint_data.velocity,
                              &raw_joint_data.effort);
      interface.registerHandle(handle);
    }

  }
  return true;
}

bool JointStateInterfaceProvider::getJointStateData(const TransmissionInfo& transmission_info,
                                                    const RawJointDataMap&  raw_joint_data_map,
                                                    JointData&              jnt_state_data)
{
  const unsigned int dim = transmission_info.joints_.size();
  jnt_state_data.position.resize(dim);
  jnt_state_data.velocity.resize(dim);
  jnt_state_data.effort.resize(dim);

  bool hasAbsolutePosition = true;
  bool hasTorqueSensor = true;

  for (unsigned int i = 0; i < dim; ++i)
  {
    const std::string& joint_name = transmission_info.joints_[i].name_;
    RawJointDataMap::const_iterator raw_joint_data_it = raw_joint_data_map.find(joint_name);
    if (raw_joint_data_it == raw_joint_data_map.end()) {return false;} // Joint name not found!
    const RawJointData& raw_joint_data = raw_joint_data_it->second;

    hasAbsolutePosition = hasAbsolutePosition && raw_joint_data.hasAbsolutePosition;
    hasTorqueSensor = hasTorqueSensor && raw_joint_data.hasTorqueSensor;
  }

  if(hasAbsolutePosition)
  {
    jnt_state_data.absolute_position.resize(dim);
  }

  if(hasTorqueSensor)
  {
    jnt_state_data.torque_sensor.resize(dim);
  }

  for (unsigned int i = 0; i < dim; ++i)
  {
    const std::string& joint_name = transmission_info.joints_[i].name_;
    RawJointDataMap::const_iterator raw_joint_data_it = raw_joint_data_map.find(joint_name);
    if (raw_joint_data_it == raw_joint_data_map.end()) {return false;} // Joint name not found!
    const RawJointData& raw_joint_data = raw_joint_data_it->second;

    // TODO: Get rid of these const casts!
    jnt_state_data.position[i] = const_cast<double*>(&(raw_joint_data.position));
    jnt_state_data.velocity[i] = const_cast<double*>(&(raw_joint_data.velocity));
    jnt_state_data.effort[i]   = const_cast<double*>(&(raw_joint_data.effort));
    if(hasAbsolutePosition)
    {
      jnt_state_data.absolute_position[i] = const_cast<double*>(&(raw_joint_data.absolute_position));
    }
    if(hasTorqueSensor)
    {
      jnt_state_data.torque_sensor[i] = const_cast<double*>(&(raw_joint_data.torque_sensor));
    }
  }

  return true;
}

bool JointStateInterfaceProvider::getActuatorStateData(const TransmissionInfo&      transmission_info,
                                                       hardware_interface::RobotHW* robot_hw,
                                                       ActuatorData&                act_state_data)
{
  using hardware_interface::ActuatorStateInterface;
  using hardware_interface::ActuatorStateHandle;

  // Get handles to all required actuators
  std::vector<ActuatorStateHandle> handles;
  if (!this->getActuatorHandles<ActuatorStateInterface, ActuatorStateHandle>(transmission_info.actuators_,
                                                                             robot_hw,
                                                                             handles)) {return false;}

  // Populate actuator data
  const unsigned int dim = transmission_info.actuators_.size();
  act_state_data.position.resize(dim);
  act_state_data.velocity.resize(dim);
  act_state_data.effort.resize(dim);

  bool hasAbsolutePositionInterface = true;
  bool hasTorqueSensorInterface = true;

  for (unsigned int i = 0; i < dim; ++i)
  {
    hasAbsolutePositionInterface = hasAbsolutePositionInterface && handles[i].hasAbsolutePosition();
    hasTorqueSensorInterface = hasTorqueSensorInterface && handles[i].hasTorqueSensor();
  }

  if(hasAbsolutePositionInterface)
  {
    act_state_data.absolute_position.resize(dim);
  }

  if(hasTorqueSensorInterface)
  {
    act_state_data.torque_sensor.resize(dim);
  }

  for (unsigned int i = 0; i < dim; ++i)
  {
    // TODO: Get rid of these const casts!
    act_state_data.position[i] = const_cast<double*>(handles[i].getPositionPtr());
    act_state_data.velocity[i] = const_cast<double*>(handles[i].getVelocityPtr());
    act_state_data.effort[i]   = const_cast<double*>(handles[i].getEffortPtr());
    if(hasAbsolutePositionInterface)
    {
      act_state_data.absolute_position[i] = const_cast<double*>(handles[i].getAbsolutePositionPtr());
    }
    if(hasTorqueSensorInterface)
    {
      act_state_data.torque_sensor[i] = const_cast<double*>(handles[i].getTorqueSensorPtr());
    }
  }
  return true;
}

bool JointStateInterfaceProvider::registerTransmission(TransmissionLoaderData& loader_data,
                                                       TransmissionHandleData& handle_data)
{
  // If interface does not yet exist in the robot transmissions, add it and use internal data structures
  if (!loader_data.robot_transmissions->get<ActuatorToJointStateInterface>())
  {
    loader_data.robot_transmissions->registerInterface(&loader_data.transmission_interfaces.act_to_jnt_state);
  }
  ActuatorToJointStateInterface& interface = *(loader_data.robot_transmissions->get<ActuatorToJointStateInterface>());

  // Update transmission interface
  ActuatorToJointStateHandle handle(handle_data.name,
                                    handle_data.transmission.get(),
                                    handle_data.act_state_data,
                                    handle_data.jnt_state_data);
  interface.registerHandle(handle);
  return true;
}

} // namespace

PLUGINLIB_EXPORT_CLASS(transmission_interface::JointStateInterfaceProvider,
                       transmission_interface::RequisiteProvider)
