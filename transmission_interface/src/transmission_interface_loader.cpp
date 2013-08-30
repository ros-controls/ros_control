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

// C++ standard
#include <cassert>
#include <iterator>
#include <limits>

// Pluginlib
#include <pluginlib/class_list_macros.h>

// ros_control
#include <transmission_interface/transmission_interface_loader.h>


namespace transmission_interface
{

bool RequisiteProvider::loadTransmissionMaps(const TransmissionInfo& transmission_info,
                                             TransmissionLoaderData& loader_data,
                                             TransmissionPtr         transmission)
{
  TransmissionHandleData handle_data;
  handle_data.name         = transmission_info.name_;
  handle_data.transmission = transmission;

  // Check that required actuators are available in the robot
  const bool act_state_data_ok = getActuatorStateData(transmission_info,
                                                      loader_data.robot_hw,
                                                      handle_data.act_state_data);
  if (!act_state_data_ok) {return false;}

  const bool act_cmd_data_ok = getActuatorCommandData(transmission_info,
                                                      loader_data.robot_hw,
                                                      handle_data.act_cmd_data);
  if (!act_cmd_data_ok) {return false;}

  // Update raw joint data
  // This call potentially allocates resources in the raw data structure, so it should be the last requisite to setup.
  // This is because we want to alter the state of the raw data only when we're sure that the transmission will load
  // successfully
  const bool jnt_data_ok = updateJointInterfaces(transmission_info,
                                                 *(loader_data.joint_interfaces),
                                                 *(loader_data.raw_joint_data));
  if (!jnt_data_ok) {return false;}

  // Joint data
  const bool jnt_state_data_ok = getJointStateData(transmission_info,
                                                   *(loader_data.raw_joint_data),
                                                   handle_data.jnt_state_data);
  if (!jnt_state_data_ok) {return false;}

  const bool jnt_cmd_data_ok = getJointCommandData(transmission_info,
                                                   *(loader_data.raw_joint_data),
                                                   handle_data.jnt_cmd_data);
  if (!jnt_cmd_data_ok) {return false;}

  // Update transmission interface
  loader_data.transmission_data.push_back(transmission);

  // Register transmission
  registerTransmission(loader_data, handle_data);

  return true;
}

unsigned int RequisiteProvider::addJoint(const std::string& name, RawJointData& raw_joint_data)
{
  // Fail if joint already exists
  std::vector<std::string>::iterator name_it = std::find(raw_joint_data.names.begin(),
                                                         raw_joint_data.names.end(),
                                                         name);
  if (name_it != raw_joint_data.names.end())
  {
    return std::distance(raw_joint_data.names.begin(), name_it); // Joint already added
  }

  // Add joint
  const double nan = std::numeric_limits<double>::quiet_NaN();
  raw_joint_data.names.push_back(name);
  raw_joint_data.position.push_back(nan);
  raw_joint_data.velocity.push_back(nan);
  raw_joint_data.effort.push_back(nan);
  raw_joint_data.position_cmd.push_back(nan);
  raw_joint_data.velocity_cmd.push_back(nan);
  raw_joint_data.effort_cmd.push_back(nan);

  return raw_joint_data.names.size() - 1;
}

TransmissionInterfaceLoader::TransmissionInterfaceLoader()
{
  // Can throw
  transmission_class_loader_.reset(new TransmissionClassLoader("transmission_interface",
                                                         "transmission_interface::TransmissionLoader"));

  // Can throw
  req_provider_loader_.reset(new RequisiteProviderClassLoader("transmission_interface",
                                                              "transmission_interface::RequisiteProvider"));
}

bool TransmissionInterfaceLoader::load(const TransmissionInfo& transmission_info,
                                       TransmissionLoaderData& loader_data)
{
  // Validate inputs
  if (!isValid(loader_data)) {return false;}

  // Create transmission instance
  TransmissionPtr transmission;
  try
  {
    TransmissionLoaderPtr transmission_loader = transmission_class_loader_->createInstance(transmission_info.type_);
    transmission = transmission_loader->load(transmission_info);
    if (!transmission) {return false;}
  }
  catch(pluginlib::LibraryLoadException &ex)
  {
    ROS_ERROR_STREAM_NAMED("parser", "Failed to load transmission '" << transmission_info.name_ <<
                           "'. Unsupported type '" << transmission_info.type_ << "'.\n" << ex.what());
    return false;
  }

  // We currently only deal with transmissions specifying a single hardware interface in the joints
  assert(!transmission_info.joints_.empty() && !transmission_info.joints_.front().hardware_interfaces_.empty());
  const std::vector<std::string>& hw_ifaces_ref = transmission_info.joints_.front().hardware_interfaces_; // First joint
  BOOST_FOREACH(const JointInfo& jnt_info, transmission_info.joints_)
  {
    // Error out if at least one joint has a different set of hardware interfaces
    if (!internal::is_permutation(hw_ifaces_ref.begin(), hw_ifaces_ref.end(),
                                  jnt_info.hardware_interfaces_.begin()))
    {
      ROS_ERROR_STREAM_NAMED("parser",
                             "Failed to load transmission '" << transmission_info.name_ <<
                             "'. It has joints with different hardware interfaces. This is currently unsupported.");
      return false;
    }
  }

  // Load transmission to all specified interfaces
  bool has_at_least_one_hw_iface = false;
  BOOST_FOREACH(const std::string& hw_iface, hw_ifaces_ref)
  {
    RequisiteProviderPtr req_provider;
    try
    {
      req_provider = req_provider_loader_->createInstance(hw_iface);
      if (!req_provider) {continue;}
    }
    catch(pluginlib::LibraryLoadException &ex)
    {
      ROS_WARN_STREAM_NAMED("parser", "Failed to process the '" << hw_iface <<
                             "' hardware interface for transmission '" << transmission_info.name_ <<
                             "'.\n" << ex.what());
      continue;
    }

    const bool load_ok = req_provider->loadTransmissionMaps(transmission_info, loader_data, transmission);
    if (load_ok) {has_at_least_one_hw_iface = true;}
    else {continue;}
  }

  if (!has_at_least_one_hw_iface)
  {
    ROS_ERROR_STREAM_NAMED("parser", "Failed to load transmission '" << transmission_info.name_ <<
                           "'. It contains no valid hardware interfaces.");
    return false;
  }

  return true;
}

bool TransmissionInterfaceLoader::isValid(const TransmissionLoaderData& loader_data)
{
  std::string error_str;
  bool ret = true;
  if (!loader_data.robot_hw)            {error_str += "\n- RobotHW instance not specified.";    ret = false;}
  if (!loader_data.joint_interfaces)    {error_str += "\n- Joint_interfaces not specified.";    ret = false;}
  if (!loader_data.raw_joint_data)      {error_str += "\n- Raw joint data not specified.";      ret = false;}
  if (!loader_data.robot_transmissions) {error_str += "\n- Robot transmissions not specified."; ret = false;}
  if (!loader_data.transmission_interfaces) {error_str += "\n- Transmission interfaces not specified."; ret = false;}

  if (!ret)
  {
    ROS_ERROR_STREAM_NAMED("parser", "Transmission loader data not properly initialized:" << error_str);
  }
  return ret;
}

bool JointStateInterfaceProvider::updateJointInterfaces(const TransmissionInfo& transmission_info,
                                                        JointInterfaces&        joint_interfaces,
                                                        RawJointData&           raw_joint_data)
{
  // Register joint on all its hardware interfaces
  BOOST_FOREACH(const JointInfo& joint_info, transmission_info.joints_)
  {
    const std::string& name = joint_info.name_;

    // Do nothing if joint already exists on the hardware interface
    if (hasResource(name, joint_interfaces.joint_state_interface)) {return true;}

    // Update hardware interface
    using hardware_interface::JointStateHandle;
    const unsigned int id = addJoint(name, raw_joint_data);
    JointStateHandle handle(raw_joint_data.names[id],
                            &raw_joint_data.position[id],
                            &raw_joint_data.velocity[id],
                            &raw_joint_data.effort[id]);
    joint_interfaces.joint_state_interface.registerHandle(handle);
  }
  return true;
}

bool JointStateInterfaceProvider::getJointStateData(const TransmissionInfo& transmission_info,
                                                    const RawJointData&     raw_joint_data,
                                                    JointData&              jnt_state_data)
{
  const unsigned int dim = transmission_info.joints_.size();
  jnt_state_data.position.resize(dim);
  jnt_state_data.velocity.resize(dim);
  jnt_state_data.effort.resize(dim);

  for (unsigned int i = 0; i < dim; ++i)
  {
    const std::string& joint_name = transmission_info.joints_[i].name_;
    std::vector<std::string>::const_iterator name_it = std::find(raw_joint_data.names.begin(),
                                                                 raw_joint_data.names.end(),
                                                                 joint_name);
    if (name_it == raw_joint_data.names.end()) {return false;} // Joint name not found!
    const unsigned int id = std::distance(raw_joint_data.names.begin(), name_it);

    // TODO: Get rid of these const casts!
    jnt_state_data.position[i] = const_cast<double*>(&(raw_joint_data.position[id]));
    jnt_state_data.velocity[i] = const_cast<double*>(&(raw_joint_data.velocity[id]));
    jnt_state_data.effort[i]   = const_cast<double*>(&(raw_joint_data.effort[id]));
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

  for (unsigned int i = 0; i < dim; ++i)
  {
    // TODO: Get rid of these const casts!
    act_state_data.position[i] = const_cast<double*>(handles[i].getPositionPtr());
    act_state_data.velocity[i] = const_cast<double*>(handles[i].getVelocityPtr());
    act_state_data.effort[i]   = const_cast<double*>(handles[i].getEffortPtr());
  }
  return true;
}

bool JointStateInterfaceProvider::registerTransmission(TransmissionLoaderData& loader_data,
                                                       TransmissionHandleData& handle_data)
{
  ActuatorToJointStateHandle handle(handle_data.name,
                                    handle_data.transmission.get(),
                                    handle_data.act_state_data,
                                    handle_data.jnt_state_data);

  loader_data.transmission_interfaces->act_to_jnt_state.registerHandle(handle);
  return true;
}

bool PositionJointInterfaceProvider::updateJointInterfaces(const TransmissionInfo& transmission_info,
                                                           JointInterfaces&        joint_interfaces,
                                                           RawJointData&           raw_joint_data)
{
  // Setup joint state interface first
  if (!JointStateInterfaceProvider::updateJointInterfaces(transmission_info,
                                                          joint_interfaces,
                                                          raw_joint_data)) {return false;}

  // Register joint on all its hardware interfaces
  BOOST_FOREACH(const JointInfo& joint_info, transmission_info.joints_)
  {
    const std::string& name = joint_info.name_;

    // Do nothing if joint already exists on the hardware interface
    if (hasResource(name, joint_interfaces.position_joint_interface)) {return true;}

    // Update hardware interface
    using hardware_interface::JointHandle;
    const unsigned int id = addJoint(name, raw_joint_data);
    JointHandle handle(joint_interfaces.joint_state_interface.getHandle(joint_info.name_),
                       &raw_joint_data.position_cmd[id]);
    joint_interfaces.position_joint_interface.registerHandle(handle);
  }
  return true;
}

bool PositionJointInterfaceProvider::getJointCommandData(const TransmissionInfo& transmission_info,
                                                         const RawJointData&     raw_joint_data,
                                                         JointData&              jnt_cmd_data)
{
  const unsigned int dim = transmission_info.joints_.size();
  jnt_cmd_data.position.resize(dim);

  for (unsigned int i = 0; i < dim; ++i)
  {
    const std::string& joint_name = transmission_info.joints_[i].name_;
    std::vector<std::string>::const_iterator name_it = std::find(raw_joint_data.names.begin(),
                                                                 raw_joint_data.names.end(),
                                                                 joint_name);
    if (name_it == raw_joint_data.names.end()) {return false;} // Joint name not found!
    const unsigned int id = std::distance(raw_joint_data.names.begin(), name_it);

    // TODO: Get rid of these const casts!
    jnt_cmd_data.position[i] = const_cast<double*>(&(raw_joint_data.position_cmd[id]));
  }

  return true;
}

bool PositionJointInterfaceProvider::getActuatorCommandData(const TransmissionInfo&      transmission_info,
                                                            hardware_interface::RobotHW* robot_hw,
                                                            ActuatorData&                act_cmd_data)
{
  using hardware_interface::PositionActuatorInterface;
  using hardware_interface::ActuatorHandle;

  // Get handles to all required actuators
  std::vector<ActuatorHandle> handles;
  if (!this->getActuatorHandles<PositionActuatorInterface, ActuatorHandle>(transmission_info.actuators_,
                                                                           robot_hw,
                                                                           handles)) {return false;}

  // Populate actuator data
  const unsigned int dim = transmission_info.actuators_.size();
  act_cmd_data.position.resize(dim);

  for (unsigned int i = 0; i < dim; ++i)
  {
    // TODO: Get rid of these const casts!
    act_cmd_data.position[i] = const_cast<double*>(handles[i].getCommandPtr());
  }
  return true;
}

bool PositionJointInterfaceProvider::registerTransmission(TransmissionLoaderData& loader_data,
                                                          TransmissionHandleData& handle_data)
{
  // Setup joint state interface first (if not yet done)
  if (!hasResource(handle_data.name, loader_data.transmission_interfaces->act_to_jnt_state))
  {
    if (!JointStateInterfaceProvider::registerTransmission(loader_data, handle_data)) {return false;}
  }

  // setup command interface
  JointToActuatorPositionHandle handle(handle_data.name,
                                       handle_data.transmission.get(),
                                       handle_data.act_cmd_data,
                                       handle_data.jnt_cmd_data);

  loader_data.transmission_interfaces->jnt_to_act_pos_cmd.registerHandle(handle);
  return true;
}

bool VelocityJointInterfaceProvider::updateJointInterfaces(const TransmissionInfo& transmission_info,
                                                           JointInterfaces&        joint_interfaces,
                                                           RawJointData&           raw_joint_data)
{
  // Setup joint state interface first
  if (!JointStateInterfaceProvider::updateJointInterfaces(transmission_info,
                                                          joint_interfaces,
                                                          raw_joint_data)) {return false;}

  // Register joint on all its hardware interfaces
  BOOST_FOREACH(const JointInfo& joint_info, transmission_info.joints_)
  {
    const std::string& name = joint_info.name_;

    // Do nothing if joint already exists on the hardware interface
    if (hasResource(name, joint_interfaces.velocity_joint_interface)) {return true;}

    // Update hardware interface
    using hardware_interface::JointHandle;
    const unsigned int id = addJoint(name, raw_joint_data);
    JointHandle handle(joint_interfaces.joint_state_interface.getHandle(joint_info.name_),
                       &raw_joint_data.velocity_cmd[id]);
    joint_interfaces.velocity_joint_interface.registerHandle(handle);
  }
  return true;
}

bool VelocityJointInterfaceProvider::getJointCommandData(const TransmissionInfo& transmission_info,
                                                         const RawJointData&     raw_joint_data,
                                                         JointData&              jnt_cmd_data)
{
  const unsigned int dim = transmission_info.joints_.size();
  jnt_cmd_data.velocity.resize(dim);

  for (unsigned int i = 0; i < dim; ++i)
  {
    const std::string& joint_name = transmission_info.joints_[i].name_;
    std::vector<std::string>::const_iterator name_it = std::find(raw_joint_data.names.begin(),
                                                                 raw_joint_data.names.end(),
                                                                 joint_name);
    if (name_it == raw_joint_data.names.end()) {return false;} // Joint name not found!
    const unsigned int id = std::distance(raw_joint_data.names.begin(), name_it);

    // TODO: Get rid of these const casts!
    jnt_cmd_data.velocity[i] = const_cast<double*>(&(raw_joint_data.velocity_cmd[id]));
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
  if (!hasResource(handle_data.name, loader_data.transmission_interfaces->act_to_jnt_state))
  {
    if (!JointStateInterfaceProvider::registerTransmission(loader_data, handle_data)) {return false;}
  }

  // setup command interface
  JointToActuatorVelocityHandle handle(handle_data.name,
                                       handle_data.transmission.get(),
                                       handle_data.act_cmd_data,
                                       handle_data.jnt_cmd_data);

  loader_data.transmission_interfaces->jnt_to_act_vel_cmd.registerHandle(handle);
  return true;
}

bool EffortJointInterfaceProvider::updateJointInterfaces(const TransmissionInfo& transmission_info,
                                                         JointInterfaces&        joint_interfaces,
                                                         RawJointData&           raw_joint_data)
{
  // Setup joint state interface first
  if (!JointStateInterfaceProvider::updateJointInterfaces(transmission_info,
                                                          joint_interfaces,
                                                          raw_joint_data)) {return false;}

  // Register joint on all its hardware interfaces
  BOOST_FOREACH(const JointInfo& joint_info, transmission_info.joints_)
  {
    const std::string& name = joint_info.name_;

    // Do nothing if joint already exists on the hardware interface
    if (hasResource(name, joint_interfaces.effort_joint_interface)) {return true;}

    // Update hardware interface
    using hardware_interface::JointHandle;
    const unsigned int id = addJoint(name, raw_joint_data);
    JointHandle handle(joint_interfaces.joint_state_interface.getHandle(joint_info.name_),
                       &raw_joint_data.effort_cmd[id]);
    joint_interfaces.effort_joint_interface.registerHandle(handle);
  }
  return true;
}

bool EffortJointInterfaceProvider::getJointCommandData(const TransmissionInfo& transmission_info,
                                                       const RawJointData&     raw_joint_data,
                                                       JointData&              jnt_cmd_data)
{
  const unsigned int dim = transmission_info.joints_.size();
  jnt_cmd_data.effort.resize(dim);

  for (unsigned int i = 0; i < dim; ++i)
  {
    const std::string& joint_name = transmission_info.joints_[i].name_;
    std::vector<std::string>::const_iterator name_it = std::find(raw_joint_data.names.begin(),
                                                                 raw_joint_data.names.end(),
                                                                 joint_name);
    if (name_it == raw_joint_data.names.end()) {return false;} // Joint name not found!
    const unsigned int id = std::distance(raw_joint_data.names.begin(), name_it);

    // TODO: Get rid of these const casts!
    jnt_cmd_data.effort[i] = const_cast<double*>(&(raw_joint_data.effort_cmd[id]));
  }

  return true;
}

bool EffortJointInterfaceProvider::getActuatorCommandData(const TransmissionInfo&      transmission_info,
                                                          hardware_interface::RobotHW* robot_hw,
                                                          ActuatorData&                act_cmd_data)
{
  using hardware_interface::EffortActuatorInterface;
  using hardware_interface::ActuatorHandle;

  // Get handles to all required actuators
  std::vector<ActuatorHandle> handles;
  if (!this->getActuatorHandles<EffortActuatorInterface, ActuatorHandle>(transmission_info.actuators_,
                                                                         robot_hw,
                                                                         handles)) {return false;}

  // Populate actuator data
  const unsigned int dim = transmission_info.actuators_.size();
  act_cmd_data.effort.resize(dim);

  for (unsigned int i = 0; i < dim; ++i)
  {
    // TODO: Get rid of these const casts!
    act_cmd_data.effort[i] = const_cast<double*>(handles[i].getCommandPtr());
  }
  return true;
}

bool EffortJointInterfaceProvider::registerTransmission(TransmissionLoaderData& loader_data,
                                                        TransmissionHandleData& handle_data)
{
  // Setup joint state interface first (if not yet done)
  if (!hasResource(handle_data.name, loader_data.transmission_interfaces->act_to_jnt_state))
  {
    if (!JointStateInterfaceProvider::registerTransmission(loader_data, handle_data)) {return false;}
  }

  // setup command interface
  JointToActuatorEffortHandle handle(handle_data.name,
                                     handle_data.transmission.get(),
                                     handle_data.act_cmd_data,
                                     handle_data.jnt_cmd_data);

  loader_data.transmission_interfaces->jnt_to_act_eff_cmd.registerHandle(handle);
  return true;
}

} // namespace

PLUGINLIB_DECLARE_CLASS(hardware_interface,
                        JointStateInterface,
                        transmission_interface::JointStateInterfaceProvider,
                        transmission_interface::RequisiteProvider)

PLUGINLIB_DECLARE_CLASS(hardware_interface,
                        PositionJointInterface,
                        transmission_interface::PositionJointInterfaceProvider,
                        transmission_interface::RequisiteProvider)

PLUGINLIB_DECLARE_CLASS(hardware_interface,
                        VelocityJointInterface,
                        transmission_interface::VelocityJointInterfaceProvider,
                        transmission_interface::RequisiteProvider)

PLUGINLIB_DECLARE_CLASS(hardware_interface,
                        EffortJointInterface,
                        transmission_interface::EffortJointInterfaceProvider,
                        transmission_interface::RequisiteProvider)
