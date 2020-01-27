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

// C++ standard
#include <algorithm>
#include <cassert>
#include <stdexcept>

// ros_control
#include <transmission_interface/transmission_interface_loader.h>


namespace transmission_interface
{

bool RequisiteProvider::loadTransmissionMaps(const TransmissionInfo& transmission_info,
                                             TransmissionLoaderData& loader_data,
                                             TransmissionSharedPtr   transmission)
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
                                                 loader_data.robot_hw,
                                                 loader_data.joint_interfaces,
                                                 loader_data.raw_joint_data_map);
  if (!jnt_data_ok) {return false;}

  // Joint data
  const bool jnt_state_data_ok = getJointStateData(transmission_info,
                                                   loader_data.raw_joint_data_map,
                                                   handle_data.jnt_state_data);
  if (!jnt_state_data_ok) {return false;}

  const bool jnt_cmd_data_ok = getJointCommandData(transmission_info,
                                                   loader_data.raw_joint_data_map,
                                                   handle_data.jnt_cmd_data);
  if (!jnt_cmd_data_ok) {return false;}

  // Update transmission interface
  loader_data.transmission_data.push_back(transmission);

  // Register transmission
  registerTransmission(loader_data, handle_data);

  return true;
}

TransmissionInterfaceLoader::TransmissionInterfaceLoader(hardware_interface::RobotHW* robot_hw,
                                                         RobotTransmissions*          robot_transmissions)
  : robot_hw_ptr_(robot_hw),
    robot_transmissions_ptr_(robot_transmissions)
{
  // Can throw
  transmission_class_loader_.reset(new TransmissionClassLoader("transmission_interface",
                                                               "transmission_interface::TransmissionLoader"));

  // Can throw
  req_provider_loader_.reset(new RequisiteProviderClassLoader("transmission_interface",
                                                              "transmission_interface::RequisiteProvider"));

  // Throw if invalid
  if (!robot_hw)            {throw std::invalid_argument("Invalid robot hardware pointer.");}
  if (!robot_transmissions) {throw std::invalid_argument("Invalid robot transmissions pointer.");}

  loader_data_.robot_hw            =  robot_hw_ptr_;
  loader_data_.robot_transmissions =  robot_transmissions_ptr_;
}

bool TransmissionInterfaceLoader::load(const std::string& urdf)
{
  TransmissionParser parser;
  std::vector<TransmissionInfo> infos;
  if (!parser.parse(urdf, infos)) {return false;}

  if (infos.empty())
  {
    ROS_ERROR_STREAM_NAMED("parser", "No transmissions were found in the robot description.");
    return false;
  }

  return load(infos);
}

bool TransmissionInterfaceLoader::load(const std::vector<TransmissionInfo>& transmission_info_vec)
{
  for (const auto& info : transmission_info_vec)
  {
    if (!load(info)) {return false;}
  }
  return true;
}

bool TransmissionInterfaceLoader::load(const TransmissionInfo& transmission_info)
{
  // Create transmission instance
  TransmissionSharedPtr transmission;
  try
  {
    TransmissionLoaderSharedPtr transmission_loader = transmission_class_loader_->createUniqueInstance(transmission_info.type_);
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
  for (const auto& jnt_info : transmission_info.joints_)
  {
    // Error out if at least one joint has a different set of hardware interfaces
    if (hw_ifaces_ref.size() != jnt_info.hardware_interfaces_.size() ||
        !std::is_permutation(hw_ifaces_ref.begin(), hw_ifaces_ref.end(),
                                  jnt_info.hardware_interfaces_.begin()))
    {
      ROS_ERROR_STREAM_NAMED("parser",
                             "Failed to load transmission '" << transmission_info.name_ <<
                             "'. It has joints with different hardware interfaces. This is currently unsupported.");
      return false;
    }
  }

  // Load transmission for all specified hardware interfaces
  bool has_at_least_one_hw_iface = false;
  for (const auto& hw_iface : hw_ifaces_ref)
  {
    RequisiteProviderPtr req_provider;
    try
    {
      req_provider = req_provider_loader_->createUniqueInstance(hw_iface);
      if (!req_provider) {continue;}
    }
    catch(pluginlib::LibraryLoadException &ex)
    {
      ROS_WARN_STREAM_NAMED("parser", "Failed to process the '" << hw_iface <<
                             "' hardware interface for transmission '" << transmission_info.name_ <<
                             "'.\n" << ex.what());
      continue;
    }

    const bool load_ok = req_provider->loadTransmissionMaps(transmission_info, loader_data_, transmission);
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

} // namespace
