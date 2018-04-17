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

// ROS
#include <ros/console.h>

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

// ros_control
#include <hardware_interface/internal/demangle_symbol.h>
#include <transmission_interface/four_bar_linkage_transmission.h>
#include <transmission_interface/four_bar_linkage_transmission_loader.h>

namespace transmission_interface
{

TransmissionSharedPtr
FourBarLinkageTransmissionLoader::load(const TransmissionInfo& transmission_info)
{
  // Transmission should contain only one actuator/joint
  if (!checkActuatorDimension(transmission_info, 2)) {return TransmissionSharedPtr();}
  if (!checkJointDimension(transmission_info,    2)) {return TransmissionSharedPtr();}

  // Get actuator and joint configuration sorted by role: [actuator1, actuator2] and [joint1, joint2]
  std::vector<double> act_reduction;
  const bool act_config_ok = getActuatorConfig(transmission_info, act_reduction);
  if (!act_config_ok) {return TransmissionSharedPtr();}

  std::vector<double> jnt_reduction;
  std::vector<double> jnt_offset;
  const bool jnt_config_ok = getJointConfig(transmission_info,
                                            jnt_reduction,
                                            jnt_offset);

  if (!jnt_config_ok) {return TransmissionSharedPtr();}

  // Transmission instance
  try
  {
    TransmissionSharedPtr transmission(new FourBarLinkageTransmission(act_reduction,
                                                                jnt_reduction,
                                                                jnt_offset));
    return transmission;
  }
  catch(const TransmissionInterfaceException& ex)
  {
    using hardware_interface::internal::demangledTypeName;
    ROS_ERROR_STREAM_NAMED("parser", "Failed to construct transmission '" << transmission_info.name_ << "' of type '" <<
                           demangledTypeName<FourBarLinkageTransmission>()<< "'. " << ex.what());
    return TransmissionSharedPtr();
  }
}

bool FourBarLinkageTransmissionLoader::getActuatorConfig(const TransmissionInfo& transmission_info,
                                                         std::vector<double>&    actuator_reduction)
{
  const std::string ACTUATOR1_ROLE = "actuator1";
  const std::string ACTUATOR2_ROLE = "actuator2";

  std::vector<TiXmlElement> act_elements(2,"");
  std::vector<std::string>  act_names(2);
  std::vector<std::string>  act_roles(2);

  for (unsigned int i = 0; i < 2; ++i)
  {
    // Actuator name
    act_names[i] = transmission_info.actuators_[i].name_;

    // Actuator xml element
    act_elements[i] = loadXmlElement(transmission_info.actuators_[i].xml_element_);

    // Populate role string
    std::string& act_role = act_roles[i];
    const ParseStatus act_role_status = getActuatorRole(act_elements[i],
                                                        act_names[i],
                                                        transmission_info.name_,
                                                        true, // Required
                                                        act_role);
    if (act_role_status != SUCCESS) {return false;}

    // Validate role string
    if (ACTUATOR1_ROLE != act_role && ACTUATOR2_ROLE != act_role)
    {
      ROS_ERROR_STREAM_NAMED("parser", "Actuator '" << act_names[i] << "' of transmission '" << transmission_info.name_ <<
                             "' does not specify a valid <role> element. Got '" << act_role << "', expected '" <<
                             ACTUATOR1_ROLE << "' or '" << ACTUATOR2_ROLE << "'.");
      return false;
    }
  }

  // Roles must be different
  if (act_roles[0] == act_roles[1])
  {
    ROS_ERROR_STREAM_NAMED("parser", "Actuators '" << act_names[0] << "' and '" << act_names[1] <<
                           "' of transmission '" << transmission_info.name_ <<
                           "' must have different roles. Both specify '" << act_roles[0] << "'.");
    return false;
  }

  // Indices sorted according to role
  std::vector<unsigned int> id_map(2);
  if (ACTUATOR1_ROLE == act_roles[0])
  {
    id_map[0] = 0;
    id_map[1] = 1;

  }
  else
  {
    id_map[0] = 1;
    id_map[1] = 0;
  }

  // Parse required mechanical reductions
  actuator_reduction.resize(2);
  for (unsigned int i = 0; i < 2; ++i)
  {
    const unsigned int id = id_map[i];
    const ParseStatus reduction_status = getActuatorReduction(act_elements[id],
                                                              act_names[id],
                                                              transmission_info.name_,
                                                              true, // Required
                                                              actuator_reduction[i]);
    if (reduction_status != SUCCESS) {return false;}
  }

  return true;
}

bool FourBarLinkageTransmissionLoader::getJointConfig(const TransmissionInfo& transmission_info,
                                                      std::vector<double>&    joint_reduction,
                                                      std::vector<double>&    joint_offset)
{
  const std::string JOINT1_ROLE = "joint1";
  const std::string JOINT2_ROLE = "joint2";

  std::vector<TiXmlElement> jnt_elements(2,"");
  std::vector<std::string>  jnt_names(2);
  std::vector<std::string>  jnt_roles(2);

  for (unsigned int i = 0; i < 2; ++i)
  {
    // Joint name
    jnt_names[i] = transmission_info.joints_[i].name_;

    // Joint xml element
    jnt_elements[i] = loadXmlElement(transmission_info.joints_[i].xml_element_);

    // Populate role string
    std::string& jnt_role = jnt_roles[i];
    const ParseStatus jnt_role_status = getJointRole(jnt_elements[i],
                                                     jnt_names[i],
                                                     transmission_info.name_,
                                                     true, // Required
                                                     jnt_role);
    if (jnt_role_status != SUCCESS) {return false;}

    // Validate role string
    if (JOINT1_ROLE != jnt_role && JOINT2_ROLE != jnt_role)
    {
      ROS_ERROR_STREAM_NAMED("parser", "Joint '" << jnt_names[i] << "' of transmission '" << transmission_info.name_ <<
                             "' does not specify a valid <role> element. Got '" << jnt_role << "', expected '" <<
                             JOINT1_ROLE << "' or '" << JOINT2_ROLE << "'.");
      return false;
    }
  }

  // Roles must be different
  if (jnt_roles[0] == jnt_roles[1])
  {
    ROS_ERROR_STREAM_NAMED("parser", "Joints '" << jnt_names[0] << "' and '" << jnt_names[1] <<
                           "' of transmission '" << transmission_info.name_ <<
                           "' must have different roles. Both specify '" << jnt_roles[0] << "'.");
    return false;
  }

  // Indices sorted according to role
  std::vector<unsigned int> id_map(2);
  if (JOINT1_ROLE == jnt_roles[0])
  {
    id_map[0] = 0;
    id_map[1] = 1;

  }
  else
  {
    id_map[0] = 1;
    id_map[1] = 0;
  }

  // Joint configuration
  joint_reduction.resize(2, 1.0);
  joint_offset.resize(2, 0.0);
  for (unsigned int i = 0; i < 2; ++i)
  {
    const unsigned int id = id_map[i];

    // Parse optional mechanical reductions. Even though it's optional --and to avoid surprises-- we fail if the element
    // is specified but is of the wrong type
    const ParseStatus reduction_status = getJointReduction(jnt_elements[id],
                                                           jnt_names[id],
                                                           transmission_info.name_,
                                                           false, // Optional
                                                           joint_reduction[i]);
    if (reduction_status == BAD_TYPE) {return false;}

    // Parse optional joint offset. Even though it's optional --and to avoid surprises-- we fail if the element is
    // specified but is of the wrong type
    const ParseStatus offset_status = getJointOffset(jnt_elements[id],
                                                     jnt_names[id],
                                                     transmission_info.name_,
                                                     false, // Optional
                                                     joint_offset[i]);
    if (offset_status == BAD_TYPE) {return false;}
  }

  return true;
}

} // namespace

PLUGINLIB_EXPORT_CLASS(transmission_interface::FourBarLinkageTransmissionLoader,
                       transmission_interface::TransmissionLoader)
