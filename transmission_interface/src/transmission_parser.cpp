/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <sstream>
#include <transmission_interface/transmission_parser.h>

namespace transmission_interface
{

bool TransmissionParser::parse(const std::string& urdf, std::vector<TransmissionInfo>& transmissions)
{
  // initialize TiXmlDocument doc with a string
  TiXmlDocument doc;
  if (!doc.Parse(urdf.c_str()) && doc.Error())
  {
    ROS_ERROR("Can't parse transmissions. Invalid robot description.");
    return false;
  }

  // Find joints in transmission tags
  TiXmlElement *root = doc.RootElement();

  // Constructs the transmissions by parsing custom xml.
  TiXmlElement *trans_it = nullptr;
  for (trans_it = root->FirstChildElement("transmission"); trans_it;
       trans_it = trans_it->NextSiblingElement("transmission"))
  {
    transmission_interface::TransmissionInfo transmission;

    // Transmission name
    if(trans_it->Attribute("name"))
    {
      transmission.name_ = trans_it->Attribute("name");
      if (transmission.name_.empty())
      {
        ROS_ERROR_STREAM_NAMED("parser","Empty name attribute specified for transmission.");
        continue;
      }
    }
    else
    {
      ROS_ERROR_STREAM_NAMED("parser","No name attribute specified for transmission.");
      continue;
    }

    // Transmission type
    TiXmlElement *type_child = trans_it->FirstChildElement("type");
    if(!type_child)
    {
      ROS_ERROR_STREAM_NAMED("parser","No type element found in transmission '"
        << transmission.name_ << "'.");
      continue;
    }
    if (!type_child->GetText())
    {
      ROS_ERROR_STREAM_NAMED("parser","Skipping empty type element in transmission '"
                             << transmission.name_ << "'.");
      continue;
    }
    transmission.type_ = type_child->GetText();

    // Load joints
    if(!parseJoints(trans_it, transmission.joints_))
    {
      ROS_ERROR_STREAM_NAMED("parser","Failed to load joints for transmission '"
        << transmission.name_ << "'.");
      continue;
    }

    // Load actuators
    if(!parseActuators(trans_it, transmission.actuators_))
    {
      ROS_ERROR_STREAM_NAMED("parser","Failed to load actuators for transmission '"
        << transmission.name_ << "'.");
      continue;
    }

    // Save loaded transmission
    transmissions.push_back(transmission);

  } // end for <transmission>

  if( transmissions.empty() )
  {
    ROS_DEBUG_STREAM_NAMED("parser", "No valid transmissions found.");
  }

  return true;
}

bool TransmissionParser::parseJoints(TiXmlElement *trans_it, std::vector<JointInfo>& joints)
{
  // Loop through each available joint
  TiXmlElement *joint_it = nullptr;
  for (joint_it = trans_it->FirstChildElement("joint"); joint_it;
       joint_it = joint_it->NextSiblingElement("joint"))
  {
    // Create new joint
    transmission_interface::JointInfo joint;

    // Joint name
    if(joint_it->Attribute("name"))
    {
      joint.name_ = joint_it->Attribute("name");
      if (joint.name_.empty())
      {
        ROS_ERROR_STREAM_NAMED("parser","Empty name attribute specified for joint.");
        continue;
      }
    }
    else
    {
      ROS_ERROR_STREAM_NAMED("parser","No name attribute specified for joint.");
      return false;
    }

    TiXmlElement *role_it = joint_it->FirstChildElement("role");
    if(role_it)
    {
      joint.role_ = role_it->GetText() ? role_it->GetText() : std::string();
    }

    // Hardware interfaces (required)
    TiXmlElement *hw_iface_it = nullptr;
    for (hw_iface_it = joint_it->FirstChildElement("hardwareInterface"); hw_iface_it;
         hw_iface_it = hw_iface_it->NextSiblingElement("hardwareInterface"))
    {
      if(!hw_iface_it) {continue;}
      if (!hw_iface_it->GetText())
      {
        ROS_DEBUG_STREAM_NAMED("parser","Skipping empty hardware interface element in joint '"
                               << joint.name_ << "'.");
        continue;
      }
      const std::string hw_iface_name = hw_iface_it->GetText();
      joint.hardware_interfaces_.push_back(hw_iface_name);
    }
    if (joint.hardware_interfaces_.empty())
    {
      ROS_ERROR_STREAM_NAMED("parser","No valid hardware interface element found in joint '"
        << joint.name_ << "'.");
      continue;
    }

    // Joint xml element
    std::stringstream ss;
    ss << *joint_it;
    joint.xml_element_ = ss.str();

    // Add joint to vector
    joints.push_back(joint);
  }

  if(joints.empty())
  {
    ROS_DEBUG_NAMED("parser","No valid joint element found.");
    return false;
  }

  return true;
}

bool TransmissionParser::parseActuators(TiXmlElement *trans_it, std::vector<ActuatorInfo>& actuators)
{
  // Loop through each available actuator
  TiXmlElement *actuator_it = nullptr;
  for (actuator_it = trans_it->FirstChildElement("actuator"); actuator_it;
       actuator_it = actuator_it->NextSiblingElement("actuator"))
  {
    // Create new actuator
    transmission_interface::ActuatorInfo actuator;

    // Actuator name
    if(actuator_it->Attribute("name"))
    {
      actuator.name_ = actuator_it->Attribute("name");
      if (actuator.name_.empty())
      {
        ROS_ERROR_STREAM_NAMED("parser","Empty name attribute specified for actuator.");
        continue;
      }
    }
    else
    {
      ROS_ERROR_STREAM_NAMED("parser","No name attribute specified for actuator.");
      return false;
    }

    // Hardware interfaces (optional)
    TiXmlElement *hw_iface_it = nullptr;
    for (hw_iface_it = actuator_it->FirstChildElement("hardwareInterface"); hw_iface_it;
         hw_iface_it = hw_iface_it->NextSiblingElement("hardwareInterface"))
    {
      if(!hw_iface_it) {continue;}
      if (!hw_iface_it->GetText())
      {
        ROS_DEBUG_STREAM_NAMED("parser","Skipping empty hardware interface element in actuator '"
                               << actuator.name_ << "'.");
        continue;
      }
      const std::string hw_iface_name = hw_iface_it->GetText();
      actuator.hardware_interfaces_.push_back(hw_iface_name);
    }
    if (actuator.hardware_interfaces_.empty())
    {
      ROS_DEBUG_STREAM_NAMED("parser","No valid hardware interface element found in actuator '"
        << actuator.name_ << "'.");
      // continue; // NOTE: Hardware interface is optional, so we keep on going
    }

    // Actuator xml element
    std::stringstream ss;
    ss << *actuator_it;
    actuator.xml_element_ = ss.str();

    // Add actuator to vector
    actuators.push_back(actuator);
  }

  if(actuators.empty())
  {
    ROS_DEBUG_NAMED("parser","No valid actuator element found.");
    return false;
  }

  return true;
}

} // namespace
