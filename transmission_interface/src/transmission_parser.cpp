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

#include <transmission_interface/transmission_parser.h>

namespace transmission_interface
{

bool TransmissionParser::parse(const std::string& urdf, std::vector<TransmissionInfo>& transmissions)
{
  // initialize TiXmlDocument doc with a string
  TiXmlDocument doc;
  if (!doc.Parse(urdf.c_str()) && doc.Error())
  {
    ROS_ERROR("Could not load the gazebo_ros_control plugin's"
      " configuration file: %s\n", urdf.c_str());
    return false;
  }

  // Find joints in transmission tags
  TiXmlElement *root = doc.RootElement();

  // Constructs the transmissions by parsing custom xml.
  TiXmlElement *trans_it = NULL;
  for (trans_it = root->FirstChildElement("transmission"); trans_it;
       trans_it = trans_it->NextSiblingElement("transmission"))
  {
    transmission_interface::TransmissionInfo transmission;

    // Transmission name
    if(trans_it->Attribute("name"))
    {
      transmission.name_ = trans_it->Attribute("name");
    }
    else
    {
      ROS_ERROR_STREAM_NAMED("parser","No name attribute for transmission tag!");
      continue;
    }

    // Transmission type
    TiXmlElement *type_child = trans_it->FirstChildElement("type");
    if(!type_child)
    {
      ROS_ERROR_STREAM_NAMED("parser","No type element found for transmission "
        << transmission.name_);
      continue;
    }
    transmission.type_ = type_child->GetText();
    if(transmission.type_.empty())
    {
      ROS_ERROR_STREAM_NAMED("parser","No type string found for transmission "
        << transmission.name_);
      continue;
    }

    // Load joints
    if(!parseJoints(trans_it, transmission.joints_))
    {
      ROS_ERROR_STREAM_NAMED("parser","Failed to load joints for transmission "
        << transmission.name_);
      continue;
    }

    // Load actuators
    if(!parseActuators(trans_it, transmission.actuators_))
    {
      ROS_ERROR_STREAM_NAMED("parser","Failed to load actuators for transmission "
        << transmission.name_);
      continue;
    }

    // Save loaded transmission
    transmissions.push_back(transmission);

  } // end for <transmission>

  if( transmissions.empty() )
  {
    ROS_DEBUG_STREAM_NAMED("parser","No tranmissions found.");
  }

  return true;
}

bool TransmissionParser::parseJoints(TiXmlElement *trans_it, std::vector<JointInfo>& joints)
{
  // Loop through each available joint
  TiXmlElement *joint_it = NULL;
  for (joint_it = trans_it->FirstChildElement("joint"); joint_it;
       joint_it = joint_it->NextSiblingElement("joint"))
  {
    // Create new joint
    transmission_interface::JointInfo joint;

    // Joint xml element
    joint.xml_element_ = joint_it;

    // Joint name
    if(joint_it->Attribute("name"))
    {
      joint.name_ = joint_it->Attribute("name");
    }
    else
    {
      ROS_ERROR_STREAM_NAMED("parser","No name attribute for joint");
      return false;
    }

    // \todo: implement other generic joint properties

    // Add joint to vector
    joints.push_back(joint);
  }

  if(joints.empty())
  {
    ROS_ERROR_STREAM_NAMED("parser","No joint element found");
    return false;
  }

  return true;
}

bool TransmissionParser::parseActuators(TiXmlElement *trans_it, std::vector<ActuatorInfo>& actuators)
{
  // Loop through each available actuator
  TiXmlElement *actuator_it = NULL;
  for (actuator_it = trans_it->FirstChildElement("actuator"); actuator_it;
       actuator_it = actuator_it->NextSiblingElement("actuator"))
  {
    // Create new actuator
    transmission_interface::ActuatorInfo actuator;

    // Actuator xml element
    actuator.xml_element_ = actuator_it;

    // Actuator name
    if(actuator_it->Attribute("name"))
    {
      actuator.name_ = actuator_it->Attribute("name");
    }
    else
    {
      ROS_ERROR_STREAM_NAMED("parser","No name attribute for actuator");
      return false;
    }

    // Hardware interface
    TiXmlElement *hardware_interface_child = actuator_it->FirstChildElement("hardwareInterface");
    if(!hardware_interface_child)
    {
      ROS_ERROR_STREAM_NAMED("parser","No hardware interface element found for actuator '"
        << actuator.name_ << "'");
      continue;
    }
    actuator.hardware_interface_ = hardware_interface_child->GetText();
    if(actuator.hardware_interface_.empty())
    {
      ROS_ERROR_STREAM_NAMED("parser","No hardware interface string found for actuator '"
        << actuator.name_ << "'");
      continue;
    }

    // \todo: implement other generic actuator properties

    // Add actuator to vector
    actuators.push_back(actuator);
  }

  if(actuators.empty())
  {
    ROS_ERROR_STREAM_NAMED("parser","No actuator element found");
    return false;
  }

  return true;
}

} // namespace
