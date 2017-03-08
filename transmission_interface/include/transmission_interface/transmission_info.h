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
/**
 * \file
 * \brief Structs to hold tranmission data loaded straight from XML (URDF).
 * \author Dave Coleman
 */

#ifndef TRANSMISSION_INTERFACE_TRANSMISSION_INTERFACE_INFO_H
#define TRANSMISSION_INTERFACE_TRANSMISSION_INTERFACE_INFO_H

// C++ standard
#include <vector>
#include <string>

// TinyXML
#include <tinyxml.h>

namespace transmission_interface
{

/**
 * \brief Contains semantic info about a given joint loaded from XML (URDF)
 */
struct JointInfo
{
  std::string name_;
  std::vector<std::string> hardware_interfaces_;
  std::string role_;
  std::string xml_element_;
};

/**
 * \brief Contains semantic info about a given actuator loaded from XML (URDF)
 */
struct ActuatorInfo
{
  std::string name_;
  std::vector<std::string> hardware_interfaces_;
  std::string xml_element_;
};

/**
 * \brief Contains semantic info about a given transmission loaded from XML (URDF)
 */
struct TransmissionInfo
{
  std::string name_;
  std::string type_;
  std::vector<JointInfo> joints_;
  std::vector<ActuatorInfo> actuators_;
};

} // namespace

#endif
