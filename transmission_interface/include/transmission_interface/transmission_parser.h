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
 * \brief Parses <tt>\<transmission\></tt> elements into corresponding structs from XML (URDF).
 * \author Dave Coleman
 */

#pragma once


// ros_control
#include <transmission_interface/transmission_info.h>

// ROS
#include <ros/ros.h>

// XML
#include <tinyxml.h>

namespace transmission_interface
{

/** \brief Parse all transmissions specified in a URDF. */
class TransmissionParser
{
public:

  /**
   * \brief Parses the transmission elements of a URDF
   * \param[in] urdf_string XML string of a valid URDF file that contains <tt>\<transmission\></tt> elements
   * \param[out] transmissions vector of loaded transmission meta data
   * \return true if parsing was successful
   */
  static bool parse(const std::string& urdf_string, std::vector<TransmissionInfo>& transmissions);

protected:
  /**
   * \brief Parses the joint elements within transmission elements of a URDF
   * \param[in] trans_it pointer to the current XML element being parsed
   * \param[out] joints resulting list of joints in the transmission
   * \return true if successful
   */
  static bool parseJoints(TiXmlElement *trans_it, std::vector<JointInfo>& joints);

  /**
   * \brief Parses the actuator elements within transmission elements of a URDF
   * \param[in] trans_it pointer to the current XML element being parsed
   * \param[out] actuators resulting list of actuators in the transmission
   * \return true if successful
   */
  static bool parseActuators(TiXmlElement *trans_it, std::vector<ActuatorInfo>& actuators);

}; // class

} // namespace
