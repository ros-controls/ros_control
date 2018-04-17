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

/// \author Adolfo Rodriguez Tsouroukdissian

#ifndef TRANSMISSION_INTERFACE_TRANSMISSION_LOADER_H
#define TRANSMISSION_INTERFACE_TRANSMISSION_LOADER_H

// C++ standard
#include <algorithm>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

// Boost
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

// TinyXML
#include <tinyxml.h>

// ros_control
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

#include <transmission_interface/robot_transmissions.h>
#include <transmission_interface/transmission_info.h>
#include <transmission_interface/transmission.h>

namespace transmission_interface
{

/**
 * \brief Abstract interface for loading transmission instances from configuration data.
 *
 * It also provides convenience methods for specific transmission loaders to leverage.
 */
class TransmissionLoader
{
public:

  virtual ~TransmissionLoader() {}

  virtual TransmissionSharedPtr load(const TransmissionInfo& transmission_info) = 0;

protected:
  enum ParseStatus
  {
    SUCCESS,
    NO_DATA,
    BAD_TYPE
  };

  static bool checkActuatorDimension(const TransmissionInfo& transmission_info, const unsigned int expected_dim)
  {
    const unsigned int dim = transmission_info.actuators_.size();
    if (expected_dim != dim)
    {
      ROS_ERROR_STREAM_NAMED("parser", "Invalid description for transmission '" << transmission_info.name_ <<
                             "' of type '" << transmission_info.type_ <<
                             "'. Expected " << expected_dim << " actuators, got " << dim << ".");
      return false;
    }
    return true;
  }

  static bool checkJointDimension(const TransmissionInfo& transmission_info, const unsigned int expected_dim)
  {
    const unsigned int dim = transmission_info.joints_.size();
    if (expected_dim != dim)
    {
      ROS_ERROR_STREAM_NAMED("parser", "Invalid description for transmission '" << transmission_info.name_ <<
                             "' of type '" << transmission_info.type_ <<
                             "'. Expected " << expected_dim << " joints, got " << dim << ".");
      return false;
    }
    return true;
  }

  static TiXmlElement loadXmlElement(const std::string& element_str)
  {
    TiXmlElement element("");
    std::stringstream element_stream;
    element_stream << element_str;
    element_stream >> element;
    return element;
  }

  static ParseStatus getActuatorReduction(const TiXmlElement& parent_el,
                                          const std::string&  actuator_name,
                                          const std::string&  transmission_name,
                                          bool                required,
                                          double&             reduction);

  static ParseStatus getJointReduction(const TiXmlElement& parent_el,
                                       const std::string&  joint_name,
                                       const std::string&  transmission_name,
                                       bool                required,
                                       double&             reduction);

  static ParseStatus getJointOffset(const TiXmlElement& parent_el,
                                    const std::string&  joint_name,
                                    const std::string&  transmission_name,
                                    bool                required,
                                    double&             offset);

  static ParseStatus getActuatorRole(const TiXmlElement& parent_el,
                                     const std::string&  actuator_name,
                                     const std::string&  transmission_name,
                                     bool                required,
                                     std::string&        role);

  static ParseStatus getJointRole(const TiXmlElement& parent_el,
                                  const std::string&  joint_name,
                                  const std::string&  transmission_name,
                                  bool                required,
                                  std::string&        role);
};

typedef boost::shared_ptr<TransmissionLoader> TransmissionLoaderSharedPtr;

} // namespace

#endif // header guard
