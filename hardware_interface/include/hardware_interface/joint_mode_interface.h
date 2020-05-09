/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
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

/* Author: Dave Coleman
   Desc:   This interface is for switching a hardware interface between different controller modes
           i.e. position, velocity, effort
*/

#pragma once


#include <cassert>
#include <hardware_interface/internal/hardware_resource_manager.h>

namespace hardware_interface
{

  enum class JointCommandModes
  {
    BEGIN = -1,
    MODE_POSITION = 0,
    MODE_VELOCITY = 1,
    MODE_EFFORT = 2,
    NOMODE = 3,
    EMERGENCY_STOP = 4,
    SWITCHING = 5,
    ERROR = 6
  };

  /** \brief A handle used to read and mode a single joint. */
  class JointModeHandle
  {
  public:
    JointModeHandle() = default;

    /** \param mode Which mode to start in */
    JointModeHandle(std::string name, JointCommandModes* mode)
      : mode_(mode)
      , name_(name)
    {
      if (!mode_)
      {
        throw HardwareInterfaceException("Cannot create mode interface. Mode data pointer is null.");
      }
    }

    std::string getName() const {return name_;}

    void setMode(JointCommandModes mode) {assert(mode_); *mode_ = mode;}
    JointCommandModes getMode() const {assert(mode_); return *mode_;}
    const JointCommandModes* getModePtr() const {assert(mode_); return mode_;}

    // Helper function for console messages
    std::string getModeName(JointCommandModes mode) const
    {
      switch(mode)
      {
        case JointCommandModes::BEGIN:
          return "not_operational";
        case JointCommandModes::MODE_POSITION:
          return "position";
        case JointCommandModes::MODE_VELOCITY:
          return "velocity";
        case JointCommandModes::MODE_EFFORT:
          return "effort";
        case JointCommandModes::NOMODE:
          return "other";
        case JointCommandModes::EMERGENCY_STOP:
          return "emergency_stop";
        case JointCommandModes::SWITCHING:
          return "switching";
        case JointCommandModes::ERROR:
          return "error";
      }
      return "unknown";
    }

  private:
    JointCommandModes* mode_ = {nullptr};
    std::string name_;
  };

  /** \brief Hardware interface to support changing between control modes */
  class JointModeInterface : public HardwareResourceManager<JointModeHandle, DontClaimResources> {};

} // namespace
