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
           i.e. position, velocity, force
*/

#ifndef HARDWARE_INTERFACE_JOINT_MODE_INTERFACE_H
#define HARDWARE_INTERFACE_JOINT_MODE_INTERFACE_H

#include <cassert>
#include <hardware_interface/internal/hardware_resource_manager.h>

namespace hardware_interface
{

enum JointCommandModes {
  MODE_POSITION = 1,
  MODE_VELOCITY = 2,
  MODE_EFFORT = 3,
  MODE_OTHER = 4
};

/** \brief A handle used to read and mode a single joint. */
class JointModeHandle
{
public:

  /**
   * \param mode Which mode to start in
   */
  JointModeHandle(std::string name, int* mode)
    : mode_(mode)
    , name_(name)
  {
    if (!mode_)
    {
      throw HardwareInterfaceException("Cannot create mode interface. Mode data pointer is null.");
    }
  }

  std::string getName() const {return name_;}

  void setMode(int mode) {assert(mode_); *mode_ = mode;}
  int getMode() const {assert(mode_); return *mode_;}

  // Helper function for console messages
  std::string getModeName(int mode)
  {
    switch(mode)
    {
      case MODE_POSITION:
        return "position";
      case MODE_VELOCITY:
        return "velocity";
      case MODE_EFFORT:
        return "effort";
      case MODE_OTHER:
        return "other";
    }
    return "unknown";
  }

private:
  int* mode_;
  std::string name_;
};

/** \brief Hardware interface to support changing between control modes
 *
 */
class JointModeInterface : public HardwareResourceManager<JointModeHandle, ClaimResources> {};

} // namespace

#endif
