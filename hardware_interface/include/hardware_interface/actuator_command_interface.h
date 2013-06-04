///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012, hiDOF INC.
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

#ifndef HARDWARE_INTERFACE_ACTUATOR_COMMAND_INTERFACE_H
#define HARDWARE_INTERFACE_ACTUATOR_COMMAND_INTERFACE_H

#include <hardware_interface/internal/demangle_symbol.h>
#include <hardware_interface/internal/named_resource_manager.h>
#include <hardware_interface/actuator_state_interface.h>


namespace hardware_interface
{

/** \brief A handle used to read and command a single actuator. */
class ActuatorHandle : public ActuatorStateHandle
{
public:
  ActuatorHandle() {}
  ActuatorHandle(const ActuatorStateHandle& js, double* cmd)
    : ActuatorStateHandle(js), cmd_(cmd)
  {}
  void setCommand(double command) {*cmd_ = command;}

private:
  double* cmd_;
};


/** \brief Hardware interface to support commanding an array of actuators
 *
 * This \ref HardwareInterface supports commanding the output of an array of
 * named actuators. Note that these commands can have any semantic meaning as long
 * as they each can be represented by a single double, they are not necessarily
 * effort commands. To specify a meaning to this command, see the derived
 * classes like \ref EffortActuatorInterface etc.
 *
 */
class ActuatorCommandInterface : public hardware_interface::HardwareInterface
{
public:
  /// Get the vector of actuator names registered to this interface.
  std::vector<std::string> getActuatorNames() const
  {
    return handle_map_.getNames();
  }

  /** \brief Register a new actuator with this interface.
   *
   * \param name The name of the new actuator
   * \param cmd A pointer to the storage for this actuator's output command
   */
  void registerActuator(const ActuatorStateHandle& as, double* cmd)
  {
    ActuatorHandle handle(as, cmd);
    handle_map_.insert(as.getName(), handle);
  }

  /** \brief Get a \ref ActuatorHandle for accessing a actuator's state and setting
   * its output command.
   *
   * When a \ref ActuatorHandle is acquired, this interface will claim the actuator
   * as a resource.
   *
   * \param name The name of the actuator
   *
   * \returns A \ref ActuatorHandle corresponding to the actuator identified by \c name
   *
   */
  ActuatorHandle getActuatorHandle(const std::string& name)
  {
    try
    {
      return handle_map_.get(name);
    }
    catch(...)
    {
      throw HardwareInterfaceException("Could not find actuator [" + name + "] in " + internal::demangledTypeName(*this));
    }
  }

protected:
  internal::NamedResourceManager<ActuatorHandle> handle_map_;
};

/// \ref ActuatorCommandInterface for commanding effort-based actuators
class EffortActuatorInterface : public ActuatorCommandInterface {};

/// \ref ActuatorCommandInterface for commanding velocity-based actuators
class VelocityActuatorInterface : public ActuatorCommandInterface {};

/// \ref ActuatorCommandInterface for commanding position-based actuators
class PositionActuatorInterface : public ActuatorCommandInterface {};

}

#endif
