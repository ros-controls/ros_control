///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2019, PAL Robotics S.L.
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

/// \author Jordán Palacios

#ifndef HARDWARE_INTERFACE_EXTENDED_ACTUATOR_INTERFACE_H
#define HARDWARE_INTERFACE_EXTENDED_ACTUATOR_INTERFACE_H

#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/actuator_command_interface.h>

namespace hardware_interface
{

// foo handle
class FooActuatorStateHandle
{
public:
  FooActuatorStateHandle() = default;

  FooActuatorStateHandle(const double* foo) : foo_(foo)
  {
  }

  virtual ~FooActuatorStateHandle()
  {
  }

  virtual double getFoo() const
  {
    assert(foo_);
    return *foo_;
  }

protected:
  const double* foo_ = nullptr;
};

// bar handle
class BarActuatorStateHandle
{
public:
  BarActuatorStateHandle() = default;

  BarActuatorStateHandle(const double* bar) : bar_(bar)
  {
  }

  virtual ~BarActuatorStateHandle()
  {
  }

  virtual double getBar() const
  {
    assert(bar_);
    return *bar_;
  }

protected:
  const double* bar_ = nullptr;
};

// actuator state handle + foo + bar
class ExtendedActuatorStateHandle : public ActuatorStateHandle,
                                    public FooActuatorStateHandle,
                                    public BarActuatorStateHandle
{
public:
  ExtendedActuatorStateHandle() = default;

  ExtendedActuatorStateHandle(const std::string& name, const double* pos, const double* vel,
                              const double* eff, const double* foo, const double* bar)
    : ActuatorStateHandle(name, pos, vel, eff)
    , FooActuatorStateHandle(foo)
    , BarActuatorStateHandle(bar)
  {
  }

  std::string getName() const
  {
    return ActuatorStateHandle::getName();
  }
};

class ExtendedActuatorStateInterface : public HardwareResourceManager<ExtendedActuatorStateHandle>
{
};

// actuator foo bar command interfaces
class FooActuatorInterface : public ActuatorCommandInterface
{
};
class BarActuatorInterface : public ActuatorCommandInterface
{
};

}  // namespace hardware_interface

#endif
