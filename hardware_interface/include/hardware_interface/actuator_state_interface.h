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

/// \author Wim Meeussen

#ifndef HARDWARE_INTERFACE_ACTUATOR_STATE_INTERFACE_H
#define HARDWARE_INTERFACE_ACTUATOR_STATE_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <string>

namespace hardware_interface
{
class PositionActuatorStateHandle
{
  public:
    PositionActuatorStateHandle() = default;

    PositionActuatorStateHandle(const double* pos) : pos_(pos)
    {
        if (!pos)
        {
            throw HardwareInterfaceException("Position data pointer is null.");
        }
    }

    virtual double getPosition() const
    {
        assert(pos_);
        return *pos_;
    }

    virtual const double* getPositionPtr() const
    {
        return pos_;
    }

    virtual std::string getName() const = 0;

  protected:
    const double* pos_ = nullptr;
};

class VelocityActuatorStateHandle
{
  public:
    VelocityActuatorStateHandle() = default;

    VelocityActuatorStateHandle(const double* vel) : vel_(vel)
    {
        if (!vel)
        {
            throw HardwareInterfaceException("Velocity data pointer is null.");
        }
    }

    virtual double getVelocity() const
    {
        assert(vel_);
        return *vel_;
    }

    virtual const double* getVelocityPtr() const
    {
        return vel_;
    }

    virtual std::string getName() const = 0;

  protected:
    const double* vel_ = nullptr;
};

class EffortActuatorStateHandle
{
  public:
    EffortActuatorStateHandle() = default;

    EffortActuatorStateHandle(const double* eff) : eff_(eff)
    {
        if (!eff)
        {
            throw HardwareInterfaceException("Effort data pointer is null.");
        }
    }

    virtual double getEffort() const
    {
        assert(eff_);
        return *eff_;
    }

    virtual const double* getEffortPtr() const
    {
        return eff_;
    }

    virtual std::string getName() const = 0;

  protected:
    const double* eff_ = nullptr;
};

/** A handle used to read the state of a single actuator. */
class ActuatorStateHandle : public PositionActuatorStateHandle,
                            public VelocityActuatorStateHandle,
                            public EffortActuatorStateHandle
{
  public:
    ActuatorStateHandle() = default;

    /**
     * \param name The name of the actuator
     * \param pos A pointer to the storage for this actuator's position
     * \param vel A pointer to the storage for this actuator's velocity
     * \param eff A pointer to the storage for this actuator's effort (force or torque)
     */
    ActuatorStateHandle(const std::string& name, const double* pos, const double* vel, const double* eff)
      try : PositionActuatorStateHandle(pos), VelocityActuatorStateHandle(vel), EffortActuatorStateHandle(eff), name_(name)
    {
    }
    catch(const HardwareInterfaceException& ex)
    {
       throw HardwareInterfaceException("Cannot create handle '" + name + "'. " + ex.what());
    }

    std::string getName() const
    {
        return name_;
    }

  private:
    std::string name_ = "";
};

/** \brief Hardware interface to support reading the state of an array of actuators
 *
 * This \ref HardwareInterface supports reading the state of an array of named
 * actuators, each of which has some position, velocity, and effort (force or
 * torque).
 *
 */
class ActuatorStateInterface : public HardwareResourceManager<ActuatorStateHandle>
{
};
}

#endif
