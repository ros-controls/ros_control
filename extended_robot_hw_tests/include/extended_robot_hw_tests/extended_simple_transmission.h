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

/// \author Jordán Palacios

#ifndef TRANSMISSION_INTERFACE_EXTENDED_SIMPLE_TRANSMISSION_H
#define TRANSMISSION_INTERFACE_EXTENDED_SIMPLE_TRANSMISSION_H

#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/transmission_interface_exception.h>
#include <transmission_interface/transmission.h>

#include <vector>

namespace transmission_interface
{
struct FooActuatorData
{
  std::vector<double*> foo;
};

struct BarActuatorData
{
  std::vector<double*> bar;
};

struct FooJointData
{
  std::vector<double*> foo;
};

struct BarJointData
{
  std::vector<double*> bar;
};

class ExtendedSimpleTransmission : public SimpleTransmission
{
public:
  ExtendedSimpleTransmission(const double reduction, const double joint_offset = 0.0)
    : SimpleTransmission(reduction, joint_offset), reduction_(reduction)
  {
  }

  virtual ~ExtendedSimpleTransmission()
  {
  }

  // bring base functions into scope
  using SimpleTransmission::actuatorToJoint;
  using SimpleTransmission::jointToActuator;

  void actuatorToJoint(const FooActuatorData& act_data, FooJointData& jnt_data)
  {
    assert(numActuators() == act_data.foo.size() && numJoints() == jnt_data.foo.size());
    assert(act_data.foo[0] && jnt_data.foo[0]);
    *jnt_data.foo[0] = *act_data.foo[0] * reduction_;
  }

  void actuatorToJoint(const BarActuatorData& act_data, BarJointData& jnt_data)
  {
    assert(numActuators() == act_data.bar.size() && numJoints() == jnt_data.bar.size());
    assert(act_data.bar[0] && jnt_data.bar[0]);
    *jnt_data.bar[0] = *act_data.bar[0] * reduction_;
  }

  void jointToActuator(const FooJointData& jnt_data, FooActuatorData& act_data)
  {
    assert(numActuators() == act_data.foo.size() && numJoints() == jnt_data.foo.size());
    assert(act_data.foo[0] && jnt_data.foo[0]);
    *act_data.foo[0] = *jnt_data.foo[0] * reduction_;
  }

  void jointToActuator(const BarJointData& jnt_data, BarActuatorData& act_data)
  {
    assert(numActuators() == act_data.bar.size() && numJoints() == jnt_data.bar.size());
    assert(act_data.bar[0] && jnt_data.bar[0]);
    *act_data.bar[0] = *jnt_data.bar[0] * reduction_;
  }

private:
  double reduction_;
};

}  // namespace transmission_interface

#endif  // TRANSMISSION_INTERFACE_EXTENDED_SIMPLE_TRANSMISSION_H
