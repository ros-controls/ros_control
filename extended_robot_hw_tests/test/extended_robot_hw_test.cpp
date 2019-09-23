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

#include "extended_actuator_interface.h"
#include "extended_joint_interface.h"

#include <transmission_interface/simple_transmission.h>
#include <hardware_interface/robot_hw.h>

#include <gtest/gtest.h>

struct ExtendedActuatorData
{
  ExtendedActuatorData()
    : pos_data(0.0)
    , vel_data(0.0)
    , eff_data(0.0)
    , foo_data(0.0)
    , bar_data(0.0)
    , pos_cmd(0.0)
    , vel_cmd(0.0)
    , eff_cmd(0.0)
    , foo_cmd(0.0)
    , bar_cmd(0.0)
  {
  }

  double pos_data;
  double vel_data;
  double eff_data;
  double foo_data;
  double bar_data;

  double pos_cmd;
  double vel_cmd;
  double eff_cmd;
  double foo_cmd;
  double bar_cmd;
};

class ExtendedRobotHW : public hardware_interface::RobotHW
{
public:
  ExtendedRobotHW(ExtendedActuatorData* act_data)
    : hardware_interface::RobotHW(), act_name_("extended_actuator")
  {
    init_robot(act_data);
  }

  virtual ~ExtendedRobotHW()
  {
  }

  void init_robot(ExtendedActuatorData* act_data)
  {
    // for simplicity, robot with one actuator

    // actuator state interface
    act_state_.registerHandle(hardware_interface::ExtendedActuatorStateHandle(
        act_name_, &act_data->pos_data, &act_data->vel_data, &act_data->eff_data,
        &act_data->foo_data, &act_data->bar_data));

    /// @note should we use a new 'ExtendedActuatorHandle' here?

    // actuator command interfaces
    act_pos_cmd_.registerHandle(hardware_interface::ActuatorHandle(
        act_state_.getHandle(act_name_), &act_data->pos_cmd));
    act_vel_cmd_.registerHandle(hardware_interface::ActuatorHandle(
        act_state_.getHandle(act_name_), &act_data->vel_cmd));
    act_eff_cmd_.registerHandle(hardware_interface::ActuatorHandle(
        act_state_.getHandle(act_name_), &act_data->eff_cmd));
    act_foo_cmd_.registerHandle(hardware_interface::ActuatorHandle(
        act_state_.getHandle(act_name_), &act_data->foo_cmd));
    act_bar_cmd_.registerHandle(hardware_interface::ActuatorHandle(
        act_state_.getHandle(act_name_), &act_data->bar_cmd));

    // register all actuator interfaces
    registerInterface(&act_state_);
    registerInterface(&act_pos_cmd_);
    registerInterface(&act_vel_cmd_);
    registerInterface(&act_eff_cmd_);
    registerInterface(&act_foo_cmd_);
    registerInterface(&act_bar_cmd_);



  }

private:
  // actuators
  hardware_interface::ExtendedActuatorStateInterface act_state_;
  hardware_interface::PositionActuatorInterface act_pos_cmd_;
  hardware_interface::VelocityActuatorInterface act_vel_cmd_;
  hardware_interface::EffortActuatorInterface act_eff_cmd_;
  hardware_interface::FooActuatorInterface act_foo_cmd_;
  hardware_interface::BarActuatorInterface act_bar_cmd_;

  // joints
  hardware_interface::PositionJointInterface jnt_pos_cmd_;
  hardware_interface::VelocityJointInterface jnt_vel_cmd_;
  hardware_interface::EffortJointInterface jnt_eff_cmd_;
  hardware_interface::FooJointInterface jnt_foo_cmd_;
  hardware_interface::BarJointInterface jnt_bar_cmd_;

  // transmissions

  std::string act_name_;
};

TEST(RobotHWExtensionTest, ExtensionTest)
{
  ExtendedActuatorData data;

  ExtendedRobotHW robot_hw(&data);

  /// @todo test write actuator data, check equals joint data
  /// @todo test write joint command, check equals actuator command
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
