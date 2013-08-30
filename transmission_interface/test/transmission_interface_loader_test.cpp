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

/// \author Adolfo Rodriguez Tsouroukdissian

#include <gtest/gtest.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/actuator_command_interface.h>
//#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/transmission_interface_loader.h>
#include "read_file.h"

using namespace transmission_interface;
typedef TransmissionLoader::TransmissionPtr TransmissionPtr;

// Floating-point value comparison threshold
const double EPS = 1e-6;

class TransmissionInterfaceLoaderTest : public ::testing::Test
{
public:
  TransmissionInterfaceLoaderTest()
    : act_pos(0.0),
      act_vel(0.0),
      act_eff(0.0),
      jnt_pos_cmd(0.0),
      jnt_vel_cmd(0.0),
      jnt_eff_cmd(0.0)
  {
    // Populate actuators interface
    {
      hardware_interface::ActuatorStateHandle state_handle("foo_actuator", &act_pos, &act_vel, &act_eff);
      act_state_iface.registerHandle(state_handle);
      robot_hw.registerInterface(&act_state_iface);

      hardware_interface::ActuatorHandle pos_cmd_handle(state_handle, &jnt_pos_cmd);
      pos_act_iface.registerHandle(pos_cmd_handle);
      robot_hw.registerInterface(&pos_act_iface);

      hardware_interface::ActuatorHandle vel_cmd_handle(state_handle, &jnt_vel_cmd);
      vel_act_iface.registerHandle(vel_cmd_handle);
      robot_hw.registerInterface(&vel_act_iface);

      hardware_interface::ActuatorHandle eff_cmd_handle(state_handle, &jnt_eff_cmd);
      eff_act_iface.registerHandle(eff_cmd_handle);
      robot_hw.registerInterface(&eff_act_iface);
    }

    // Transmission loader input
    loader_data.robot_hw         = &robot_hw;
    loader_data.joint_interfaces = &joint_interfaces;
    loader_data.raw_joint_data   = &raw_joint_data;

    loader_data.robot_transmissions     = &robot_transmissions;
    loader_data.transmission_interfaces = &transmission_interfaces;
    loader_data.transmission_data       = transmission_data;
  }

protected:
  double act_pos, act_vel, act_eff, jnt_pos_cmd, jnt_vel_cmd, jnt_eff_cmd;
  hardware_interface::ActuatorStateInterface    act_state_iface;
  hardware_interface::PositionActuatorInterface pos_act_iface;
  hardware_interface::VelocityActuatorInterface vel_act_iface;
  hardware_interface::EffortActuatorInterface   eff_act_iface;

  hardware_interface::RobotHW   robot_hw;
  JointInterfaces               joint_interfaces;
  RawJointData                  raw_joint_data;

  RobotTransmissions            robot_transmissions;
  std::vector<TransmissionPtr>  transmission_data;
  ForwardTransmissionInterfaces transmission_interfaces;

  TransmissionLoaderData loader_data;
};

TEST_F(TransmissionInterfaceLoaderTest, InvalidLoaderData)
{
  // Parse transmission info
  std::vector<TransmissionInfo> infos = parseUrdf("test/urdf/simple_transmission_loader_full.urdf");
  ASSERT_EQ(1, infos.size());
  const TransmissionInfo& info = infos.front();

  TransmissionInterfaceLoader trans_iface_loader;

  // A single missing requisite makes loader fail
  {
    TransmissionLoaderData loader_data_bad = loader_data;
    loader_data_bad.robot_hw = 0;
    ASSERT_FALSE(trans_iface_loader.load(info, loader_data_bad));
  }
  {
    TransmissionLoaderData loader_data_bad = loader_data;
    loader_data_bad.joint_interfaces = 0;
    ASSERT_FALSE(trans_iface_loader.load(info, loader_data_bad));
  }
  {
    TransmissionLoaderData loader_data_bad = loader_data;
    loader_data_bad.raw_joint_data = 0;
    ASSERT_FALSE(trans_iface_loader.load(info, loader_data_bad));
  }
  {
    TransmissionLoaderData loader_data_bad = loader_data;
    loader_data_bad.robot_transmissions = 0;
    ASSERT_FALSE(trans_iface_loader.load(info, loader_data_bad));
  }
  {
    TransmissionLoaderData loader_data_bad = loader_data;
    loader_data_bad.transmission_interfaces = 0;
    ASSERT_FALSE(trans_iface_loader.load(info, loader_data_bad));
  }

  // All missing requisites should be reported to logs
  {
    TransmissionLoaderData loader_data_bad;
    ASSERT_FALSE(trans_iface_loader.load(info, loader_data_bad));
  }
}

TEST_F(TransmissionInterfaceLoaderTest, UnsupportedTransmissionType)
{
  // Parse transmission info
  std::vector<TransmissionInfo> infos = parseUrdf("test/urdf/simple_transmission_loader_full.urdf");
  ASSERT_EQ(1, infos.size());
  TransmissionInfo& info = infos.front();
  info.type_ = "unsupported/transmission_type";

  TransmissionInterfaceLoader trans_iface_loader;
  ASSERT_FALSE(trans_iface_loader.load(info, loader_data));
}

TEST_F(TransmissionInterfaceLoaderTest, UnsupportedHwInterfaceType)
{
  // Parse transmission info
  std::vector<TransmissionInfo> infos = parseUrdf("test/urdf/simple_transmission_loader_full.urdf");
  ASSERT_EQ(1, infos.size());

  // Transmission has only one hardware interface, which is unsupported. Loading should fail
  {
    TransmissionInfo info = infos.front();
    info.joints_.front().hardware_interfaces_.clear();
    info.joints_.front().hardware_interfaces_.resize(1, "unsupported/hw_interface_type");

    TransmissionInterfaceLoader trans_iface_loader;
    EXPECT_FALSE(trans_iface_loader.load(info, loader_data));
  }

  // Transmission has multiple hardware interfaces, of which one is unsupported. Loading should succeed
  // (best-effort policy)
  {
    TransmissionInfo info = infos.front();
    info.joints_.front().hardware_interfaces_.push_back("unsupported/hw_interface_type");

    TransmissionInterfaceLoader trans_iface_loader;
    EXPECT_TRUE(trans_iface_loader.load(info, loader_data));
  }
}

TEST_F(TransmissionInterfaceLoaderTest, UnavailableInterface)
{
  // Parse transmission info
  std::vector<TransmissionInfo> infos = parseUrdf("test/urdf/simple_transmission_loader_full.urdf");
  ASSERT_EQ(1, infos.size());
  const TransmissionInfo& info = infos.front();

  hardware_interface::RobotHW empty_robot_hw; // No actuator interfaces are registered to this robot
  loader_data.robot_hw = &empty_robot_hw;

  TransmissionInterfaceLoader trans_iface_loader;
  ASSERT_FALSE(trans_iface_loader.load(info, loader_data));
}

// We currently can't load a transmission where each joint requires a different set of hardware interfaces
TEST_F(TransmissionInterfaceLoaderTest, UnsupportedFeature)
{
  // Parse transmission info
  std::vector<TransmissionInfo> infos = parseUrdf("test/urdf/transmission_interface_loader_unsupported.urdf");
  ASSERT_EQ(1, infos.size());
  const TransmissionInfo& info = infos.front();

  TransmissionInterfaceLoader trans_iface_loader;
  ASSERT_FALSE(trans_iface_loader.load(info, loader_data));
}

TEST_F(TransmissionInterfaceLoaderTest, SuccessfulLoad)
{
  // Parse transmission info
  std::vector<TransmissionInfo> infos = parseUrdf("test/urdf/simple_transmission_loader_full.urdf");
  ASSERT_EQ(1, infos.size());
  const TransmissionInfo& info = infos.front();

  // Load transmission
  TransmissionInterfaceLoader trans_iface_loader;
  ASSERT_TRUE(trans_iface_loader.load(info, loader_data));

  // TODO: Revisit implementation so this is not needed
  loader_data.robot_hw->registerInterface(&(loader_data.joint_interfaces->joint_state_interface));
  loader_data.robot_hw->registerInterface(&(loader_data.joint_interfaces->position_joint_interface));
  loader_data.robot_hw->registerInterface(&(loader_data.joint_interfaces->velocity_joint_interface));
  loader_data.robot_hw->registerInterface(&(loader_data.joint_interfaces->effort_joint_interface));

  loader_data.robot_transmissions->registerInterface(&(loader_data.transmission_interfaces->act_to_jnt_state));
  loader_data.robot_transmissions->registerInterface(&(loader_data.transmission_interfaces->jnt_to_act_pos_cmd));
  loader_data.robot_transmissions->registerInterface(&(loader_data.transmission_interfaces->jnt_to_act_vel_cmd));
  loader_data.robot_transmissions->registerInterface(&(loader_data.transmission_interfaces->jnt_to_act_eff_cmd));

  using namespace hardware_interface;

  // Actuator handles
  ASSERT_EQ(1, info.actuators_.size());
  PositionActuatorInterface* act_pos_cmd_iface = robot_hw.get<PositionActuatorInterface>();
  VelocityActuatorInterface* act_vel_cmd_iface = robot_hw.get<VelocityActuatorInterface>();
  EffortActuatorInterface*   act_eff_cmd_iface = robot_hw.get<EffortActuatorInterface>();

  ASSERT_TRUE(0 != act_pos_cmd_iface);
  ASSERT_TRUE(0 != act_vel_cmd_iface);
  ASSERT_TRUE(0 != act_eff_cmd_iface);

  ASSERT_NO_THROW(act_pos_cmd_iface->getHandle(info.actuators_.front().name_));
  ASSERT_NO_THROW(act_vel_cmd_iface->getHandle(info.actuators_.front().name_));
  ASSERT_NO_THROW(act_eff_cmd_iface->getHandle(info.actuators_.front().name_));
  ActuatorHandle act_pos_cmd_handle = act_pos_cmd_iface->getHandle(info.actuators_.front().name_);
  ActuatorHandle act_vel_cmd_handle = act_vel_cmd_iface->getHandle(info.actuators_.front().name_);
  ActuatorHandle act_eff_cmd_handle = act_eff_cmd_iface->getHandle(info.actuators_.front().name_);

  // Joint handles
  ASSERT_EQ(1, info.joints_.size());
  PositionJointInterface* pos_jnt_iface = robot_hw.get<PositionJointInterface>();
  VelocityJointInterface* vel_jnt_iface = robot_hw.get<VelocityJointInterface>();
  EffortJointInterface*   eff_jnt_iface = robot_hw.get<EffortJointInterface>();

  ASSERT_TRUE(0 != pos_jnt_iface);
  ASSERT_TRUE(0 != vel_jnt_iface);
  ASSERT_TRUE(0 != eff_jnt_iface);

  ASSERT_NO_THROW(pos_jnt_iface->getHandle(info.joints_.front().name_));
  ASSERT_NO_THROW(vel_jnt_iface->getHandle(info.joints_.front().name_));
  ASSERT_NO_THROW(eff_jnt_iface->getHandle(info.joints_.front().name_));
  JointHandle pos_jnt_handle = pos_jnt_iface->getHandle(info.joints_.front().name_);
  JointHandle vel_jnt_handle = vel_jnt_iface->getHandle(info.joints_.front().name_);
  JointHandle eff_jnt_handle = eff_jnt_iface->getHandle(info.joints_.front().name_);

  // Transmission interfaces
  ActuatorToJointStateInterface*    act_to_jnt_state   = robot_transmissions.get<ActuatorToJointStateInterface>();
  JointToActuatorPositionInterface* jnt_to_act_pos_cmd = robot_transmissions.get<JointToActuatorPositionInterface>();
  JointToActuatorVelocityInterface* jnt_to_act_vel_cmd = robot_transmissions.get<JointToActuatorVelocityInterface>();
  JointToActuatorEffortInterface*   jnt_to_act_eff_cmd = robot_transmissions.get<JointToActuatorEffortInterface>();

  ASSERT_TRUE(0 != act_to_jnt_state);
  ASSERT_TRUE(0 != jnt_to_act_pos_cmd);
  ASSERT_TRUE(0 != jnt_to_act_vel_cmd);
  ASSERT_TRUE(0 != jnt_to_act_eff_cmd);

  // Propagate state forward
  act_pos = 50.0;
  act_vel = -50.0;
  act_eff = 1.0;
  act_to_jnt_state->propagate();
  EXPECT_NEAR( 1.5, pos_jnt_handle.getPosition(), EPS);
  EXPECT_NEAR(-1.0, pos_jnt_handle.getVelocity(), EPS);
  EXPECT_NEAR(50.0, pos_jnt_handle.getEffort(),   EPS);

  // Propagate commands forward
  pos_jnt_handle.setCommand(1.5);
  jnt_to_act_pos_cmd->propagate();
  EXPECT_NEAR(50.0, act_pos_cmd_handle.getPosition(), EPS);

  vel_jnt_handle.setCommand(1.0);
  jnt_to_act_vel_cmd->propagate();
  EXPECT_NEAR(50.0, act_vel_cmd_handle.getPosition(), EPS);

  eff_jnt_handle.setCommand(50.0);
  jnt_to_act_eff_cmd->propagate();
  EXPECT_NEAR(1.0, act_eff_cmd_handle.getEffort(), EPS);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
