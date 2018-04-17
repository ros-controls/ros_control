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

#include <gtest/gtest.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/actuator_command_interface.h>
#include <transmission_interface/transmission_interface_loader.h>
#include "read_file.h"

using namespace transmission_interface;

// Floating-point value comparison threshold
const double EPS = 1e-6;

TEST(IsPermutationTest, IsPermutation)
{
  std::vector<int> a(3);
  a[0] = 0;
  a[1] = 1;
  a[2] = 2;

  std::vector<int> b(3);
  b[0] = 0;
  b[1] = 2;
  b[2] = 1;

  std::vector<int> c(3);
  c[0] = 0;
  c[1] = 1;
  c[2] = 3;

  std::vector<int> d(3, 1);

  EXPECT_TRUE(internal::is_permutation(a.begin(),  a.end(), a.begin()));
  EXPECT_TRUE(internal::is_permutation(a.begin(),  a.end(), b.begin()));
  EXPECT_FALSE(internal::is_permutation(a.begin(), a.end(), c.begin()));
  EXPECT_FALSE(internal::is_permutation(a.begin(), a.end(), d.begin()));
  EXPECT_FALSE(internal::is_permutation(d.begin(), d.end(), a.begin()));
}

class TransmissionInterfaceLoaderTest : public ::testing::Test
{
public:
  TransmissionInterfaceLoaderTest()
    : dim(3),
      act_names(3),
      act_pos(3, 0.0),
      act_vel(3, 0.0),
      act_eff(3, 0.0),
      act_pos_cmd(3, 0.0),
      act_vel_cmd(3, 0.0),
      act_eff_cmd(3, 0.0)
  {
    act_names[0] = "foo_actuator";
    act_names[1] = "bar_actuator";
    act_names[2] = "baz_actuator";

    // Populate actuators interface
    for (unsigned int i = 0; i < dim; ++i)
    {
      hardware_interface::ActuatorStateHandle state_handle(act_names[i], &act_pos[i], &act_vel[i], &act_eff[i]);
      act_state_iface.registerHandle(state_handle);

      hardware_interface::ActuatorHandle pos_cmd_handle(state_handle, &act_pos_cmd[i]);
      pos_act_iface.registerHandle(pos_cmd_handle);

      hardware_interface::ActuatorHandle vel_cmd_handle(state_handle, &act_vel_cmd[i]);
      vel_act_iface.registerHandle(vel_cmd_handle);

      hardware_interface::ActuatorHandle eff_cmd_handle(state_handle, &act_eff_cmd[i]);
      eff_act_iface.registerHandle(eff_cmd_handle);
    }
    robot_hw.registerInterface(&act_state_iface);
    robot_hw.registerInterface(&pos_act_iface);
    robot_hw.registerInterface(&vel_act_iface);
    robot_hw.registerInterface(&eff_act_iface);
  }

protected:
  unsigned int dim;
  std::vector<std::string> act_names;
  std::vector<double> act_pos, act_vel, act_eff, act_pos_cmd, act_vel_cmd, act_eff_cmd;
  hardware_interface::ActuatorStateInterface    act_state_iface;
  hardware_interface::PositionActuatorInterface pos_act_iface;
  hardware_interface::VelocityActuatorInterface vel_act_iface;
  hardware_interface::EffortActuatorInterface   eff_act_iface;

  hardware_interface::RobotHW   robot_hw;
  RobotTransmissions            robot_transmissions;
};

TEST_F(TransmissionInterfaceLoaderTest, InvalidConstruction)
{
  EXPECT_THROW(TransmissionInterfaceLoader(0, &robot_transmissions), std::invalid_argument);
  EXPECT_THROW(TransmissionInterfaceLoader(&robot_hw, 0), std::invalid_argument);
}

TEST_F(TransmissionInterfaceLoaderTest, UnsupportedTransmissionType)
{
  // Parse transmission info
  std::vector<TransmissionInfo> infos = parseUrdf("test/urdf/simple_transmission_loader_full.urdf");
  ASSERT_EQ(1, infos.size());
  TransmissionInfo& info = infos.front();
  info.type_ = "unsupported/transmission_type";

  TransmissionInterfaceLoader trans_iface_loader(&robot_hw, &robot_transmissions);
  ASSERT_FALSE(trans_iface_loader.load(info));
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

    TransmissionInterfaceLoader trans_iface_loader(&robot_hw, &robot_transmissions);
    EXPECT_FALSE(trans_iface_loader.load(info));
  }

  // Transmission has multiple hardware interfaces, of which one is unsupported. Loading should succeed
  // (best-effort policy)
  {
    TransmissionInfo info = infos.front();
    info.joints_.front().hardware_interfaces_.push_back("unsupported/hw_interface_type");

    TransmissionInterfaceLoader trans_iface_loader(&robot_hw, &robot_transmissions);
    EXPECT_TRUE(trans_iface_loader.load(info));
  }
}

TEST_F(TransmissionInterfaceLoaderTest, UnavailableInterface)
{
  // Parse transmission info
  std::vector<TransmissionInfo> infos = parseUrdf("test/urdf/simple_transmission_loader_full.urdf");
  ASSERT_EQ(1, infos.size());
  const TransmissionInfo& info = infos.front();

  hardware_interface::RobotHW empty_robot_hw; // No actuator interfaces are registered to this robot

  TransmissionInterfaceLoader trans_iface_loader(&empty_robot_hw, &robot_transmissions);
  ASSERT_FALSE(trans_iface_loader.load(info));
}

// We currently can't load a transmission where each joint requires a different set of hardware interfaces
TEST_F(TransmissionInterfaceLoaderTest, UnsupportedFeature)
{
  // Parse transmission info
  std::vector<TransmissionInfo> infos = parseUrdf("test/urdf/transmission_interface_loader_unsupported.urdf");
  ASSERT_EQ(2, infos.size());

  // Different hw interface counts
  {
    const TransmissionInfo& info = infos.front();
    TransmissionInterfaceLoader trans_iface_loader(&robot_hw, &robot_transmissions);
    ASSERT_FALSE(trans_iface_loader.load(info));
  }

  // Same hw interface count, but different types
  {
    const TransmissionInfo& info = infos.back();
    TransmissionInterfaceLoader trans_iface_loader(&robot_hw, &robot_transmissions);
    ASSERT_FALSE(trans_iface_loader.load(info));
  }
}

TEST_F(TransmissionInterfaceLoaderTest, HwIfacePermutation)
{
  // Parse transmission info
  std::vector<TransmissionInfo> infos = parseUrdf("test/urdf/transmission_interface_loader_hw_iface_permutation.urdf");
  ASSERT_EQ(1, infos.size());
  const TransmissionInfo& info = infos.front();

  TransmissionInterfaceLoader trans_iface_loader(&robot_hw, &robot_transmissions);
  ASSERT_TRUE(trans_iface_loader.load(info));
}

TEST_F(TransmissionInterfaceLoaderTest, AccessorValidation)
{
  // Parse transmission info
  const std::string urdf_filename = "test/urdf/transmission_interface_loader_valid.urdf";
  std::string urdf;
  ASSERT_TRUE(readFile(urdf_filename, urdf));

  // Load transmissions
  TransmissionInterfaceLoader trans_iface_loader(&robot_hw, &robot_transmissions);
  ASSERT_TRUE(trans_iface_loader.load(urdf)); // NOTE: Using URDF loader

  // Validate raw data accessor
  TransmissionLoaderData* loader_data_ptr = trans_iface_loader.getData();
  ASSERT_TRUE(0 != loader_data_ptr);
  ASSERT_TRUE(&robot_hw == loader_data_ptr->robot_hw);
  ASSERT_TRUE(&robot_transmissions == loader_data_ptr->robot_transmissions);
  ASSERT_EQ(3, loader_data_ptr->raw_joint_data_map.size());
  ASSERT_EQ(6, loader_data_ptr->transmission_data.size()); // Each transmission is added as many times as joint interfaces
}

TEST_F(TransmissionInterfaceLoaderTest, DuplicateTransmissions)
{
  // Parse transmission info
  const std::string urdf_filename = "test/urdf/transmission_interface_loader_duplicate.urdf";
  std::string urdf;
  ASSERT_TRUE(readFile(urdf_filename, urdf));

  // Load transmissions
  TransmissionInterfaceLoader trans_iface_loader(&robot_hw, &robot_transmissions);
  ASSERT_TRUE(trans_iface_loader.load(urdf)); // NOTE: Using URDF loader

  // NOTE: We allow to overwrite an existing transmission in the same way a hardware interface can be overwritten.
  // An informative warning message is printed notifying that a transmission handle has been overwritten.
}

TEST_F(TransmissionInterfaceLoaderTest, SuccessfulLoad)
{
  // Parse transmission info
  const std::string urdf_filename = "test/urdf/transmission_interface_loader_valid.urdf";
  std::string urdf;
  ASSERT_TRUE(readFile(urdf_filename, urdf));

  std::vector<TransmissionInfo> infos = parseUrdf(urdf_filename);
  ASSERT_EQ(2, infos.size());

  // Get info for each transmission
  const TransmissionInfo& info_red = infos.front();
  ASSERT_EQ(1, info_red.actuators_.size());
  ASSERT_EQ(1, info_red.joints_.size());

  const TransmissionInfo& info_diff = infos.back();
  ASSERT_EQ(2, info_diff.actuators_.size());
  ASSERT_EQ(2, info_diff.joints_.size());

  // Load transmissions
  TransmissionInterfaceLoader trans_iface_loader(&robot_hw, &robot_transmissions);
  ASSERT_TRUE(trans_iface_loader.load(urdf)); // NOTE: Using URDF loader

  using namespace hardware_interface;

  // Actuator interfaces
  PositionActuatorInterface* act_pos_cmd_iface = robot_hw.get<PositionActuatorInterface>();
  VelocityActuatorInterface* act_vel_cmd_iface = robot_hw.get<VelocityActuatorInterface>();
  EffortActuatorInterface*   act_eff_cmd_iface = robot_hw.get<EffortActuatorInterface>();
  ASSERT_TRUE(0 != act_pos_cmd_iface);
  ASSERT_TRUE(0 != act_vel_cmd_iface);
  ASSERT_TRUE(0 != act_eff_cmd_iface);

  // Joint interfaces
  PositionJointInterface* pos_jnt_iface = robot_hw.get<PositionJointInterface>();
  VelocityJointInterface* vel_jnt_iface = robot_hw.get<VelocityJointInterface>();
  EffortJointInterface*   eff_jnt_iface = robot_hw.get<EffortJointInterface>();
  ASSERT_TRUE(0 != pos_jnt_iface);
  ASSERT_TRUE(0 != vel_jnt_iface);
  ASSERT_TRUE(0 != eff_jnt_iface);

  // Transmission interfaces
  ActuatorToJointStateInterface*    act_to_jnt_state   = robot_transmissions.get<ActuatorToJointStateInterface>();
  JointToActuatorPositionInterface* jnt_to_act_pos_cmd = robot_transmissions.get<JointToActuatorPositionInterface>();
  JointToActuatorVelocityInterface* jnt_to_act_vel_cmd = robot_transmissions.get<JointToActuatorVelocityInterface>();
  JointToActuatorEffortInterface*   jnt_to_act_eff_cmd = robot_transmissions.get<JointToActuatorEffortInterface>();

  ASSERT_TRUE(0 != act_to_jnt_state);
  ASSERT_TRUE(0 != jnt_to_act_pos_cmd);
  ASSERT_TRUE(0 != jnt_to_act_vel_cmd);
  ASSERT_TRUE(0 != jnt_to_act_eff_cmd);

  // Actuator handles
  ASSERT_NO_THROW(act_pos_cmd_iface->getHandle(info_red.actuators_.front().name_));
  ASSERT_NO_THROW(act_vel_cmd_iface->getHandle(info_red.actuators_.front().name_));
  ASSERT_NO_THROW(act_eff_cmd_iface->getHandle(info_red.actuators_.front().name_));
  ActuatorHandle act_pos_cmd_handle_red = act_pos_cmd_iface->getHandle(info_red.actuators_.front().name_);
  ActuatorHandle act_vel_cmd_handle_red = act_vel_cmd_iface->getHandle(info_red.actuators_.front().name_);
  ActuatorHandle act_eff_cmd_handle_red = act_eff_cmd_iface->getHandle(info_red.actuators_.front().name_);

  ASSERT_NO_THROW(act_pos_cmd_iface->getHandle(info_diff.actuators_.front().name_));
  ASSERT_NO_THROW(act_vel_cmd_iface->getHandle(info_diff.actuators_.front().name_));
  ASSERT_NO_THROW(act_eff_cmd_iface->getHandle(info_diff.actuators_.front().name_));
  ASSERT_NO_THROW(act_pos_cmd_iface->getHandle(info_diff.actuators_.back().name_));
  ASSERT_NO_THROW(act_vel_cmd_iface->getHandle(info_diff.actuators_.back().name_));
  ASSERT_NO_THROW(act_eff_cmd_iface->getHandle(info_diff.actuators_.back().name_));
  ActuatorHandle act_pos_cmd_handle_diff1 = act_pos_cmd_iface->getHandle(info_diff.actuators_.front().name_);
  ActuatorHandle act_vel_cmd_handle_diff1 = act_vel_cmd_iface->getHandle(info_diff.actuators_.front().name_);
  ActuatorHandle act_eff_cmd_handle_diff1 = act_eff_cmd_iface->getHandle(info_diff.actuators_.front().name_);
  ActuatorHandle act_pos_cmd_handle_diff2 = act_pos_cmd_iface->getHandle(info_diff.actuators_.back().name_);
  ActuatorHandle act_vel_cmd_handle_diff2 = act_vel_cmd_iface->getHandle(info_diff.actuators_.back().name_);
  ActuatorHandle act_eff_cmd_handle_diff2 = act_eff_cmd_iface->getHandle(info_diff.actuators_.back().name_);

  // Joint handles
  ASSERT_NO_THROW(pos_jnt_iface->getHandle(info_red.joints_.front().name_));
  ASSERT_NO_THROW(vel_jnt_iface->getHandle(info_red.joints_.front().name_));
  ASSERT_NO_THROW(eff_jnt_iface->getHandle(info_red.joints_.front().name_));
  JointHandle pos_jnt_handle_red = pos_jnt_iface->getHandle(info_red.joints_.front().name_);
  JointHandle vel_jnt_handle_red = vel_jnt_iface->getHandle(info_red.joints_.front().name_);
  JointHandle eff_jnt_handle_red = eff_jnt_iface->getHandle(info_red.joints_.front().name_);

  ASSERT_NO_THROW(pos_jnt_iface->getHandle(info_diff.joints_.front().name_));
  ASSERT_NO_THROW(vel_jnt_iface->getHandle(info_diff.joints_.front().name_));
  ASSERT_NO_THROW(eff_jnt_iface->getHandle(info_diff.joints_.front().name_));
  ASSERT_NO_THROW(pos_jnt_iface->getHandle(info_diff.joints_.back().name_));
  ASSERT_NO_THROW(vel_jnt_iface->getHandle(info_diff.joints_.back().name_));
  ASSERT_NO_THROW(eff_jnt_iface->getHandle(info_diff.joints_.back().name_));
  JointHandle pos_jnt_handle_diff1 = pos_jnt_iface->getHandle(info_diff.joints_.front().name_);
  JointHandle vel_jnt_handle_diff1 = vel_jnt_iface->getHandle(info_diff.joints_.front().name_);
  JointHandle eff_jnt_handle_diff1 = eff_jnt_iface->getHandle(info_diff.joints_.front().name_);
  JointHandle pos_jnt_handle_diff2 = pos_jnt_iface->getHandle(info_diff.joints_.back().name_);
  JointHandle vel_jnt_handle_diff2 = vel_jnt_iface->getHandle(info_diff.joints_.back().name_);
  JointHandle eff_jnt_handle_diff2 = eff_jnt_iface->getHandle(info_diff.joints_.back().name_);

  // Propagate state forward
  act_pos.assign(3,  50.0);
  act_vel.assign(3, -50.0);
  act_eff.assign(3,   1.0);

  act_to_jnt_state->propagate();

  EXPECT_NEAR( 1.5, pos_jnt_handle_red.getPosition(),   EPS);
  EXPECT_NEAR( 1.5, pos_jnt_handle_diff1.getPosition(), EPS);
  EXPECT_NEAR( 0.5, pos_jnt_handle_diff2.getPosition(), EPS);

  EXPECT_NEAR(-1.0, pos_jnt_handle_red.getVelocity(),   EPS);
  EXPECT_NEAR(-1.0, pos_jnt_handle_diff1.getVelocity(), EPS);
  EXPECT_NEAR( 0.0, pos_jnt_handle_diff2.getVelocity(), EPS);

  EXPECT_NEAR( 50.0, pos_jnt_handle_red.getEffort(),     EPS);
  EXPECT_NEAR(100.0, pos_jnt_handle_diff1.getEffort(),  EPS);
  EXPECT_NEAR(  0.0, pos_jnt_handle_diff2.getEffort(),   EPS);

  // Propagate position commands forward
  pos_jnt_handle_red.setCommand(1.5);
  pos_jnt_handle_diff1.setCommand(1.5);
  pos_jnt_handle_diff2.setCommand(0.5);

  jnt_to_act_pos_cmd->propagate();

  EXPECT_NEAR(50.0, act_pos_cmd_handle_red.getPosition(),   EPS);
  EXPECT_NEAR(50.0, act_pos_cmd_handle_diff1.getPosition(), EPS);
  EXPECT_NEAR(50.0, act_pos_cmd_handle_diff2.getPosition(), EPS);

  // Propagate velocity commands forward
  vel_jnt_handle_red.setCommand(1.0);
  vel_jnt_handle_diff1.setCommand(1.0);
  vel_jnt_handle_diff2.setCommand(0.0);

  jnt_to_act_vel_cmd->propagate();

  EXPECT_NEAR(-50.0, act_vel_cmd_handle_red.getVelocity(),   EPS);
  EXPECT_NEAR(-50.0, act_vel_cmd_handle_diff1.getVelocity(), EPS);
  EXPECT_NEAR(-50.0, act_vel_cmd_handle_diff2.getVelocity(), EPS);

  // Propagate effort commands forward
  eff_jnt_handle_red.setCommand(50.0);
  eff_jnt_handle_diff1.setCommand(100.0);
  eff_jnt_handle_diff2.setCommand( 0.0);

  jnt_to_act_eff_cmd->propagate();

  EXPECT_NEAR(1.0, act_eff_cmd_handle_red.getEffort(), EPS);
  EXPECT_NEAR(1.0, act_eff_cmd_handle_diff1.getEffort(), EPS);
  EXPECT_NEAR(1.0, act_eff_cmd_handle_diff2.getEffort(), EPS);
}

TEST_F(TransmissionInterfaceLoaderTest, SuccessfulLoadReversible)
{
  // Parse transmission info
  const std::string urdf_filename = "test/urdf/transmission_interface_loader_bidirectional_valid.urdf";
  std::string urdf;
  ASSERT_TRUE(readFile(urdf_filename, urdf));

  std::vector<TransmissionInfo> infos = parseUrdf(urdf_filename);
  ASSERT_EQ(2, infos.size());

  // Get info for each transmission
  const TransmissionInfo& info_red = infos.front();
  ASSERT_EQ(1, info_red.actuators_.size());
  ASSERT_EQ(1, info_red.joints_.size());

  const TransmissionInfo& info_diff = infos.back();
  ASSERT_EQ(2, info_diff.actuators_.size());
  ASSERT_EQ(2, info_diff.joints_.size());

  // Load transmissions
  TransmissionInterfaceLoader trans_iface_loader(&robot_hw, &robot_transmissions);
  ASSERT_TRUE(trans_iface_loader.load(urdf)); // NOTE: Using URDF loader

  using namespace hardware_interface;

  // Actuator interfaces
  PositionActuatorInterface* act_pos_cmd_iface = robot_hw.get<PositionActuatorInterface>();
  VelocityActuatorInterface* act_vel_cmd_iface = robot_hw.get<VelocityActuatorInterface>();
  EffortActuatorInterface*   act_eff_cmd_iface = robot_hw.get<EffortActuatorInterface>();
  ActuatorStateInterface*    act_state_iface   = robot_hw.get<ActuatorStateInterface>();
  ASSERT_TRUE(0 != act_pos_cmd_iface);
  ASSERT_TRUE(0 != act_vel_cmd_iface);
  ASSERT_TRUE(0 != act_eff_cmd_iface);
  ASSERT_TRUE(0 != act_state_iface);

  // Joint interfaces
  PositionJointInterface* pos_jnt_iface = robot_hw.get<PositionJointInterface>();
  VelocityJointInterface* vel_jnt_iface = robot_hw.get<VelocityJointInterface>();
  EffortJointInterface*   eff_jnt_iface = robot_hw.get<EffortJointInterface>();
  JointStateInterface*    state_jnt_iface = robot_hw.get<JointStateInterface>();
  ASSERT_TRUE(0 != pos_jnt_iface);
  ASSERT_TRUE(0 != vel_jnt_iface);
  ASSERT_TRUE(0 != eff_jnt_iface);

  // Forward Transmission interfaces
  ActuatorToJointStateInterface*    act_to_jnt_state   = robot_transmissions.get<ActuatorToJointStateInterface>();
  JointToActuatorPositionInterface* jnt_to_act_pos_cmd = robot_transmissions.get<JointToActuatorPositionInterface>();
  JointToActuatorVelocityInterface* jnt_to_act_vel_cmd = robot_transmissions.get<JointToActuatorVelocityInterface>();
  JointToActuatorEffortInterface*   jnt_to_act_eff_cmd = robot_transmissions.get<JointToActuatorEffortInterface>();

  // Inverse Transmission interfaces
  JointToActuatorStateInterface*    jnt_to_act_state   = robot_transmissions.get<JointToActuatorStateInterface>();
  ActuatorToJointPositionInterface* act_to_jnt_pos_cmd = robot_transmissions.get<ActuatorToJointPositionInterface>();
  ActuatorToJointVelocityInterface* act_to_jnt_vel_cmd = robot_transmissions.get<ActuatorToJointVelocityInterface>();
  ActuatorToJointEffortInterface*   act_to_jnt_eff_cmd = robot_transmissions.get<ActuatorToJointEffortInterface>();

  ASSERT_TRUE(0 != act_to_jnt_state);
  ASSERT_TRUE(0 != jnt_to_act_pos_cmd);
  ASSERT_TRUE(0 != jnt_to_act_vel_cmd);
  ASSERT_TRUE(0 != jnt_to_act_eff_cmd);

  ASSERT_TRUE(0 != jnt_to_act_state);
  ASSERT_TRUE(0 != act_to_jnt_pos_cmd);
  ASSERT_TRUE(0 != act_to_jnt_vel_cmd);
  ASSERT_TRUE(0 != act_to_jnt_eff_cmd);

  // Actuator handles
  ASSERT_NO_THROW(act_pos_cmd_iface->getHandle(info_red.actuators_.front().name_));
  ASSERT_NO_THROW(act_vel_cmd_iface->getHandle(info_red.actuators_.front().name_));
  ASSERT_NO_THROW(act_eff_cmd_iface->getHandle(info_red.actuators_.front().name_));
  ASSERT_NO_THROW(act_state_iface->getHandle(info_red.actuators_.front().name_));
  ActuatorHandle act_pos_cmd_handle_red = act_pos_cmd_iface->getHandle(info_red.actuators_.front().name_);
  ActuatorHandle act_vel_cmd_handle_red = act_vel_cmd_iface->getHandle(info_red.actuators_.front().name_);
  ActuatorHandle act_eff_cmd_handle_red = act_eff_cmd_iface->getHandle(info_red.actuators_.front().name_);

  ActuatorStateHandle act_state_handle_red = act_state_iface->getHandle(info_red.actuators_.front().name_);

  ASSERT_NO_THROW(act_pos_cmd_iface->getHandle(info_diff.actuators_.front().name_));
  ASSERT_NO_THROW(act_vel_cmd_iface->getHandle(info_diff.actuators_.front().name_));
  ASSERT_NO_THROW(act_eff_cmd_iface->getHandle(info_diff.actuators_.front().name_));
  ASSERT_NO_THROW(act_state_iface->getHandle(info_diff.actuators_.front().name_));
  ASSERT_NO_THROW(act_pos_cmd_iface->getHandle(info_diff.actuators_.back().name_));
  ASSERT_NO_THROW(act_vel_cmd_iface->getHandle(info_diff.actuators_.back().name_));
  ASSERT_NO_THROW(act_eff_cmd_iface->getHandle(info_diff.actuators_.back().name_));
  ASSERT_NO_THROW(act_state_iface->getHandle(info_diff.actuators_.front().name_));
  ActuatorHandle act_pos_cmd_handle_diff1 = act_pos_cmd_iface->getHandle(info_diff.actuators_.front().name_);
  ActuatorHandle act_vel_cmd_handle_diff1 = act_vel_cmd_iface->getHandle(info_diff.actuators_.front().name_);
  ActuatorHandle act_eff_cmd_handle_diff1 = act_eff_cmd_iface->getHandle(info_diff.actuators_.front().name_);
  ActuatorHandle act_pos_cmd_handle_diff2 = act_pos_cmd_iface->getHandle(info_diff.actuators_.back().name_);
  ActuatorHandle act_vel_cmd_handle_diff2 = act_vel_cmd_iface->getHandle(info_diff.actuators_.back().name_);
  ActuatorHandle act_eff_cmd_handle_diff2 = act_eff_cmd_iface->getHandle(info_diff.actuators_.back().name_);

  ActuatorStateHandle act_state_handle_diff1 = act_state_iface->getHandle(info_diff.actuators_.front().name_);
  ActuatorStateHandle act_state_handle_diff2 = act_state_iface->getHandle(info_diff.actuators_.back().name_);

  // Joint handles
  ASSERT_NO_THROW(pos_jnt_iface->getHandle(info_red.joints_.front().name_));
  ASSERT_NO_THROW(vel_jnt_iface->getHandle(info_red.joints_.front().name_));
  ASSERT_NO_THROW(eff_jnt_iface->getHandle(info_red.joints_.front().name_));
  ASSERT_NO_THROW(state_jnt_iface->getHandle(info_red.joints_.front().name_));
  JointHandle pos_jnt_handle_red = pos_jnt_iface->getHandle(info_red.joints_.front().name_);
  JointHandle vel_jnt_handle_red = vel_jnt_iface->getHandle(info_red.joints_.front().name_);
  JointHandle eff_jnt_handle_red = eff_jnt_iface->getHandle(info_red.joints_.front().name_);

  JointStateHandle state_jnt_handle_red = state_jnt_iface->getHandle(info_red.joints_.front().name_);

  ASSERT_NO_THROW(pos_jnt_iface->getHandle(info_diff.joints_.front().name_));
  ASSERT_NO_THROW(vel_jnt_iface->getHandle(info_diff.joints_.front().name_));
  ASSERT_NO_THROW(eff_jnt_iface->getHandle(info_diff.joints_.front().name_));
  ASSERT_NO_THROW(pos_jnt_iface->getHandle(info_diff.joints_.back().name_));
  ASSERT_NO_THROW(vel_jnt_iface->getHandle(info_diff.joints_.back().name_));
  ASSERT_NO_THROW(eff_jnt_iface->getHandle(info_diff.joints_.back().name_));
  JointHandle pos_jnt_handle_diff1 = pos_jnt_iface->getHandle(info_diff.joints_.front().name_);
  JointHandle vel_jnt_handle_diff1 = vel_jnt_iface->getHandle(info_diff.joints_.front().name_);
  JointHandle eff_jnt_handle_diff1 = eff_jnt_iface->getHandle(info_diff.joints_.front().name_);
  JointHandle pos_jnt_handle_diff2 = pos_jnt_iface->getHandle(info_diff.joints_.back().name_);
  JointHandle vel_jnt_handle_diff2 = vel_jnt_iface->getHandle(info_diff.joints_.back().name_);
  JointHandle eff_jnt_handle_diff2 = eff_jnt_iface->getHandle(info_diff.joints_.back().name_);

  // Propagate state forward
  act_pos.assign(3,  50.0);
  act_vel.assign(3, -50.0);
  act_eff.assign(3,   1.0);

  act_to_jnt_state->propagate();

  EXPECT_NEAR(1.5, pos_jnt_handle_red.getPosition(),   EPS);
  EXPECT_NEAR(1.5, pos_jnt_handle_diff1.getPosition(), EPS);
  EXPECT_NEAR(0.5, pos_jnt_handle_diff2.getPosition(), EPS);

  EXPECT_NEAR(-1.0, pos_jnt_handle_red.getVelocity(),   EPS);
  EXPECT_NEAR(-1.0, pos_jnt_handle_diff1.getVelocity(), EPS);
  EXPECT_NEAR( 0.0, pos_jnt_handle_diff2.getVelocity(), EPS);

  EXPECT_NEAR(50.0, pos_jnt_handle_red.getEffort(),     EPS);
  EXPECT_NEAR(100.0, pos_jnt_handle_diff1.getEffort(),  EPS);
  EXPECT_NEAR(0.0, pos_jnt_handle_diff2.getEffort(),    EPS);

  // Propagate position commands forward
  pos_jnt_handle_red.setCommand(1.5);
  pos_jnt_handle_diff1.setCommand(1.5);
  pos_jnt_handle_diff2.setCommand(0.5);

  jnt_to_act_pos_cmd->propagate();

  EXPECT_NEAR(50.0, act_pos_cmd_handle_red.getPosition(),   EPS);
  EXPECT_NEAR(50.0, act_pos_cmd_handle_diff1.getPosition(), EPS);
  EXPECT_NEAR(50.0, act_pos_cmd_handle_diff2.getPosition(), EPS);

  // Propagate velocity commands forward
  vel_jnt_handle_red.setCommand(1.0);
  vel_jnt_handle_diff1.setCommand(1.0);
  vel_jnt_handle_diff2.setCommand(0.0);

  jnt_to_act_vel_cmd->propagate();

  EXPECT_NEAR(-50.0, act_vel_cmd_handle_red.getVelocity(),   EPS);
  EXPECT_NEAR(-50.0, act_vel_cmd_handle_diff1.getVelocity(), EPS);
  EXPECT_NEAR(-50.0, act_vel_cmd_handle_diff2.getVelocity(), EPS);

  // Propagate effort commands forward
  eff_jnt_handle_red.setCommand(50.0);
  eff_jnt_handle_diff1.setCommand(100.0);
  eff_jnt_handle_diff2.setCommand( 0.0);

  jnt_to_act_eff_cmd->propagate();

  EXPECT_NEAR(1.0, act_eff_cmd_handle_red.getEffort(),   EPS);
  EXPECT_NEAR(1.0, act_eff_cmd_handle_diff1.getEffort(), EPS);
  EXPECT_NEAR(1.0, act_eff_cmd_handle_diff2.getEffort(), EPS);

  // Now propegate things in the reverse direction
  RawJointDataMap* joint_data_map = &trans_iface_loader.getData()->raw_joint_data_map;
  joint_data_map->operator[]("foo_joint").position = 1.5;
  joint_data_map->operator[]("bar_joint").position = 1.5;
  joint_data_map->operator[]("baz_joint").position = 0.5;

  joint_data_map->operator[]("foo_joint").velocity = -1.0;
  joint_data_map->operator[]("bar_joint").velocity = -1.0;
  joint_data_map->operator[]("baz_joint").velocity = -2.0;

  joint_data_map->operator[]("foo_joint").effort = 5.0;
  joint_data_map->operator[]("bar_joint").effort = 10.0;
  joint_data_map->operator[]("baz_joint").effort = -5.0;

  jnt_to_act_state->propagate();

  EXPECT_NEAR(50.0, act_state_handle_red.getPosition(),   EPS);
  EXPECT_NEAR(50.0, act_state_handle_diff1.getPosition(), EPS);
  EXPECT_NEAR(50.0, act_state_handle_diff2.getPosition(), EPS);

  EXPECT_NEAR(-50.0,  act_state_handle_red.getVelocity(),   EPS);
  EXPECT_NEAR(-150.0, act_state_handle_diff1.getVelocity(), EPS);
  EXPECT_NEAR( 50.0,  act_state_handle_diff2.getVelocity(), EPS);

  EXPECT_NEAR(0.1,  act_state_handle_red.getEffort(),   EPS);
  EXPECT_NEAR(0.05, act_state_handle_diff1.getEffort(), EPS);
  EXPECT_NEAR(0.15, act_state_handle_diff2.getEffort(), EPS);

  act_pos_cmd_handle_red.setCommand(3.0);
  act_pos_cmd_handle_diff1.setCommand(3.0);
  act_pos_cmd_handle_diff2.setCommand(3.0);

    // reverse propegate position commands
  act_to_jnt_pos_cmd->propagate();

  EXPECT_NEAR(0.56, joint_data_map->operator[]("foo_joint").position_cmd, EPS);
  EXPECT_NEAR(0.56, joint_data_map->operator[]("bar_joint").position_cmd, EPS);
  EXPECT_NEAR(0.5,  joint_data_map->operator[]("baz_joint").position_cmd, EPS);

  // Propagate velocity commands forward
  act_vel_cmd_handle_red.setCommand(1.0);
  act_vel_cmd_handle_diff1.setCommand(1.0);
  act_vel_cmd_handle_diff2.setCommand(0.0);

  act_to_jnt_vel_cmd->propagate();

  EXPECT_NEAR(0.02, joint_data_map->operator[]("foo_joint").velocity_cmd, EPS);
  EXPECT_NEAR(0.01, joint_data_map->operator[]("bar_joint").velocity_cmd, EPS);
  EXPECT_NEAR(0.01, joint_data_map->operator[]("baz_joint").velocity_cmd, EPS);

   // Propagate effort commands forward
  act_eff_cmd_handle_red.setCommand(50.0);
  act_eff_cmd_handle_diff1.setCommand(1.0);
  act_eff_cmd_handle_diff2.setCommand( 0.0);

  act_to_jnt_eff_cmd->propagate();

  EXPECT_NEAR(2500.0, joint_data_map->operator[]("foo_joint").effort_cmd, EPS);
  EXPECT_NEAR(50.0, joint_data_map->operator[]("bar_joint").effort_cmd,   EPS);
  EXPECT_NEAR(50.0, joint_data_map->operator[]("baz_joint").effort_cmd,   EPS);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
