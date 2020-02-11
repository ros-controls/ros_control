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

#include <list>
#include <set>
#include <string>
#include <gtest/gtest.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

using std::list;
using std::string;
using namespace hardware_interface;

class RobotHWTest : public ::testing::Test
{
protected:
  double pos1 = {1.0}, vel1 = {2.0}, eff1 = {3.0}, cmd1 = {0.0};
  double pos2 = {4.0}, vel2 = {5.0}, eff2 = {6.0}, cmd2 = {0.0};
  string name1 = {"name_1"};
  string name2 = {"name_2"};
  JointStateHandle hs1 = {name1, &pos1, &vel1, &eff1};
  JointStateHandle hs2 = {name2, &pos2, &vel2, &eff2};
  JointHandle hc1 = {hs1, &cmd1};
  JointHandle hc2 = {hs2, &cmd2};
};

TEST_F(RobotHWTest, InterfaceRegistration)
{
  // Populate hardware interfaces
  JointStateInterface state_iface;
  state_iface.registerHandle(hs1);
  state_iface.registerHandle(hs2);

  EffortJointInterface eff_cmd_iface;
  eff_cmd_iface.registerHandle(hc1);
  eff_cmd_iface.registerHandle(hc2);

  PositionJointInterface pos_cmd_iface;
  pos_cmd_iface.registerHandle(hc1);
  pos_cmd_iface.registerHandle(hc2);

  // Register them to a RobotHW instance
  RobotHW hw;
  hw.registerInterface(&state_iface);
  hw.registerInterface(&eff_cmd_iface);
  hw.registerInterface(&pos_cmd_iface);

  // Get interfaces
  EXPECT_TRUE(&state_iface   == hw.get<JointStateInterface>());
  EXPECT_TRUE(&eff_cmd_iface == hw.get<EffortJointInterface>());
  EXPECT_TRUE(&pos_cmd_iface == hw.get<PositionJointInterface>());
  EXPECT_FALSE(hw.get<VelocityJointInterface>());
}

TEST_F(RobotHWTest, InterfaceRewriting)
{
  // Two hardware interfaces of the same type, with different joints
  JointStateInterface state1_iface;
  state1_iface.registerHandle(hs1);

  JointStateInterface state2_iface;
  state2_iface.registerHandle(hs1);
  state2_iface.registerHandle(hs2);

  // Register first interface and validate it
  RobotHW hw;
  hw.registerInterface(&state1_iface);

  JointStateInterface* state_iface_ptr = hw.get<JointStateInterface>();
  EXPECT_EQ(1, state_iface_ptr->getNames().size());

  // Register second interface and verify that it has taken the place of the previously inserted one
  hw.registerInterface(&state2_iface);
  state_iface_ptr = hw.get<JointStateInterface>();
  EXPECT_EQ(2, state_iface_ptr->getNames().size());
}

TEST_F(RobotHWTest, ConflictChecking)
{
  // Controller with one interface claiming one resource
  ControllerInfo info1;
  info1.name = "controller_1";
  info1.type = "type_1";
  std::set<string> resources_1;
  resources_1.insert("resource_1");
  InterfaceResources ir_1("interface_1", resources_1);
  info1.claimed_resources.push_back(ir_1);

  // Controller with one interface claiming one resource
  ControllerInfo info2;
  info2.name = "controller_2";
  info2.type = "type_2";
  std::set<string> resources_2;
  resources_2.insert("resource_2");
  InterfaceResources ir_2("interface_2", resources_2);
  info2.claimed_resources.push_back(ir_2);

  // Controller with one interface claiming two resources
  ControllerInfo info12;
  info12.name = "controller_12";
  info12.type = "type_12";
  std::set<string> resources_12;
  resources_12.insert("resource_1");
  resources_12.insert("resource_2");
  InterfaceResources ir_12("interface_1", resources_12);
  info12.claimed_resources.push_back(ir_12);

  // Controller with two interfaces claiming one resource each
  ControllerInfo info12_multi_iface;
  info12_multi_iface.name = "controller_12_multi_iface";
  info12_multi_iface.type = "type_12_multi_iface";
  info12_multi_iface.claimed_resources.push_back(ir_1);
  info12_multi_iface.claimed_resources.push_back(ir_2);

  // Controller with two interfaces claiming one or more resources
  ControllerInfo info12_multi_iface_dup;
  info12_multi_iface_dup.name = "controller_12_multi_iface_dup";
  info12_multi_iface_dup.type = "type_12_multi_iface";
  info12_multi_iface_dup.claimed_resources.push_back(ir_1);
  info12_multi_iface_dup.claimed_resources.push_back(ir_12);

  // No conflict
  {
    list<ControllerInfo> info_list;
    info_list.push_back(info1);
    info_list.push_back(info2);

    RobotHW hw;
    EXPECT_FALSE(hw.checkForConflict(info_list));
  }

  // No conflict
  {
    list<ControllerInfo> info_list;
    info_list.push_back(info12_multi_iface);

    RobotHW hw;
    EXPECT_FALSE(hw.checkForConflict(info_list));
  }

  // Conflict
  {
    list<ControllerInfo> info_list;
    info_list.push_back(info1);
    info_list.push_back(info12);

    RobotHW hw;
    EXPECT_TRUE(hw.checkForConflict(info_list));
  }

  // Conflict
  {
    list<ControllerInfo> info_list;
    info_list.push_back(info2);
    info_list.push_back(info12);

    RobotHW hw;
    EXPECT_TRUE(hw.checkForConflict(info_list));
  }

  // Conflict
  {
    list<ControllerInfo> info_list;
    info_list.push_back(info1);
    info_list.push_back(info12_multi_iface);

    RobotHW hw;
    EXPECT_TRUE(hw.checkForConflict(info_list));
  }

  // Conflict
  {
    list<ControllerInfo> info_list;
    info_list.push_back(info2);
    info_list.push_back(info12_multi_iface);

    RobotHW hw;
    EXPECT_TRUE(hw.checkForConflict(info_list));
  }

  // Conflict: Single controller claims same resource in more than one interface
  {
    list<ControllerInfo> info_list;
    info_list.push_back(info12_multi_iface_dup);

    RobotHW hw;
    EXPECT_TRUE(hw.checkForConflict(info_list));
  }
}

TEST_F(RobotHWTest, CombineDifferentInterfaces)
{
  // Populate hardware interfaces
  JointStateInterface state_iface;
  state_iface.registerHandle(hs1);
  state_iface.registerHandle(hs2);

  EffortJointInterface eff_cmd_iface;
  eff_cmd_iface.registerHandle(hc1);
  eff_cmd_iface.registerHandle(hc2);

  PositionJointInterface pos_cmd_iface;
  pos_cmd_iface.registerHandle(hc1);
  pos_cmd_iface.registerHandle(hc2);

  // Register them to different RobotHW instances
  RobotHW hw1, hw2;
  RobotHW hw_grp;
  hw1.registerInterface(&state_iface);
  hw2.registerInterface(&eff_cmd_iface);
  hw_grp.registerInterface(&pos_cmd_iface);

  hw_grp.registerInterfaceManager(&hw1);
  hw_grp.registerInterfaceManager(&hw2);

  // Get interfaces
  EXPECT_TRUE(&state_iface   == hw_grp.get<JointStateInterface>());
  EXPECT_TRUE(&eff_cmd_iface == hw_grp.get<EffortJointInterface>());
  EXPECT_TRUE(&pos_cmd_iface == hw_grp.get<PositionJointInterface>());
  EXPECT_FALSE(hw_grp.get<VelocityJointInterface>());
}

TEST_F(RobotHWTest, CombineSameInterfaces)
{
  // Populate hardware interfaces
  JointStateInterface state_iface1;
  state_iface1.registerHandle(hs1);
  EffortJointInterface eff_cmd_iface1;
  eff_cmd_iface1.registerHandle(hc1);

  JointStateInterface state_iface2;
  state_iface2.registerHandle(hs2);
  EffortJointInterface eff_cmd_iface2;
  eff_cmd_iface2.registerHandle(hc2);

  // Register them to different RobotHW instances
  RobotHW hw1, hw2;
  RobotHW hw_grp;
  hw1.registerInterface(&state_iface1);
  hw1.registerInterface(&eff_cmd_iface1);
  hw2.registerInterface(&state_iface2);
  hw2.registerInterface(&eff_cmd_iface2);

  hw_grp.registerInterfaceManager(&hw1);
  hw_grp.registerInterfaceManager(&hw2);

  // Get interfaces
  JointStateInterface* js_combo = hw_grp.get<JointStateInterface>();
  EffortJointInterface* ej_combo = hw_grp.get<EffortJointInterface>();

  // confirm that the combined interfaces are different from the originals
  EXPECT_FALSE(&state_iface1   == js_combo);
  EXPECT_FALSE(&state_iface2   == js_combo);
  EXPECT_FALSE(&eff_cmd_iface1 == ej_combo);
  EXPECT_FALSE(&eff_cmd_iface2 == ej_combo);
  EXPECT_FALSE(hw_grp.get<VelocityJointInterface>());

  // confirm that each RobotHW is still working properly independently
  EXPECT_TRUE(&state_iface1   == hw1.get<JointStateInterface>());
  EXPECT_TRUE(&state_iface2   == hw2.get<JointStateInterface>());
  EXPECT_TRUE(&eff_cmd_iface1 == hw1.get<EffortJointInterface>());
  EXPECT_TRUE(&eff_cmd_iface2 == hw2.get<EffortJointInterface>());

  // Retrieve all the handles from the combined interfaces
  JointStateHandle hs1_ret = js_combo->getHandle(name1);
  JointStateHandle hs2_ret = js_combo->getHandle(name2);
  JointHandle hc1_ret = ej_combo->getHandle(name1);
  JointHandle hc2_ret = ej_combo->getHandle(name2);

  // confirm the handles are proper copies
  EXPECT_TRUE(hs1.getPosition() == hs1_ret.getPosition());
  EXPECT_TRUE(hs2.getPosition() == hs2_ret.getPosition());
  hc1.setCommand(3.14);
  EXPECT_EQ(3.14, hc1_ret.getCommand());
  hc2.setCommand(6.28);
  EXPECT_EQ(6.28, hc2_ret.getCommand());

  // check to make sure further calls return the same combined interface objects
  JointStateInterface* js_combo2 = hw_grp.get<JointStateInterface>();
  EffortJointInterface* ej_combo2 = hw_grp.get<EffortJointInterface>();
  EXPECT_TRUE(js_combo == js_combo2);
  EXPECT_TRUE(ej_combo == ej_combo2);
}

TEST_F(RobotHWTest, IncrementalSameInterfaces)
{
  // Populate hardware interfaces
  JointStateInterface state_iface1;
  state_iface1.registerHandle(hs1);

  JointStateInterface state_iface2;
  state_iface2.registerHandle(hs2);

  // Register them to different RobotHW instances
  RobotHW hw1, hw2;
  hw1.registerInterface(&state_iface1);
  hw2.registerInterface(&state_iface2);

  RobotHW hw_grp;
  hw_grp.registerInterfaceManager(&hw1);
  JointStateInterface* js_combo1 = hw_grp.get<JointStateInterface>();
  // only one interface exists, so the combined should be exactly the registered interface object
  EXPECT_TRUE(&state_iface1 == js_combo1);
  // check that it contains hs1 handle
  JointStateHandle hs1_ret1 = js_combo1->getHandle(name1);
  EXPECT_TRUE(hs1.getPosition() == hs1_ret1.getPosition());

  hw_grp.registerInterfaceManager(&hw2);
  JointStateInterface* js_combo2 = hw_grp.get<JointStateInterface>();
  EXPECT_FALSE(&state_iface1 == js_combo2);
  EXPECT_FALSE(&state_iface2 == js_combo2);

  // check to see if both joint handles are here
  JointStateHandle hs1_ret2 = js_combo2->getHandle(name1);
  JointStateHandle hs2_ret = js_combo2->getHandle(name2);
  EXPECT_TRUE(hs1.getPosition() == hs1_ret2.getPosition());
  EXPECT_TRUE(hs2.getPosition() == hs2_ret.getPosition());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
