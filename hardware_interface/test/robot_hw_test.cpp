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
public:
  RobotHWTest()
    : pos1(1.0), vel1(2.0), eff1(3.0), cmd1(0.0),
      pos2(4.0), vel2(5.0), eff2(6.0), cmd2(0.0),
      name1("name_1"),
      name2("name_2"),
      hs1(name1, &pos1, &vel1, &eff1),
      hs2(name2, &pos2, &vel2, &eff2),
      hc1(hs1, &cmd1),
      hc2(hs2, &cmd2)
  {}

protected:
  double pos1, vel1, eff1, cmd1;
  double pos2, vel2, eff2, cmd2;
  string name1;
  string name2;
  JointStateHandle hs1, hs2;
  JointHandle hc1, hc2;
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
  ControllerInfo info1;
  info1.name = "controller_1";
  info1.type = "type_1";
  info1.hardware_interface = "interface_1";
  info1.resources.insert("resource_1");

  ControllerInfo info2;
  info2.name = "controller_2";
  info2.type = "type_2";
  info2.hardware_interface = "interface_2";
  info2.resources.insert("resource_2");

  ControllerInfo info12;
  info12.name = "controller_12";
  info12.type = "type_12";
  info12.hardware_interface = "interface_12";
  info12.resources.insert("resource_1");
  info12.resources.insert("resource_2");

  // No conflict
  {
    list<ControllerInfo> info_list;
    info_list.push_back(info1);
    info_list.push_back(info2);

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
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

