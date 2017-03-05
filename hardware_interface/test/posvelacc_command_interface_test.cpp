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

#include <string>
#include <gtest/gtest.h>
#include <ros/console.h>
#include <hardware_interface/posvelacc_command_interface.h>

using std::string;
using namespace hardware_interface;

TEST(PosVelAccCommandHandleTest, HandleConstruction)
{
  string name = "name1";
  double pos, vel, eff;
  double cmd_pos, cmd_vel, cmd_acc;
  EXPECT_NO_THROW(PosVelAccJointHandle tmp(JointStateHandle(name, &pos, &vel, &eff), &cmd_pos, &cmd_vel, &cmd_acc));
  EXPECT_THROW(PosVelAccJointHandle tmp(JointStateHandle(name, &pos, &vel, &eff), 0, &cmd_vel, &cmd_acc), HardwareInterfaceException);
  EXPECT_THROW(PosVelAccJointHandle tmp(JointStateHandle(name, &pos, &vel, &eff), &cmd_pos, 0, &cmd_acc), HardwareInterfaceException);
  EXPECT_THROW(PosVelAccJointHandle tmp(JointStateHandle(name, &pos, &vel, &eff), &cmd_pos, &cmd_vel, 0), HardwareInterfaceException);

  // Print error messages
  // Requires manual output inspection, but exception message should be descriptive
  try {PosVelAccJointHandle tmp(JointStateHandle(name, &pos, &vel, &eff), 0, 0, 0);}
  catch(const HardwareInterfaceException& e) {ROS_ERROR_STREAM(e.what());}
}

#ifndef NDEBUG // NOTE: This test validates assertion triggering, hence only gets compiled in debug mode
TEST(JointStateHandleTest, AssertionTriggering)
{
  PosVelAccJointHandle h;

  // Data with invalid pointers should trigger an assertion
  EXPECT_DEATH(h.getPosition(),   ".*");
  EXPECT_DEATH(h.getVelocity(),   ".*");
  EXPECT_DEATH(h.getEffort(),     ".*");
  EXPECT_DEATH(h.getCommandPosition(),     ".*");
  EXPECT_DEATH(h.getCommandVelocity(),     ".*");
  EXPECT_DEATH(h.getCommandAcceleration(), ".*");
  EXPECT_DEATH(h.setCommandPosition(2.0),     ".*");
  EXPECT_DEATH(h.setCommandVelocity(3.0),     ".*");
  EXPECT_DEATH(h.setCommandAcceleration(4.0), ".*");
  EXPECT_DEATH(h.setCommand(1.0, 2.0, 3.0), ".*");
}
#endif // NDEBUG

class PosVelAccCommandInterfaceTest : public ::testing::Test
{
public:
  PosVelAccCommandInterfaceTest()
    : pos1(1.0), vel1(2.0), eff1(3.0),
      cmd_pos1(0.0), cmd_vel1(0.0), cmd_acc1(0.0),
      pos2(4.0), vel2(5.0), eff2(6.0),
      cmd_pos2(0.0), cmd_vel2(0.0), cmd_acc2(0.0),
      name1("name_1"),
      name2("name_2"),
      hs1(name1, &pos1, &vel1, &eff1),
      hs2(name2, &pos2, &vel2, &eff2),
      hc1(hs1, &cmd_pos1, &cmd_vel1, &cmd_acc1),
      hc2(hs2, &cmd_pos2, &cmd_vel2, &cmd_acc2)
  {}

protected:
  double pos1, vel1, eff1, cmd_pos1, cmd_vel1, cmd_acc1;
  double pos2, vel2, eff2, cmd_pos2, cmd_vel2, cmd_acc2;
  string name1;
  string name2;
  JointStateHandle hs1, hs2;
  PosVelAccJointHandle hc1, hc2;
};

TEST_F(PosVelAccCommandInterfaceTest, ExcerciseApi)
{
  PosVelAccJointInterface iface;
  iface.registerHandle(hc1);
  iface.registerHandle(hc2);

  // Get handles
  EXPECT_NO_THROW(iface.getHandle(name1));
  EXPECT_NO_THROW(iface.getHandle(name2));

  PosVelAccJointHandle hc1_tmp = iface.getHandle(name1);
  EXPECT_EQ(name1, hc1_tmp.getName());
  EXPECT_DOUBLE_EQ(pos1, hc1_tmp.getPosition());
  EXPECT_DOUBLE_EQ(vel1, hc1_tmp.getVelocity());
  EXPECT_DOUBLE_EQ(eff1, hc1_tmp.getEffort());
  EXPECT_DOUBLE_EQ(cmd_pos1, hc1_tmp.getCommandPosition());
  EXPECT_DOUBLE_EQ(cmd_vel1, hc1_tmp.getCommandVelocity());
  EXPECT_DOUBLE_EQ(cmd_acc1, hc1_tmp.getCommandAcceleration());
  const double new_cmd_pos1 = -1.0, new_cmd_vel1 = -2.0, new_cmd_acc1 = -3.0;
  hc1_tmp.setCommand(new_cmd_pos1, new_cmd_vel1, new_cmd_acc1);
  EXPECT_DOUBLE_EQ(new_cmd_pos1, hc1_tmp.getCommandPosition());
  EXPECT_DOUBLE_EQ(new_cmd_vel1, hc1_tmp.getCommandVelocity());
  EXPECT_DOUBLE_EQ(new_cmd_acc1, hc1_tmp.getCommandAcceleration());

  PosVelAccJointHandle hc2_tmp = iface.getHandle(name2);
  EXPECT_EQ(name2, hc2_tmp.getName());
  EXPECT_DOUBLE_EQ(pos2, hc2_tmp.getPosition());
  EXPECT_DOUBLE_EQ(vel2, hc2_tmp.getVelocity());
  EXPECT_DOUBLE_EQ(eff2, hc2_tmp.getEffort());
  EXPECT_DOUBLE_EQ(cmd_pos2, hc2_tmp.getCommandPosition());
  EXPECT_DOUBLE_EQ(cmd_vel2, hc2_tmp.getCommandVelocity());
  EXPECT_DOUBLE_EQ(cmd_acc2, hc2_tmp.getCommandAcceleration());
  const double new_cmd_pos2 = -1.0, new_cmd_vel2 = -2.0, new_cmd_acc2 = -3.0;
  hc2_tmp.setCommand(new_cmd_pos2, new_cmd_vel2, new_cmd_acc2);
  EXPECT_DOUBLE_EQ(new_cmd_pos2, hc2_tmp.getCommandPosition());
  EXPECT_DOUBLE_EQ(new_cmd_vel2, hc2_tmp.getCommandVelocity());
  EXPECT_DOUBLE_EQ(new_cmd_acc2, hc2_tmp.getCommandAcceleration());

  // This interface claims resources
  EXPECT_EQ(2, iface.getClaims().size());

  // Print error message
  // Requires manual output inspection, but exception message should contain the interface name (not its base clase)
  try {iface.getHandle("unknown_name");}
  catch(const HardwareInterfaceException& e) {ROS_ERROR_STREAM(e.what());}
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

