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
#include <hardware_interface/joint_state_interface.h>

using std::string;
using namespace hardware_interface;

TEST(JointStateHandleTest, HandleConstruction)
{
  string name = "name1";
  double pos, vel, eff;
  EXPECT_NO_THROW(JointStateHandle tmp(name, &pos, &vel, &eff));
  EXPECT_THROW(JointStateHandle(name, 0, &vel, &eff), HardwareInterfaceException);
  EXPECT_THROW(JointStateHandle(name, &pos, 0, &eff), HardwareInterfaceException);
  EXPECT_THROW(JointStateHandle(name, &pos, &vel, 0), HardwareInterfaceException);

  // Print error messages
  // Requires manual output inspection, but exception message should be descriptive
  try {JointStateHandle(name, 0, &vel, &eff);}
  catch(const HardwareInterfaceException& e) {ROS_ERROR_STREAM(e.what());}

  try {JointStateHandle(name, &pos, 0, &eff);}
  catch(const HardwareInterfaceException& e) {ROS_ERROR_STREAM(e.what());}

  try {JointStateHandle(name, &pos, &vel, 0);}
  catch(const HardwareInterfaceException& e) {ROS_ERROR_STREAM(e.what());}

}

#ifndef NDEBUG // NOTE: This test validates assertion triggering, hence only gets compiled in debug mode
TEST(JointStateHandleTest, AssertionTriggering)
{
  JointStateHandle h;

  // Data with invalid pointers should trigger an assertion
  EXPECT_DEATH(h.getPosition(), ".*");
  EXPECT_DEATH(h.getVelocity(), ".*");
  EXPECT_DEATH(h.getEffort(),   ".*");
}
#endif // NDEBUG

class JointStateInterfaceTest : public ::testing::Test
{
public:
  JointStateInterfaceTest()
    : pos1(1.0), vel1(2.0), eff1(3.0),
      pos2(4.0), vel2(5.0), eff2(6.0),
      name1("name_1"),
      name2("name_2"),
      h1(name1, &pos1, &vel1, &eff1),
      h2(name2, &pos2, &vel2, &eff2)
  {}

protected:
  double pos1, vel1, eff1;
  double pos2, vel2, eff2;
  string name1;
  string name2;
  JointStateHandle h1, h2;
};

TEST_F(JointStateInterfaceTest, ExcerciseApi)
{
  JointStateInterface iface;
  iface.registerHandle(h1);
  iface.registerHandle(h2);

  // Get handles
  EXPECT_NO_THROW(iface.getHandle(name1));
  EXPECT_NO_THROW(iface.getHandle(name2));

  JointStateHandle h1_tmp = iface.getHandle(name1);
  EXPECT_EQ(name1, h1_tmp.getName());
  EXPECT_DOUBLE_EQ(pos1, h1_tmp.getPosition());
  EXPECT_DOUBLE_EQ(vel1, h1_tmp.getVelocity());
  EXPECT_DOUBLE_EQ(eff1, h1_tmp.getEffort());

  JointStateHandle h2_tmp = iface.getHandle(name2);
  EXPECT_EQ(name2, h2_tmp.getName());
  EXPECT_DOUBLE_EQ(pos2, h2_tmp.getPosition());
  EXPECT_DOUBLE_EQ(vel2, h2_tmp.getVelocity());
  EXPECT_DOUBLE_EQ(eff2, h2_tmp.getEffort());

  // This interface does not claim resources
  EXPECT_TRUE(iface.getClaims().empty());

  // Print error message
  // Requires manual output inspection, but exception message should contain the interface name (not its base class)
  try {iface.getHandle("unknown_name");}
  catch(const HardwareInterfaceException& e) {ROS_ERROR_STREAM(e.what());}
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
