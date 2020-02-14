
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

#include <iostream>
#include <string>
#include <gtest/gtest.h>
#include <ros/console.h>
#include <hardware_interface/force_torque_sensor_interface.h>

using std::string;
using namespace hardware_interface;

TEST(ForceTorqueSensorHandleTest, HandleConstruction)
{
  // Empty handle
  {
    ForceTorqueSensorHandle handle;
    EXPECT_EQ("", handle.getName());
    EXPECT_EQ("", handle.getFrameId());
    EXPECT_TRUE(nullptr == handle.getForce());
    EXPECT_TRUE(nullptr == handle.getTorque());
  }

  // Valid handle
  {
    double force[3]  = {1.0, 2.0, 3.0};
    double torque[3] = {-1.0, -2.0, -3.0};
    ForceTorqueSensorHandle tmp("name_1", "frame_1", force, torque);
  }
}

class ForceTorqueSensorInterfaceTest : public ::testing::Test
{
protected:
  double force1[3] = {1.0, 2.0, 3.0};
  double force2[3] = {4.0, 5.0, 6.0};
  double torque1[3] = {-force1[0], -force1[1], -force1[2]};
  double torque2[3] = {-force2[0], -force2[1], -force2[2]};
  string name1 = {"name_1"}, name2 = {"name_2"};
  string frame_id1 = {"frame_1"}, frame_id2 = {"frame_2"};
  ForceTorqueSensorHandle h1 = {name1, frame_id1, force1, torque1};
  ForceTorqueSensorHandle h2 = {name2, frame_id2, nullptr, torque2}; // Torque-only sensor
};

TEST_F(ForceTorqueSensorInterfaceTest, ExcerciseApi)
{
  ForceTorqueSensorInterface iface;
  iface.registerHandle(h1);
  iface.registerHandle(h2);

  // Get handles
  EXPECT_NO_THROW(iface.getHandle(name1));
  EXPECT_NO_THROW(iface.getHandle(name2));

  ForceTorqueSensorHandle h1_tmp = iface.getHandle(name1);
  EXPECT_EQ(name1, h1_tmp.getName());
  EXPECT_EQ(frame_id1, h1_tmp.getFrameId());
  EXPECT_TRUE(nullptr != h1_tmp.getForce());
  EXPECT_TRUE(nullptr != h1_tmp.getTorque());
  EXPECT_DOUBLE_EQ(force1[0], h1_tmp.getForce()[0]);
  EXPECT_DOUBLE_EQ(force1[1], h1_tmp.getForce()[1]);
  EXPECT_DOUBLE_EQ(force1[2], h1_tmp.getForce()[2]);
  EXPECT_DOUBLE_EQ(torque1[0], h1_tmp.getTorque()[0]);
  EXPECT_DOUBLE_EQ(torque1[1], h1_tmp.getTorque()[1]);
  EXPECT_DOUBLE_EQ(torque1[2], h1_tmp.getTorque()[2]);

  ForceTorqueSensorHandle h2_tmp = iface.getHandle(name2);
  EXPECT_EQ(name2, h2_tmp.getName());
  EXPECT_EQ(frame_id2, h2_tmp.getFrameId());
  EXPECT_TRUE(nullptr == h2_tmp.getForce());
  EXPECT_TRUE(nullptr != h2_tmp.getTorque());
  EXPECT_DOUBLE_EQ(torque2[0], h2_tmp.getTorque()[0]);
  EXPECT_DOUBLE_EQ(torque2[1], h2_tmp.getTorque()[1]);
  EXPECT_DOUBLE_EQ(torque2[2], h2_tmp.getTorque()[2]);

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
