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

#include <cstdlib>
#include <ctime>
#include <string>
#include <gtest/gtest.h>
#include <ros/console.h>
#include <hardware_interface/imu_sensor_interface.h>

using std::string;
using namespace hardware_interface;

TEST(ImuSensorHandleTest, HandleConstruction)
{
  // Empty handle
  {
    ImuSensorHandle handle;
    EXPECT_EQ("", handle.getName());
    EXPECT_EQ("", handle.getFrameId());
    EXPECT_TRUE(0 == handle.getOrientation());
    EXPECT_TRUE(0 == handle.getOrientationCovariance());
    EXPECT_TRUE(0 == handle.getAngularVelocity());
    EXPECT_TRUE(0 == handle.getAngularVelocityCovariance());
    EXPECT_TRUE(0 == handle.getLinearAcceleration());
    EXPECT_TRUE(0 == handle.getLinearAccelerationCovariance());
  }

  // Valid handle
  {
    double orientation[4]         = {0.0, 0.0, 0.0, 1.0};
    double covariance[9]          = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double angular_velocity[3]    = {1.0, 2.0, 3.0};
    double linear_acceleration[3] = {-1.0, -2.0, -3.0};

    ImuSensorHandle::Data data;
    data.name = "name_1";
    data.frame_id = "frame_1";
    data.orientation = orientation;
    data.orientation_covariance = covariance;
    data.angular_velocity = angular_velocity;
    data.angular_velocity_covariance = covariance;
    data.linear_acceleration = linear_acceleration;
    data.linear_acceleration_covariance = covariance;

    ImuSensorHandle handle(data);
  }
}

class ImuSensorInterfaceTest : public ::testing::Test
{
public:
  ImuSensorInterfaceTest()
    : name1("name_1"), name2("name_2"),
      frame_id1("frame_1"), frame_id2("frame_2")
  {
    srand(time(NULL)); // Seed random number generator

    // Populate raw data
    for (unsigned int i = 0; i < 4; ++i)
    {
      orientation1[i] = randomDouble();
      orientation2[i] = randomDouble();
    }
    for (unsigned int i = 0; i < 3; ++i)
    {
      angular_velocity1[i]    = randomDouble();
      linear_acceleration1[i] = randomDouble();
    }
    for (unsigned int i = 0; i < 9; ++i)
    {
      orientation_covariance1[i]         = randomDouble();
      angular_velocity_covariance1[i]    = randomDouble();
      linear_acceleration_covariance1[i] = randomDouble();
    }

    // First handle exposes all fields
    ImuSensorHandle::Data data1;
    data1.name                           = name1;
    data1.frame_id                       = frame_id1;
    data1.orientation                    = orientation1;
    data1.orientation_covariance         = orientation_covariance1;
    data1.angular_velocity               = angular_velocity1;
    data1.angular_velocity_covariance    = angular_velocity_covariance1;
    data1.linear_acceleration            = linear_acceleration1;
    data1.linear_acceleration_covariance = linear_acceleration_covariance1;
    h1 = ImuSensorHandle(data1);

    // Second handle only exposes orientation data
    ImuSensorHandle::Data data2;
    data2.name        = name2;
    data2.frame_id    = frame_id2;
    data2.orientation = orientation2;
    h2 = ImuSensorHandle(data2);
  }

protected:
  double orientation1[4], orientation2[4];
  double orientation_covariance1[9];
  double angular_velocity1[3];
  double angular_velocity_covariance1[9];
  double linear_acceleration1[3];
  double linear_acceleration_covariance1[9];
  string name1, name2;
  string frame_id1, frame_id2;
  ImuSensorHandle h1;
  ImuSensorHandle h2;

  double randomDouble(double min_val = 0.0, double max_val = 1.0)
  {
    const double range = max_val - min_val;
    return rand() / static_cast<double>(RAND_MAX) * range + min_val;
  }
};

TEST_F(ImuSensorInterfaceTest, ExcerciseApi)
{
  ImuSensorInterface iface;
  iface.registerHandle(h1);
  iface.registerHandle(h2);

  // Get handles
  EXPECT_NO_THROW(iface.getHandle(name1));
  EXPECT_NO_THROW(iface.getHandle(name2));

  ImuSensorHandle h1_tmp = iface.getHandle(name1);
  EXPECT_EQ(name1, h1_tmp.getName());
  EXPECT_EQ(frame_id1, h1_tmp.getFrameId());

  EXPECT_TRUE(0 != h1_tmp.getOrientation());
  EXPECT_TRUE(0 != h1_tmp.getOrientationCovariance());
  EXPECT_TRUE(0 != h1_tmp.getAngularVelocity());
  EXPECT_TRUE(0 != h1_tmp.getAngularVelocityCovariance());
  EXPECT_TRUE(0 != h1_tmp.getLinearAcceleration());
  EXPECT_TRUE(0 != h1_tmp.getLinearAccelerationCovariance());

  for (unsigned int i = 0; i < 4; ++i)
  {
    EXPECT_DOUBLE_EQ(orientation1[i], h1_tmp.getOrientation()[i]);
  }

  for (unsigned int i = 0; i < 3; ++i)
  {
    EXPECT_DOUBLE_EQ(angular_velocity1[i],    h1_tmp.getAngularVelocity()[i]);
    EXPECT_DOUBLE_EQ(linear_acceleration1[i], h1_tmp.getLinearAcceleration()[i]);
  }

  for (unsigned int i = 0; i < 9; ++i)
  {
    EXPECT_DOUBLE_EQ(orientation_covariance1[i],         h1_tmp.getOrientationCovariance()[i]);
    EXPECT_DOUBLE_EQ(angular_velocity_covariance1[i],    h1_tmp.getAngularVelocityCovariance()[i]);
    EXPECT_DOUBLE_EQ(linear_acceleration_covariance1[i], h1_tmp.getLinearAccelerationCovariance()[i]);
  }

  ImuSensorHandle h2_tmp = iface.getHandle(name2);
  EXPECT_EQ(name2, h2_tmp.getName());
  EXPECT_EQ(frame_id2, h2_tmp.getFrameId());

  EXPECT_TRUE(0 != h2_tmp.getOrientation());
  EXPECT_TRUE(0 == h2_tmp.getOrientationCovariance());
  EXPECT_TRUE(0 == h2_tmp.getAngularVelocity());
  EXPECT_TRUE(0 == h2_tmp.getAngularVelocityCovariance());
  EXPECT_TRUE(0 == h2_tmp.getLinearAcceleration());
  EXPECT_TRUE(0 == h2_tmp.getLinearAccelerationCovariance());

  for (unsigned int i = 0; i < 4; ++i)
  {
    EXPECT_DOUBLE_EQ(orientation2[i], h2_tmp.getOrientation()[i]);
  }

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
