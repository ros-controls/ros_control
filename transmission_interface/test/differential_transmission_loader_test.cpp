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

#include <string>
#include <boost/foreach.hpp>
#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <transmission_interface/differential_transmission.h>
#include <transmission_interface/transmission_loader.h>
#include "read_file.h"
#include "loader_utils.h"



TEST(DifferentialTransmissionLoaderTest, FullSpec)
{
  // Parse transmission info
  std::vector<TransmissionInfo> infos = parseUrdf("test/urdf/differential_transmission_loader_full.urdf");
  ASSERT_EQ(1, infos.size());

  // Transmission loader
  TransmissionPluginLoader loader;
  TransmissionLoaderSharedPtr transmission_loader = loader.create(infos.front().type_);
  ASSERT_TRUE(0 != transmission_loader);

  TransmissionSharedPtr transmission;
  const TransmissionInfo& info = infos.front();
  transmission = transmission_loader->load(info);
  ASSERT_TRUE(0 != transmission);

  // Validate transmission
  DifferentialTransmission* differential_transmission = dynamic_cast<DifferentialTransmission*>(transmission.get());
  ASSERT_TRUE(0 != differential_transmission);

  const std::vector<double>& actuator_reduction = differential_transmission->getActuatorReduction();
  EXPECT_EQ( 50.0, actuator_reduction[0]);
  EXPECT_EQ(-50.0, actuator_reduction[1]);

  const std::vector<double>& joint_reduction = differential_transmission->getJointReduction();
  EXPECT_EQ( 2.0, joint_reduction[0]);
  EXPECT_EQ(-2.0, joint_reduction[1]);

  const std::vector<double>& joint_offset = differential_transmission->getJointOffset();
  EXPECT_EQ( 0.5, joint_offset[0]);
  EXPECT_EQ(-0.5, joint_offset[1]);
}

TEST(DifferentialTransmissionLoaderTest, MinimalSpec)
{
  // Parse transmission info
  std::vector<TransmissionInfo> infos = parseUrdf("test/urdf/differential_transmission_loader_minimal.urdf");
  ASSERT_EQ(1, infos.size());

  // Transmission loader
  TransmissionPluginLoader loader;
  TransmissionLoaderSharedPtr transmission_loader = loader.create(infos.front().type_);
  ASSERT_TRUE(0 != transmission_loader);

  TransmissionSharedPtr transmission;
  const TransmissionInfo& info = infos.front();
  transmission = transmission_loader->load(info);
  ASSERT_TRUE(0 != transmission);

  // Validate transmission
  DifferentialTransmission* differential_transmission = dynamic_cast<DifferentialTransmission*>(transmission.get());

  const std::vector<double>& actuator_reduction = differential_transmission->getActuatorReduction();
  EXPECT_EQ( 50.0, actuator_reduction[0]);
  EXPECT_EQ(-50.0, actuator_reduction[1]);

  const std::vector<double>& joint_reduction = differential_transmission->getJointReduction();
  EXPECT_EQ(1.0, joint_reduction[0]);
  EXPECT_EQ(1.0, joint_reduction[1]);

  const std::vector<double>& joint_offset = differential_transmission->getJointOffset();
  EXPECT_EQ(0.0, joint_offset[0]);
  EXPECT_EQ(0.0, joint_offset[1]);
}

TEST(DifferentialTransmissionLoaderTest, InvalidSpec)
{
  // Parse transmission info
  std::vector<TransmissionInfo> infos = parseUrdf("test/urdf/differential_transmission_loader_invalid.urdf");
  ASSERT_EQ(13, infos.size());

  // Transmission loader
  TransmissionPluginLoader loader;
  TransmissionLoaderSharedPtr transmission_loader = loader.create(infos.front().type_);
  ASSERT_TRUE(0 != transmission_loader);

  BOOST_FOREACH(const TransmissionInfo& info, infos)
  {
    TransmissionSharedPtr transmission;
    transmission = transmission_loader->load(info);
    ASSERT_TRUE(0 == transmission);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
