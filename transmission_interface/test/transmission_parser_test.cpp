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

#include <gtest/gtest.h>

#include <transmission_interface/transmission_parser.h>
#include "read_file.h"

using namespace transmission_interface;

TEST(TransmissionParserTest, InvalidFile)
{
  TransmissionParser parser;
  std::string urdf = "invalid urdf";
  std::vector<TransmissionInfo> infos;

  ASSERT_FALSE(parser.parse(urdf, infos));
  EXPECT_TRUE(infos.empty());
}

TEST(TransmissionParserTest, EmptyFile)
{
  TransmissionParser parser;
  std::string urdf = "<?xml version=\"1.0\"?><robot name=\"robot\" xmlns=\"http://www.ros.org\"></robot>";
  std::vector<TransmissionInfo> infos;

  ASSERT_TRUE(parser.parse(urdf, infos));
  EXPECT_TRUE(infos.empty());
}

TEST(TransmissionParserTest, InvalidTransmissions)
{
  TransmissionParser parser;
  std::string urdf;
  std::vector<TransmissionInfo> infos;

  ASSERT_TRUE(readFile("test/urdf/parser_test_invalid.urdf", urdf));
  ASSERT_TRUE(parser.parse(urdf, infos));
  EXPECT_TRUE(infos.empty());
}


TEST(TransmissionParserTest, SuccessfulParse)
{
  std::vector<TransmissionInfo> infos;
  {
    // Parser and URDF get destroyed at the end of this scope.
    std::string urdf;
    ASSERT_TRUE(readFile("test/urdf/parser_test_valid.urdf", urdf));

    TransmissionParser parser;
    ASSERT_TRUE(parser.parse(urdf, infos));
  }
  EXPECT_EQ(2, infos.size());

  // Differential transmission
  {
    const TransmissionInfo& info = infos.front();
    EXPECT_EQ("differential_trans", info.name_);
    EXPECT_EQ("transmission_interface/DifferentialTransmission", info.type_);

    ASSERT_EQ(2, info.actuators_.size());
    EXPECT_EQ("foo_actuator",info.actuators_.front().name_);
    EXPECT_EQ("bar_actuator",info.actuators_.back().name_);
    EXPECT_FALSE(info.actuators_.front().xml_element_.empty());
    EXPECT_FALSE(info.actuators_.back().xml_element_.empty());

    ASSERT_EQ(2, info.joints_.size());
    EXPECT_EQ("foo_joint",info.joints_.front().name_);
    EXPECT_EQ("bar_joint",info.joints_.back().name_);

    ASSERT_EQ(2, info.joints_[0].hardware_interfaces_.size());
    EXPECT_EQ("hardware_interface/PositionJointInterface", info.joints_[0].hardware_interfaces_[0]);
    EXPECT_EQ("hardware_interface/VelocityJointInterface", info.joints_[0].hardware_interfaces_[1]);

    EXPECT_STREQ(info.joints_[0].role_.c_str(),"role1");
    EXPECT_STREQ(info.joints_[1].role_.c_str(),"role2");

    ASSERT_EQ(1, info.joints_[1].hardware_interfaces_.size());
    EXPECT_EQ("hardware_interface/EffortJointInterface", info.joints_[1].hardware_interfaces_[0]);

    EXPECT_FALSE(info.joints_.front().xml_element_.empty());
    EXPECT_FALSE(info.joints_.back().xml_element_.empty());
  }

  // Simple transmission
  {
    const TransmissionInfo& info = infos.back();
    EXPECT_EQ("simple_trans", info.name_);
    EXPECT_EQ("transmission_interface/SimpleTransmission", info.type_);

    ASSERT_EQ(1, info.actuators_.size());
    EXPECT_EQ("baz_actuator",info.actuators_.front().name_);
    EXPECT_FALSE(info.actuators_.front().xml_element_.empty());

    ASSERT_EQ(1, info.actuators_.front().hardware_interfaces_.size());
    EXPECT_EQ("hardware_interface/PositionActuatorInterface", info.actuators_.front().hardware_interfaces_.front());

    ASSERT_EQ(1, info.joints_.size());
    EXPECT_EQ("baz_joint",info.joints_.front().name_);
    EXPECT_FALSE(info.joints_.front().xml_element_.empty());

    ASSERT_EQ(1, info.joints_.front().hardware_interfaces_.size());
    EXPECT_EQ("hardware_interface/PositionJointInterface", info.joints_.front().hardware_interfaces_.front());
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
