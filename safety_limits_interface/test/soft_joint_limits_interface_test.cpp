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
#include <safety_limits_interface/soft_joint_limits_interface.h>

using std::string;
using namespace hardware_interface;
using namespace safety_limits_interface;

// Floating-point value comparison threshold
const double EPS = 1e-12;

TEST(SaturateTest, Saturate)
{
  using namespace safety_limits_interface::internal;
  const double min = -1.0;
  const double max =  2.0;
  double val;

  val = -0.5;
  EXPECT_NEAR(val, saturate(val, min, max), EPS);

  val = 0.5;
  EXPECT_NEAR(val, saturate(val, min, max), EPS);

  val = -1.0;
  EXPECT_NEAR(val, saturate(min, min, max), EPS);

  val = -2.0;
  EXPECT_NEAR(min, saturate(val, min, max), EPS);

  val = 2.0;
  EXPECT_NEAR(val, saturate(val, min, max), EPS);

  val = 3.0;
  EXPECT_NEAR(max, saturate(val, min, max), EPS);
}

class JointLimitsTest
{
public:
  JointLimitsTest()
    : pos(0.0), vel(0.0), eff(0.0), cmd(0.0),
      name("joint_name"),
      state_handle(name, &pos, &vel, &eff),
      cmd_handle(state_handle, &cmd)
  {
    urdf_limits.reset(new urdf::JointLimits);
    urdf_limits->effort   =  8.0;
    urdf_limits->velocity =  2.0;
    urdf_limits->lower    = -1.0;
    urdf_limits->upper    =  1.0;

    urdf_safety.reset(new urdf::JointSafety);
    urdf_safety->k_position       = 20.0;
    urdf_safety->k_velocity       = 40.0; // TODO: Tune value
    urdf_safety->soft_lower_limit = -0.8;
    urdf_safety->soft_upper_limit =  0.8;

    urdf_joint.reset(new urdf::Joint);
    urdf_joint->limits = urdf_limits;
    urdf_joint->safety = urdf_safety;

    urdf_joint->type = urdf::Joint::UNKNOWN;
  }

protected:
  double pos, vel, eff, cmd;
  string name;
  JointStateHandle state_handle;
  JointHandle cmd_handle;
  boost::shared_ptr<urdf::JointLimits> urdf_limits;
  boost::shared_ptr<urdf::JointSafety> urdf_safety;
  boost::shared_ptr<urdf::Joint> urdf_joint;
};

class JointLimitsHandleTest : public JointLimitsTest, public ::testing::Test {};

#ifndef NDEBUG // NOTE: This test validates assertion triggering, hence only gets compiled in debug mode
TEST_F(JointLimitsHandleTest, AssertionTriggering)
{
  // Data with invalid pointers should trigger an assertion
  EXPECT_DEATH(PositionJointSoftLimitsHandle().enforceLimits(ros::Duration(1e-3)), ".*");
  EXPECT_DEATH(EffortJointSoftLimitsHandle().enforceLimits(ros::Duration(1e-3)), ".*");
  EXPECT_DEATH(VelocityJointSaturationHandle().enforceLimits(ros::Duration(1e-3)), ".*");
}
#endif // NDEBUG

TEST_F(JointLimitsHandleTest, HandleConstruction)
{
  // TODO: We cannot check if the joint handle is properly constructed or not!. We can only wait until it fails at enforceLimits()
  {
    boost::shared_ptr<urdf::Joint> urdf_joint_bad;
    EXPECT_THROW(PositionJointSoftLimitsHandle(cmd_handle, urdf_joint_bad), JointLimitsInterfaceException);
  }

  {
    boost::shared_ptr<urdf::Joint> urdf_joint_bad(new urdf::Joint);
    EXPECT_THROW(PositionJointSoftLimitsHandle(cmd_handle, urdf_joint_bad), JointLimitsInterfaceException);
  }

  {
    boost::shared_ptr<urdf::Joint> urdf_joint_bad(new urdf::Joint);
    urdf_joint_bad->limits = urdf_limits;
    EXPECT_THROW(PositionJointSoftLimitsHandle(cmd_handle, urdf_joint_bad), JointLimitsInterfaceException);
  }

  {
    boost::shared_ptr<urdf::Joint> urdf_joint_bad(new urdf::Joint);
    urdf_joint_bad->safety = urdf_safety;
    EXPECT_THROW(PositionJointSoftLimitsHandle(cmd_handle, urdf_joint_bad), JointLimitsInterfaceException);
  }

  EXPECT_NO_THROW(PositionJointSoftLimitsHandle(cmd_handle, urdf_joint));
}

class PositionJointSoftLimitsHandleTest : public JointLimitsTest, public ::testing::Test {};

TEST_F(PositionJointSoftLimitsHandleTest, EnforceVelocityBounds)
{
  // Test setup
  PositionJointSoftLimitsHandle limits_handle(cmd_handle, urdf_joint);
  ros::Duration period(0.1);
  const double max_increment = period.toSec() * urdf_joint->limits->velocity;
  pos = 0.0;

  double cmd;

  // Move slower than maximum velocity
  cmd = max_increment / 2.0;
  cmd_handle.setCommand(cmd);
  limits_handle.enforceLimits(period);
  EXPECT_NEAR(cmd, cmd_handle.getCommand(), EPS);

  cmd = -max_increment / 2.0;
  cmd_handle.setCommand(cmd);
  limits_handle.enforceLimits(period);
  EXPECT_NEAR(cmd, cmd_handle.getCommand(), EPS);

  // Move at maximum velocity
  cmd = max_increment;
  cmd_handle.setCommand(cmd);
  limits_handle.enforceLimits(period);
  EXPECT_NEAR(cmd, cmd_handle.getCommand(), EPS);

  cmd = -max_increment;
  cmd_handle.setCommand(cmd);
  limits_handle.enforceLimits(period);
  EXPECT_NEAR(cmd, cmd_handle.getCommand(), EPS);

  // Try to move faster than the maximum velocity, enforce velocity limits
  cmd = 2.0 * max_increment;
  cmd_handle.setCommand(cmd);
  limits_handle.enforceLimits(period);
  EXPECT_NEAR(max_increment, cmd_handle.getCommand(), EPS);

  cmd = -2.0 * max_increment;
  cmd_handle.setCommand(cmd);
  limits_handle.enforceLimits(period);
  EXPECT_NEAR(-max_increment, cmd_handle.getCommand(), EPS);
}

class EnforceSoftLimitsTest : public PositionJointSoftLimitsHandleTest, public ::testing::WithParamInterface<std::string> {};

// This is a black box test and does not verify against random precomuted values, but rather that the expected
// qualitative behavior is honored.
TEST_P(EnforceSoftLimitsTest, EnforcePositionBounds)
{
  // Test setup
  // NOTE: If urdf::Joint::type wasn't an anonymous enum, the following conditional could've been saved
  if      ("REVOLUTE"  == GetParam()) {urdf_joint->type = urdf::Joint::REVOLUTE;}
  else if ("PRISMATIC" == GetParam()) {urdf_joint->type = urdf::Joint::PRISMATIC;}

  PositionJointSoftLimitsHandle limits_handle(cmd_handle, urdf_joint);

  ros::Duration period(0.1);

  // Convenience variables
  const double soft_lower_limit = urdf_safety->soft_lower_limit;
  const double soft_upper_limit = urdf_safety->soft_upper_limit;

  const double hard_lower_limit = urdf_limits->lower;
  const double hard_upper_limit = urdf_limits->upper;

  const double workspace_center = (hard_lower_limit + hard_upper_limit) / 2.0;


  // Current position == upper soft limit
  {
    // Can't get any closer to hard limit (zero max velocity)
    pos = soft_upper_limit;
    cmd_handle.setCommand(hard_upper_limit); // Try to get closer to the hard limit
    limits_handle.enforceLimits(period);
    EXPECT_NEAR(cmd_handle.getPosition(), cmd_handle.getCommand(), EPS);

    // OK to move away from hard limit
    cmd_handle.setCommand(workspace_center); // Try to go to workspace center
    limits_handle.enforceLimits(period);
    EXPECT_GT(cmd_handle.getPosition(), cmd_handle.getCommand());
  }

  // Current position == lower soft limit
  {
    // Can't get any closer to hard limit (zero min velocity)
    pos = soft_lower_limit;
    cmd_handle.setCommand(hard_lower_limit); // Try to get closer to the hard limit
    limits_handle.enforceLimits(period);
    EXPECT_NEAR(cmd_handle.getPosition(), cmd_handle.getCommand(), EPS);

    // OK to move away from hard limit
    cmd_handle.setCommand(workspace_center); // Try to go to workspace center
    limits_handle.enforceLimits(period);
    EXPECT_LT(cmd_handle.getPosition(), cmd_handle.getCommand());
  }

  // Current position > upper soft limit
  {
    // Can't get any closer to hard limit (negative max velocity)
    pos = (soft_upper_limit + hard_upper_limit) / 2.0; // Halfway between soft and hard limit
    cmd_handle.setCommand(hard_upper_limit);           // Try to get closer to the hard limit
    limits_handle.enforceLimits(period);
    EXPECT_GT(cmd_handle.getPosition(), cmd_handle.getCommand());

    // OK to move away from hard limit
    cmd_handle.setCommand(workspace_center);           // Try to go to workspace center
    limits_handle.enforceLimits(period);
    EXPECT_GT(cmd_handle.getPosition(), cmd_handle.getCommand());
  }

  // Current position < lower soft limit
  {
    // Can't get any closer to hard limit (positive min velocity)
    pos = (soft_lower_limit + hard_lower_limit) / 2.0; // Halfway between soft and hard limit
    cmd_handle.setCommand(hard_lower_limit);           // Try to get closer to the hard limit
    limits_handle.enforceLimits(period);
    EXPECT_LT(cmd_handle.getPosition(), cmd_handle.getCommand());

    // OK to move away from hard limit
    cmd_handle.setCommand(workspace_center);           // Try to go to workspace center
    limits_handle.enforceLimits(period);
    EXPECT_LT(cmd_handle.getPosition(), cmd_handle.getCommand());
  }
}

TEST_P(EnforceSoftLimitsTest, PathologicalSoftBounds)
{
  // Test setup
  // NOTE: If urdf::Joint::type wasn't an anonymous enum, the following conditional could've been saved
  if      ("REVOLUTE"  == GetParam()) {urdf_joint->type = urdf::Joint::REVOLUTE;}
  else if ("PRISMATIC" == GetParam()) {urdf_joint->type = urdf::Joint::PRISMATIC;}

  PositionJointSoftLimitsHandle limits_handle(cmd_handle, urdf_joint);

  ros::Duration period(0.1);

  // Safety limits are past the hard limits
  urdf_safety->soft_lower_limit = urdf_limits->lower * (1.0 - 0.5 * urdf_limits->lower / std::abs(urdf_limits->lower));
  urdf_safety->soft_upper_limit = urdf_limits->upper * (1.0 + 0.5 * urdf_limits->upper / std::abs(urdf_limits->upper));

  // Convenience variables
  const double soft_lower_limit = urdf_safety->soft_lower_limit;
  const double soft_upper_limit = urdf_safety->soft_upper_limit;

  const double hard_lower_limit = urdf_limits->lower;
  const double hard_upper_limit = urdf_limits->upper;

  // Current position == higher hard limit
  {
    // Hit hard limit
    pos = hard_upper_limit;                        // On hard limit
    cmd_handle.setCommand(2.0 * hard_upper_limit); // Way beyond hard limit
    limits_handle.enforceLimits(period);
    EXPECT_NEAR(hard_upper_limit, cmd_handle.getCommand(), EPS);
  }

  // Current position == lower hard limit
  {
    // Hit hard limit
    pos = hard_lower_limit;                        // On hard limit
    cmd_handle.setCommand(2.0 * hard_lower_limit); // Way beyond hard limit
    limits_handle.enforceLimits(period);
    EXPECT_NEAR(hard_lower_limit, cmd_handle.getCommand(), EPS);
  }
}

INSTANTIATE_TEST_CASE_P(WithSoftPositionBounds,
                        EnforceSoftLimitsTest,
                        ::testing::Values("REVOLUTE", "PRISMATIC"));


class VelocityJointSaturationHandleTest : public JointLimitsTest, public ::testing::Test {};

TEST_F(VelocityJointSaturationHandleTest, EnforceVelocityBounds)
{
  // Test setup
  VelocityJointSaturationHandle limits_handle(cmd_handle, urdf_joint);
  ros::Duration period(0.1);
  const double vel_max = urdf_joint->limits->velocity;
  pos = 0.0;

  double cmd;

  // Velocity within bounds
  cmd = vel_max / 2.0;
  cmd_handle.setCommand(cmd);
  limits_handle.enforceLimits(period);
  EXPECT_NEAR(cmd, cmd_handle.getCommand(), EPS);

  cmd = -vel_max / 2.0;
  cmd_handle.setCommand(cmd);
  limits_handle.enforceLimits(period);
  EXPECT_NEAR(cmd, cmd_handle.getCommand(), EPS);

  // Velocity at bounds
  cmd = vel_max;
  cmd_handle.setCommand(cmd);
  limits_handle.enforceLimits(period);
  EXPECT_NEAR(cmd, cmd_handle.getCommand(), EPS);

  cmd = -vel_max;
  cmd_handle.setCommand(cmd);
  limits_handle.enforceLimits(period);
  EXPECT_NEAR(cmd, cmd_handle.getCommand(), EPS);

  // Velocity beyond bounds
  cmd = 2.0 * vel_max;
  cmd_handle.setCommand(cmd);
  limits_handle.enforceLimits(period);
  EXPECT_NEAR(vel_max, cmd_handle.getCommand(), EPS);

  cmd = -2.0 * vel_max;
  cmd_handle.setCommand(cmd);
  limits_handle.enforceLimits(period);
  EXPECT_NEAR(-vel_max, cmd_handle.getCommand(), EPS);
}


class SoftJointLimitsInterfaceTest :public JointLimitsTest, public ::testing::Test
{
public:
  SoftJointLimitsInterfaceTest()
    : JointLimitsTest(),
      pos2(0.0), vel2(0.0), eff2(0.0), cmd2(0.0),
      name2("joint2_name"),
      state_handle2(name2, &pos2, &vel2, &eff2),
      cmd_handle2(state_handle2, &cmd2)
  {}

protected:
  double pos2, vel2, eff2, cmd2;
  string name2;
  JointStateHandle state_handle2;
  JointHandle cmd_handle2;
};

TEST_F(SoftJointLimitsInterfaceTest, InterfaceRegistration)
{
  urdf_joint->type = urdf::Joint::REVOLUTE;
  ros::Duration period(0.1);

  // Populate interface
  PositionJointSoftLimitsHandle limits_handle1(cmd_handle,  urdf_joint);
  PositionJointSoftLimitsHandle limits_handle2(cmd_handle2, urdf_joint);

  PositionJointSoftLimitsInterface iface;
  iface.registerHandle(limits_handle1);
  iface.registerHandle(limits_handle2);

  // Get handles
  EXPECT_NO_THROW(iface.getHandle(name));
  EXPECT_NO_THROW(iface.getHandle(name2));

  PositionJointSoftLimitsHandle h1_tmp = iface.getHandle(name);
  EXPECT_EQ(name, h1_tmp.getName());

  PositionJointSoftLimitsHandle h2_tmp = iface.getHandle(name2);
  EXPECT_EQ(name2, h2_tmp.getName());

  // Print error message
  // Requires manual output inspection, but exception message should contain the interface name (not its base clase)
  try {iface.getHandle("unknown_name");}
  catch(const JointLimitsInterfaceException& e) {ROS_ERROR_STREAM(e.what());}

  // Enforce limits of all managed joints
  const double soft_upper_limit = urdf_safety->soft_upper_limit;
  const double hard_upper_limit = urdf_limits->upper;

  pos = pos2 = (soft_upper_limit + hard_upper_limit) / 2.0; // Halfway between soft and hard limit
  cmd_handle.setCommand(hard_upper_limit);                  // Try to get closer to the hard limit
  cmd_handle2.setCommand(hard_upper_limit);
  iface.enforceLimits(period);
  EXPECT_GT(cmd_handle.getPosition(),  cmd_handle.getCommand());
  EXPECT_GT(cmd_handle2.getPosition(), cmd_handle2.getCommand());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

