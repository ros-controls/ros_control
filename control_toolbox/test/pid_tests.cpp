
#include <ros/ros.h>

#include <gtest/gtest.h>

#include <control_toolbox/pid.h>

#include <boost/math/special_functions/fpclassify.hpp>

using namespace control_toolbox;

TEST(ParameterTest, zeroITermBadIBoundsTest)
{
  RecordProperty("description","This test checks robustness against divide-by-zero errors when given integral term bounds which do not include 0.0.");

  Pid pid(1.0, 0.0, 0.0, -1.0, 0.0);

  double cmd = 0.0;
  double pe,ie,de;

  cmd = pid.updatePid(-1.0, ros::Duration(1.0));
  pid.getCurrentPIDErrors(&pe,&ie,&de);
  EXPECT_FALSE(boost::math::isinf(ie));
  EXPECT_FALSE(boost::math::isnan(cmd));

  cmd = pid.updatePid(-1.0, ros::Duration(1.0));
  pid.getCurrentPIDErrors(&pe,&ie,&de);
  EXPECT_FALSE(boost::math::isinf(ie));
  EXPECT_FALSE(boost::math::isnan(cmd));
}

TEST(ParameterTest, integrationWindupTest)
{
  RecordProperty("description","This test succeeds if the integral error is prevented from winding up when the integral gain is non-zero.");

  Pid pid(0.0, 1.0, 0.0, 1.0, -1.0);

  double cmd = 0.0;
  double pe,ie,de;

  cmd = pid.updatePid(-1.0, ros::Duration(1.0));
  pid.getCurrentPIDErrors(&pe,&ie,&de);
  EXPECT_EQ(-1.0, ie);
  EXPECT_EQ(1.0, cmd);

  cmd = pid.updatePid(-1.0, ros::Duration(1.0));
  pid.getCurrentPIDErrors(&pe,&ie,&de);
  EXPECT_EQ(-1.0, ie);
  EXPECT_EQ(1.0, cmd);
}

TEST(ParameterTest, integrationWindupZeroGainTest)
{
  RecordProperty("description","This test succeeds if the integral error is prevented from winding up when the integral gain is zero. If the integral error is allowed to wind up while it is disabled, it can cause sudden jumps to the minimum or maximum bound in control command when re-enabled.");

  double i_gain = 0.0;
  double i_min = -1.0;
  double i_max = 1.0;
  Pid pid(0.0, i_gain, 0.0, i_max, i_min);

  double cmd = 0.0;
  double pe,ie,de;

  cmd = pid.updatePid(-1.0, ros::Duration(1.0));
  pid.getCurrentPIDErrors(&pe,&ie,&de);
  EXPECT_LE(i_min, ie);
  EXPECT_LE(ie, i_max);
  EXPECT_EQ(0.0, cmd);

  cmd = pid.updatePid(-1.0, ros::Duration(1.0));
  pid.getCurrentPIDErrors(&pe,&ie,&de);
  EXPECT_LE(i_min, ie);
  EXPECT_LE(ie, i_max);
  EXPECT_EQ(0.0, cmd);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
