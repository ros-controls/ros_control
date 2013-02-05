
#include <ros/ros.h>

#include <gtest/gtest.h>

#include <control_toolbox/pid.h>

using namespace control_toolbox;

TEST(ParameterTest, zeroITermBadIBoundsTest)
{
  RecordProperty("description","This test checks robustness against poorly-defined integral term bounds.");

  Pid pid(1.0, 0.0, 0.0, -1.0, 0.0);

  double cmd = 0.0;
  double pe,ie,de;

  cmd = pid.updatePid(-1.0, ros::Duration(1.0));
  pid.getCurrentPIDErrors(&pe,&ie,&de);
  EXPECT_EQ(-1.0, ie);
  EXPECT_EQ(2.0, cmd);

  cmd = pid.updatePid(-1.0, ros::Duration(1.0));
  pid.getCurrentPIDErrors(&pe,&ie,&de);
  EXPECT_EQ(-2.0, ie);
  EXPECT_EQ(2.0, cmd);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
