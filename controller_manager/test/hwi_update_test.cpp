///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2019
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Fraunhofer IPA nor the names of its
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

/// \author Jordan Palacios

#include <hardware_interface/robot_hw.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager/controller_manager.h>

#include <ros/ros.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using ::testing::StrictMock;
using ::testing::_;
using ::testing::Return;

namespace hardware_interface
{
class RobotHWMock : public RobotHW
{
public:
  RobotHWMock()
  {
  }
  ~RobotHWMock()
  {
  }

  MOCK_METHOD2(init, bool(ros::NodeHandle &, ros::NodeHandle &));
  MOCK_CONST_METHOD1(checkForConflict, bool(const std::list<ControllerInfo> &));
  MOCK_METHOD2(prepareSwitch,
               bool(const std::list<ControllerInfo> &, const std::list<ControllerInfo> &));
  MOCK_METHOD2(doSwitch,
               void(const std::list<ControllerInfo> &, const std::list<ControllerInfo> &));
  MOCK_METHOD2(read, void(const ros::Time &time, const ros::Duration &period));
  MOCK_METHOD2(write, void(const ros::Time &time, const ros::Duration &period));
};
}

void update(controller_manager::ControllerManager &cm, const ros::TimerEvent &e)
{
  cm.update(e.current_real, e.current_real - e.last_real);
}

TEST(UpdateControllerManagerTest, NoSwitchTest)
{
  StrictMock<hardware_interface::RobotHWMock> hw_mock;
  controller_manager::ControllerManager cm(&hw_mock);

  const ros::Duration period(1.0);

  EXPECT_CALL(hw_mock, doSwitch(_, _)).Times(0);

  cm.update(ros::Time::now(), period);
}

TEST(UpdateControllerManagerTest, SwitchTest)
{
  StrictMock<hardware_interface::RobotHWMock> hw_mock;
  controller_manager::ControllerManager cm(&hw_mock);

  // timer that calls controller manager's update
  ros::NodeHandle node_handle;
  ros::Timer timer =
      node_handle.createTimer(ros::Duration(0.01), boost::bind(update, boost::ref(cm), _1));

  EXPECT_CALL(hw_mock, checkForConflict(_)).Times(1).WillOnce(Return(false));
  EXPECT_CALL(hw_mock, prepareSwitch(_, _)).Times(1).WillOnce(Return(true));
  EXPECT_CALL(hw_mock, doSwitch(_, _)).Times(1);

  // only way to trigger switch is through switchController(...) which in turn waits for
  // update(...) to finish the switch
  std::vector<std::string> start_controllers, stop_controllers;
  const int strictness = controller_manager_msgs::SwitchController::Request::STRICT;
  ASSERT_TRUE(cm.switchController(start_controllers, stop_controllers, strictness));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "controller_manager_update_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  return ret;
}
