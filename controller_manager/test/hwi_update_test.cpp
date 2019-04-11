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
#include <controller_interface/controller_base.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager/controller_loader_interface.h>
#include <controller_manager/controller_manager.h>

#include <ros/ros.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using ::testing::StrictMock;
using ::testing::_;
using ::testing::Return;
using ::testing::AtLeast;
using ::testing::AnyNumber;

class RobotHWMock : public hardware_interface::RobotHW
{
public:
  RobotHWMock()
  {
  }
  ~RobotHWMock()
  {
  }

  MOCK_METHOD2(init, bool(ros::NodeHandle &, ros::NodeHandle &));
  MOCK_CONST_METHOD1(checkForConflict,
                     bool(const std::list<hardware_interface::ControllerInfo> &));
  MOCK_METHOD2(prepareSwitch, bool(const std::list<hardware_interface::ControllerInfo> &,
                                   const std::list<hardware_interface::ControllerInfo> &));
  MOCK_METHOD2(doSwitch, void(const std::list<hardware_interface::ControllerInfo> &,
                              const std::list<hardware_interface::ControllerInfo> &));
  MOCK_METHOD2(read, void(const ros::Time &time, const ros::Duration &period));
  MOCK_METHOD2(write, void(const ros::Time &time, const ros::Duration &period));
};

class ControllerLoaderMock : public controller_manager::ControllerLoaderInterface
{
public:
  ControllerLoaderMock() : ControllerLoaderInterface("ControllerLoaderMock")
  {
  }
  ~ControllerLoaderMock()
  {
  }

  MOCK_METHOD1(createInstance,
               controller_interface::ControllerBaseSharedPtr(const std::string &));
  MOCK_METHOD0(getDeclaredClasses, std::vector<std::string>(void));
  MOCK_METHOD0(reload, void(void));
};

class ControllerMock : public controller_interface::ControllerBase
{
public:
  ControllerMock()
  {
  }
  ~ControllerMock()
  {
  }

  MOCK_METHOD1(starting, void(const ros::Time &));
  MOCK_METHOD2(update, void(const ros::Time &, const ros::Duration &));
  MOCK_METHOD1(stopping, void(const ros::Time &));
  MOCK_METHOD4(initRequest, bool(hardware_interface::RobotHW *, ros::NodeHandle &,
                                 ros::NodeHandle &, ClaimedResources &));
};


void update(controller_manager::ControllerManager &cm, const ros::TimerEvent &e)
{
  cm.update(e.current_real, e.current_real - e.last_real);
}

TEST(UpdateControllerManagerTest, NoSwitchTest)
{
  StrictMock<RobotHWMock> hw_mock;
  controller_manager::ControllerManager cm(&hw_mock);

  const ros::Duration period(1.0);

  EXPECT_CALL(hw_mock, doSwitch(_, _)).Times(0);

  cm.update(ros::Time::now(), period);
}

TEST(UpdateControllerManagerTest, SwitchOnlyTest)
{
  StrictMock<RobotHWMock> hw_mock;

  controller_manager::ControllerManager cm(&hw_mock);

  // timer that calls controller manager's update
  ros::NodeHandle node_handle;
  ros::Timer timer =
      node_handle.createTimer(ros::Duration(0.01), boost::bind(update, boost::ref(cm), _1));

  EXPECT_CALL(hw_mock, checkForConflict(_)).Times(1).WillOnce(Return(false));
  EXPECT_CALL(hw_mock, prepareSwitch(_, _)).Times(1).WillOnce(Return(true));
  EXPECT_CALL(hw_mock, doSwitch(_, _)).Times(1);

  // only way to trigger switch is through switchController(...) which in turn waits for
  // update(...) to finish the switch, hence the update on a timer
  const std::vector<std::string> start_controllers, stop_controllers;
  const int strictness = controller_manager_msgs::SwitchController::Request::STRICT;
  ASSERT_TRUE(cm.switchController(start_controllers, stop_controllers, strictness));
}

TEST(UpdateControllerManagerTest, SwitchWithControllersTest)
{
  StrictMock<RobotHWMock> hw_mock;

  StrictMock<ControllerLoaderMock> *ctrl_loader_mock = new StrictMock<ControllerLoaderMock>();
  controller_manager::ControllerLoaderInterfaceSharedPtr ctrl_loader_mock_ptr(ctrl_loader_mock);

  StrictMock<ControllerMock> *ctrl_1_mock = new StrictMock<ControllerMock>();
  StrictMock<ControllerMock> *ctrl_2_mock = new StrictMock<ControllerMock>();
  controller_interface::ControllerBaseSharedPtr ctrl_1_mock_ptr(ctrl_1_mock);
  controller_interface::ControllerBaseSharedPtr ctrl_2_mock_ptr(ctrl_2_mock);

  controller_manager::ControllerManager cm(&hw_mock);

  // timer that calls controller manager's update
  ros::NodeHandle node_handle;
  ros::Timer timer =
      node_handle.createTimer(ros::Duration(0.01), boost::bind(update, boost::ref(cm), _1));

  // register mock controller loader
  cm.registerControllerLoader(ctrl_loader_mock_ptr);

  const std::vector<std::string> types = { "ControllerMock" };
  EXPECT_CALL(*ctrl_loader_mock, getDeclaredClasses()).Times(2).WillRepeatedly(Return(types));
  EXPECT_CALL(*ctrl_loader_mock, createInstance("ControllerMock"))
      .Times(2)
      .WillOnce(Return(ctrl_1_mock_ptr))
      .WillOnce(Return(ctrl_2_mock_ptr));

  EXPECT_CALL(*ctrl_1_mock, initRequest(_, _, _, _)).Times(1).WillOnce(Return(true));
  EXPECT_CALL(*ctrl_2_mock, initRequest(_, _, _, _)).Times(1).WillOnce(Return(true));

  // load mock controllers
  ASSERT_TRUE(cm.loadController("mock_ctrl_1"));
  ASSERT_TRUE(cm.loadController("mock_ctrl_2"));

  EXPECT_CALL(hw_mock, checkForConflict(_)).Times(2).WillOnce(Return(false));
  EXPECT_CALL(hw_mock, prepareSwitch(_, _)).Times(2).WillRepeatedly(Return(true));
  EXPECT_CALL(hw_mock, doSwitch(_, _)).Times(2);

  // only way to trigger switch is through switchController(...) which in turn waits for
  // update(...) to finish the switch, hence the update on a timer

  const int strictness = controller_manager_msgs::SwitchController::Request::STRICT;
  std::vector<std::string> start_controllers, stop_controllers;

  /// @todo do this on initRequest() expect call?
  ctrl_1_mock->state_ = ctrl_1_mock->INITIALIZED;
  ctrl_2_mock->state_ = ctrl_2_mock->INITIALIZED;

  EXPECT_CALL(*ctrl_1_mock, update(_, _)).Times(AtLeast(1));
  // this may or may not be called depending on the timing of the update timer
  EXPECT_CALL(*ctrl_2_mock, update(_, _)).Times(AnyNumber());

  // start controller
  EXPECT_CALL(*ctrl_1_mock, starting(_)).Times(1);

  start_controllers = { "mock_ctrl_1" };
  ASSERT_TRUE(cm.switchController(start_controllers, stop_controllers, strictness));

  // stop and start controllers
  EXPECT_CALL(*ctrl_1_mock, stopping(_)).Times(1);
  EXPECT_CALL(*ctrl_2_mock, starting(_)).Times(1);

  start_controllers = { "mock_ctrl_2" };
  stop_controllers = { "mock_ctrl_1" };
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
