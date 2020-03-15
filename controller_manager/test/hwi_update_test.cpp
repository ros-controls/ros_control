///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2019, Pal Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Pal Robotics S.L. nor the names of its
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

#include <memory>
#include <functional>

using ::testing::StrictMock;
using ::testing::_;
using ::testing::AtLeast;
using ::testing::AnyNumber;
using ::testing::DoAll;
using ::testing::InvokeWithoutArgs;
using ::testing::Return;

class RobotHWMock : public hardware_interface::RobotHW
{
public:

  MOCK_METHOD2(init, bool(ros::NodeHandle &, ros::NodeHandle &));
  MOCK_CONST_METHOD1(checkForConflict,
                     bool(const std::list<hardware_interface::ControllerInfo> &));
  MOCK_METHOD2(prepareSwitch, bool(const std::list<hardware_interface::ControllerInfo> &,
                                   const std::list<hardware_interface::ControllerInfo> &));
  MOCK_METHOD2(doSwitch, void(const std::list<hardware_interface::ControllerInfo> &,
                              const std::list<hardware_interface::ControllerInfo> &));
  MOCK_CONST_METHOD0(switchResult, SwitchState(void));
  MOCK_CONST_METHOD1(switchResult, SwitchState(const hardware_interface::ControllerInfo &));
  MOCK_METHOD2(read, void(const ros::Time &time, const ros::Duration &period));
  MOCK_METHOD2(write, void(const ros::Time &time, const ros::Duration &period));
};

class ControllerLoaderMock : public controller_manager::ControllerLoaderInterface
{
public:
  ControllerLoaderMock() : ControllerLoaderInterface("ControllerLoaderMock") {}

  MOCK_METHOD1(createInstance,
               controller_interface::ControllerBaseSharedPtr(const std::string &));
  MOCK_METHOD0(getDeclaredClasses, std::vector<std::string>(void));
  MOCK_METHOD0(reload, void(void));
};

class ControllerMock : public controller_interface::ControllerBase
{
public:

  void initializeState()
  {
    state_ = INITIALIZED;
  }

  MOCK_METHOD1(starting, void(const ros::Time &));
  MOCK_METHOD2(update, void(const ros::Time &, const ros::Duration &));
  MOCK_METHOD1(stopping, void(const ros::Time &));
  MOCK_METHOD1(waiting, void(const ros::Time &));
  MOCK_METHOD1(aborting, void(const ros::Time &));
  MOCK_METHOD4(initRequest, bool(hardware_interface::RobotHW *, ros::NodeHandle &,
                                 ros::NodeHandle &, ClaimedResources &));
};

class ControllerManagerTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    prepareMocks();
    loadControllers();
  }

  void prepareMocks()
  {
    // prepare robot hw mock
    hw_mock_.reset(new StrictMock<RobotHWMock>);

    // prepare controller loader mock
    ctrl_loader_mock_ = new StrictMock<ControllerLoaderMock>();
    ctrl_loader_mock_ptr_.reset(ctrl_loader_mock_);

    // prepare controllers mocks
    ctrl_1_mock_ = new StrictMock<ControllerMock>();
    ctrl_2_mock_ = new StrictMock<ControllerMock>();
    ctrl_1_mock_ptr_.reset(ctrl_1_mock_);
    ctrl_2_mock_ptr_.reset(ctrl_2_mock_);

    // create controller manager
    cm_.reset(new controller_manager::ControllerManager(hw_mock_.get()));
  }

  void loadControllers()
  {
    // register mock controller loader
    cm_->registerControllerLoader(ctrl_loader_mock_ptr_);

    const std::vector<std::string> types = { "ControllerMock" };
    EXPECT_CALL(*ctrl_loader_mock_, getDeclaredClasses()).Times(2).WillRepeatedly(Return(types));
    EXPECT_CALL(*ctrl_loader_mock_, createInstance("ControllerMock"))
        .Times(2)
        .WillOnce(Return(ctrl_1_mock_ptr_))
        .WillOnce(Return(ctrl_2_mock_ptr_));

    EXPECT_CALL(*ctrl_1_mock_, initRequest(_, _, _, _))
        .Times(1)
        .WillOnce(DoAll(InvokeWithoutArgs(ctrl_1_mock_, &ControllerMock::initializeState),
                        Return(true)));
    EXPECT_CALL(*ctrl_2_mock_, initRequest(_, _, _, _))
        .Times(1)
        .WillOnce(DoAll(InvokeWithoutArgs(ctrl_2_mock_, &ControllerMock::initializeState),
                        Return(true)));

    // load mock controllers
    ASSERT_TRUE(cm_->loadController("mock_ctrl_1"));
    ASSERT_TRUE(cm_->loadController("mock_ctrl_2"));
  }

  // robot hw mock
  std::unique_ptr<StrictMock<RobotHWMock>> hw_mock_;

  // controller loader mock
  StrictMock<ControllerLoaderMock> *ctrl_loader_mock_;
  controller_manager::ControllerLoaderInterfaceSharedPtr ctrl_loader_mock_ptr_;

  // controllers mocks
  StrictMock<ControllerMock> *ctrl_1_mock_;
  StrictMock<ControllerMock> *ctrl_2_mock_;
  controller_interface::ControllerBaseSharedPtr ctrl_1_mock_ptr_;
  controller_interface::ControllerBaseSharedPtr ctrl_2_mock_ptr_;

  // controller manager
  std::shared_ptr<controller_manager::ControllerManager> cm_;
};

void update(std::shared_ptr<controller_manager::ControllerManager> cm, const ros::TimerEvent &e)
{
  cm->update(e.current_real, e.current_real - e.last_real);
}

TEST_F(ControllerManagerTest, NoSwitchTest)
{
  const ros::Duration period(1.0);

  EXPECT_CALL(*hw_mock_, doSwitch(_, _)).Times(0);

  cm_->update(ros::Time::now(), period);
}

TEST_F(ControllerManagerTest, SwitchOnlyTest)
{
  // only way to trigger switch is through switchController(...) which in turn waits
  // for update(...) to finish the switch, hence the update on a timer
  ros::NodeHandle node_handle;
  ros::Timer timer = node_handle.createTimer(ros::Duration(0.01),
                                             std::bind(update, cm_, std::placeholders::_1));

  EXPECT_CALL(*hw_mock_, checkForConflict(_)).Times(1).WillOnce(Return(false));
  EXPECT_CALL(*hw_mock_, prepareSwitch(_, _)).Times(1).WillOnce(Return(true));
  EXPECT_CALL(*hw_mock_, doSwitch(_, _)).Times(1);
  EXPECT_CALL(*hw_mock_, switchResult()).Times(1).WillOnce(Return(RobotHWMock::DONE));

  // switch with no controllers at all
  const std::vector<std::string> start_controllers, stop_controllers;
  const int strictness = controller_manager_msgs::SwitchController::Request::STRICT;
  ASSERT_TRUE(cm_->switchController(start_controllers, stop_controllers, strictness));
}

TEST_F(ControllerManagerTest, SwitchControllersTest)
{
  // only way to trigger switch is through switchController(...) which in turn waits for
  // update(...) to finish the switch, hence the update on a timer
  ros::NodeHandle node_handle;
  ros::Timer timer = node_handle.createTimer(ros::Duration(0.01),
                                             std::bind(update, cm_, std::placeholders::_1));

  // one call for each controller
  EXPECT_CALL(*hw_mock_, checkForConflict(_)).Times(2).WillRepeatedly(Return(false));
  EXPECT_CALL(*hw_mock_, prepareSwitch(_, _)).Times(2).WillRepeatedly(Return(true));
  EXPECT_CALL(*hw_mock_, doSwitch(_, _)).Times(2);
  EXPECT_CALL(*hw_mock_, switchResult()).Times(2).WillRepeatedly(Return(RobotHWMock::DONE));

  const int strictness = controller_manager_msgs::SwitchController::Request::STRICT;
  std::vector<std::string> start_controllers, stop_controllers;

  // called at least once, otherwise can't perform second switch
  EXPECT_CALL(*ctrl_1_mock_, update(_, _)).Times(AtLeast(1));
  // this may or may not be called depending on the timing of the update timer
  EXPECT_CALL(*ctrl_2_mock_, update(_, _)).Times(AnyNumber());

  // start controller
  EXPECT_CALL(*ctrl_1_mock_, starting(_)).Times(1);

  start_controllers = { "mock_ctrl_1" };
  ASSERT_TRUE(cm_->switchController(start_controllers, stop_controllers, strictness));
  ASSERT_TRUE(ctrl_1_mock_->isRunning());

  // stop and start controllers
  EXPECT_CALL(*ctrl_1_mock_, stopping(_)).Times(1);
  EXPECT_CALL(*ctrl_2_mock_, starting(_)).Times(1);

  start_controllers = { "mock_ctrl_2" };
  stop_controllers = { "mock_ctrl_1" };
  ASSERT_TRUE(cm_->switchController(start_controllers, stop_controllers, strictness));
  ASSERT_TRUE(ctrl_1_mock_->isStopped());
  ASSERT_TRUE(ctrl_2_mock_->isRunning());
}

TEST_F(ControllerManagerTest, SwitchControllersAbortTest)
{
  // only way to trigger switch is through switchController(...) which in turn waits for
  // update(...) to finish the switch, hence the update on a timer
  ros::NodeHandle node_handle;
  ros::Timer timer = node_handle.createTimer(ros::Duration(0.01),
                                             std::bind(update, cm_, std::placeholders::_1));

  // one call for each controller
  EXPECT_CALL(*hw_mock_, checkForConflict(_)).Times(1).WillOnce(Return(false));
  EXPECT_CALL(*hw_mock_, prepareSwitch(_, _)).Times(1).WillOnce(Return(true));
  EXPECT_CALL(*hw_mock_, doSwitch(_, _)).Times(1);
  EXPECT_CALL(*hw_mock_, switchResult()).Times(2).WillRepeatedly(Return(RobotHWMock::ERROR));

  const int strictness = controller_manager_msgs::SwitchController::Request::STRICT;
  std::vector<std::string> start_controllers, stop_controllers;

  // controllers are aborted before they can update
  EXPECT_CALL(*ctrl_1_mock_, update(_, _)).Times(0);
  EXPECT_CALL(*ctrl_2_mock_, update(_, _)).Times(0);

  // abort controller
  EXPECT_CALL(*ctrl_1_mock_, aborting(_)).Times(1);
  EXPECT_CALL(*ctrl_2_mock_, aborting(_)).Times(1);

  start_controllers = { "mock_ctrl_1", "mock_ctrl_2" };
  ASSERT_TRUE(cm_->switchController(start_controllers, stop_controllers, strictness));
  ASSERT_TRUE(ctrl_1_mock_->isAborted());
  ASSERT_TRUE(ctrl_2_mock_->isAborted());
}

TEST_F(ControllerManagerTest, SwitchControllersTimeoutTest)
{
  // only way to trigger switch is through switchController(...) which in turn waits for
  // update(...) to finish the switch, hence the update on a timer
  ros::NodeHandle node_handle;
  ros::Timer timer = node_handle.createTimer(ros::Duration(0.01),
                                             std::bind(update, cm_, std::placeholders::_1));

  // one call for each controller
  EXPECT_CALL(*hw_mock_, checkForConflict(_)).Times(1).WillOnce(Return(false));
  EXPECT_CALL(*hw_mock_, prepareSwitch(_, _)).Times(1).WillOnce(Return(true));
  EXPECT_CALL(*hw_mock_, doSwitch(_, _)).Times(1);
  EXPECT_CALL(*hw_mock_, switchResult()).WillRepeatedly(Return(RobotHWMock::ONGOING));

  const int strictness = controller_manager_msgs::SwitchController::Request::STRICT;
  std::vector<std::string> start_controllers, stop_controllers;

  // controllers are never started
  EXPECT_CALL(*ctrl_1_mock_, update(_, _)).Times(0);
  EXPECT_CALL(*ctrl_2_mock_, update(_, _)).Times(0);

  // called while hardware interfaces are ONGOING
  EXPECT_CALL(*ctrl_1_mock_, waiting(_)).Times(AtLeast(1));
  EXPECT_CALL(*ctrl_2_mock_, waiting(_)).Times(AtLeast(1));

  // abort controller
  EXPECT_CALL(*ctrl_1_mock_, aborting(_)).Times(1);
  EXPECT_CALL(*ctrl_2_mock_, aborting(_)).Times(1);

  start_controllers = { "mock_ctrl_1", "mock_ctrl_2" };
  ASSERT_TRUE(cm_->switchController(start_controllers, stop_controllers, strictness, false, 1.0));
  ASSERT_TRUE(ctrl_1_mock_->isAborted());
  ASSERT_TRUE(ctrl_2_mock_->isAborted());
}

TEST_F(ControllerManagerTest, ControllerUpdatesTest)
{
  // only way to trigger switch is through switchController(...) which in turn waits for
  // update(...) to finish the switch, hence the update on a timer
  ros::NodeHandle node_handle;
  ros::Timer timer = node_handle.createTimer(ros::Duration(0.01),
                                             std::bind(update, cm_, std::placeholders::_1));

  EXPECT_CALL(*hw_mock_, checkForConflict(_)).Times(1).WillOnce(Return(false));
  EXPECT_CALL(*hw_mock_, prepareSwitch(_, _)).Times(1).WillOnce(Return(true));
  EXPECT_CALL(*hw_mock_, doSwitch(_, _)).Times(1);
  EXPECT_CALL(*hw_mock_, switchResult())
      .Times(11)
      .WillOnce(Return(RobotHWMock::ONGOING))
      .WillOnce(Return(RobotHWMock::ONGOING))
      .WillOnce(Return(RobotHWMock::ONGOING))
      .WillOnce(Return(RobotHWMock::ONGOING))
      .WillOnce(Return(RobotHWMock::ONGOING))
      .WillOnce(Return(RobotHWMock::ONGOING))
      .WillOnce(Return(RobotHWMock::ONGOING))
      .WillOnce(Return(RobotHWMock::ONGOING))
      .WillOnce(Return(RobotHWMock::ONGOING))
      .WillOnce(Return(RobotHWMock::ONGOING))
      .WillOnce(Return(RobotHWMock::DONE));

  const int strictness = controller_manager_msgs::SwitchController::Request::STRICT;
  std::vector<std::string> start_controllers, stop_controllers;

  // controller started
  EXPECT_CALL(*ctrl_1_mock_, update(_, _)).Times(AtLeast(5));
  // called 5 times, once for each ONGOING
  EXPECT_CALL(*ctrl_1_mock_, waiting(_)).Times(5);
  // controller not started
  EXPECT_CALL(*ctrl_2_mock_, update(_, _)).Times(0);

  // start controller
  EXPECT_CALL(*ctrl_1_mock_, starting(_)).Times(1);

  start_controllers = { "mock_ctrl_1" };
  ASSERT_TRUE(cm_->switchController(start_controllers, stop_controllers, strictness));
  ASSERT_TRUE(ctrl_1_mock_->isRunning());

  timer.stop();

  const ros::Duration period(1.0);
  for (unsigned int i = 0; i < 5; ++i)
  {
    cm_->update(ros::Time::now(), period);
  }
}

TEST_F(ControllerManagerTest, SwitchControllersAsapTest)
{
  // only way to trigger switch is through switchController(...) which in turn waits for
  // update(...) to finish the switch, hence the update on a timer
  ros::NodeHandle node_handle;
  ros::Timer timer = node_handle.createTimer(ros::Duration(0.01),
                                             std::bind(update, cm_, std::placeholders::_1));

  // one call for each controller
  EXPECT_CALL(*hw_mock_, checkForConflict(_)).Times(1).WillRepeatedly(Return(false));
  EXPECT_CALL(*hw_mock_, prepareSwitch(_, _)).Times(1).WillRepeatedly(Return(true));
  EXPECT_CALL(*hw_mock_, doSwitch(_, _)).Times(1);
  EXPECT_CALL(*hw_mock_, switchResult()).Times(0);
  EXPECT_CALL(*hw_mock_, switchResult(_))
      .Times(12)
      .WillOnce(Return(RobotHWMock::DONE))
      .WillOnce(Return(RobotHWMock::ONGOING))
      .WillOnce(Return(RobotHWMock::ONGOING))
      .WillOnce(Return(RobotHWMock::ONGOING))
      .WillOnce(Return(RobotHWMock::ONGOING))
      .WillOnce(Return(RobotHWMock::ONGOING))
      .WillOnce(Return(RobotHWMock::ONGOING))
      .WillOnce(Return(RobotHWMock::ONGOING))
      .WillOnce(Return(RobotHWMock::ONGOING))
      .WillOnce(Return(RobotHWMock::ONGOING))
      .WillOnce(Return(RobotHWMock::ONGOING))
      .WillOnce(Return(RobotHWMock::DONE));

  const int strictness = controller_manager_msgs::SwitchController::Request::STRICT;
  std::vector<std::string> start_controllers, stop_controllers;

  // called at least 5 times, once for each ONGOING
  EXPECT_CALL(*ctrl_1_mock_, update(_, _)).Times(AtLeast(5));
  // this may or may not be called depending on the timing of the update timer
  EXPECT_CALL(*ctrl_2_mock_, update(_, _)).Times(AnyNumber());

  // both controllers are started
  EXPECT_CALL(*ctrl_1_mock_, starting(_)).Times(1);
  EXPECT_CALL(*ctrl_2_mock_, starting(_)).Times(1);
  // called 5 times, once for each ONGOING
  EXPECT_CALL(*ctrl_2_mock_, waiting(_)).Times(5);

  start_controllers = { "mock_ctrl_1", "mock_ctrl_2" };
  ASSERT_TRUE(cm_->switchController(start_controllers, stop_controllers, strictness, true));
  ASSERT_TRUE(ctrl_1_mock_->isRunning());
  ASSERT_TRUE(ctrl_2_mock_->isRunning());
}

TEST_F(ControllerManagerTest, SwitchControllersAsapAbortTest)
{
  // only way to trigger switch is through switchController(...) which in turn waits for
  // update(...) to finish the switch, hence the update on a timer
  ros::NodeHandle node_handle;
  ros::Timer timer = node_handle.createTimer(ros::Duration(0.01),
                                             std::bind(update, cm_, std::placeholders::_1));

  // one call for each controller
  EXPECT_CALL(*hw_mock_, checkForConflict(_)).Times(1).WillRepeatedly(Return(false));
  EXPECT_CALL(*hw_mock_, prepareSwitch(_, _)).Times(1).WillRepeatedly(Return(true));
  EXPECT_CALL(*hw_mock_, doSwitch(_, _)).Times(1);
  EXPECT_CALL(*hw_mock_, switchResult()).Times(0);
  EXPECT_CALL(*hw_mock_, switchResult(_))
      .Times(13)
      .WillOnce(Return(RobotHWMock::DONE))
      .WillOnce(Return(RobotHWMock::ONGOING))
      .WillOnce(Return(RobotHWMock::ONGOING))
      .WillOnce(Return(RobotHWMock::ONGOING))
      .WillOnce(Return(RobotHWMock::ONGOING))
      .WillOnce(Return(RobotHWMock::ONGOING))
      .WillOnce(Return(RobotHWMock::ONGOING))
      .WillOnce(Return(RobotHWMock::ONGOING))
      .WillOnce(Return(RobotHWMock::ONGOING))
      .WillOnce(Return(RobotHWMock::ONGOING))
      .WillOnce(Return(RobotHWMock::ONGOING))
      .WillOnce(Return(RobotHWMock::ERROR))
      .WillOnce(Return(RobotHWMock::ERROR));

  const int strictness = controller_manager_msgs::SwitchController::Request::STRICT;
  std::vector<std::string> start_controllers, stop_controllers;

  // called at least 5 times, once for each ONGOING
  EXPECT_CALL(*ctrl_1_mock_, update(_, _)).Times(AtLeast(5));
  // this may or may not be called depending on the timing of the update timer
  EXPECT_CALL(*ctrl_2_mock_, update(_, _)).Times(AnyNumber());

  // one controller manages to start, the other is aborted
  EXPECT_CALL(*ctrl_1_mock_, starting(_)).Times(1);
  EXPECT_CALL(*ctrl_2_mock_, aborting(_)).Times(1);
  // called 5 times, once for each ONGOING
  EXPECT_CALL(*ctrl_2_mock_, waiting(_)).Times(5);

  start_controllers = { "mock_ctrl_1", "mock_ctrl_2" };
  ASSERT_TRUE(cm_->switchController(start_controllers, stop_controllers, strictness, true));
  ASSERT_TRUE(ctrl_1_mock_->isRunning());
  ASSERT_TRUE(ctrl_2_mock_->isAborted());
}

TEST_F(ControllerManagerTest, SwitchControllersAsapTimeoutTest)
{
  // only way to trigger switch is through switchController(...) which in turn waits for
  // update(...) to finish the switch, hence the update on a timer
  ros::NodeHandle node_handle;
  ros::Timer timer = node_handle.createTimer(ros::Duration(0.01),
                                             std::bind(update, cm_, std::placeholders::_1));

  // one call for each controller
  EXPECT_CALL(*hw_mock_, checkForConflict(_)).Times(1).WillRepeatedly(Return(false));
  EXPECT_CALL(*hw_mock_, prepareSwitch(_, _)).Times(1).WillRepeatedly(Return(true));
  EXPECT_CALL(*hw_mock_, doSwitch(_, _)).Times(1);
  EXPECT_CALL(*hw_mock_, switchResult()).Times(0);
  EXPECT_CALL(*hw_mock_, switchResult(_))
      .WillOnce(Return(RobotHWMock::DONE))
      .WillRepeatedly(Return(RobotHWMock::ONGOING));

  const int strictness = controller_manager_msgs::SwitchController::Request::STRICT;
  std::vector<std::string> start_controllers, stop_controllers;

  EXPECT_CALL(*ctrl_1_mock_, update(_, _)).Times(AtLeast(1));
  // this may or may not be called depending on the timing of the update timer
  EXPECT_CALL(*ctrl_2_mock_, update(_, _)).Times(AnyNumber());

  // one controller manages to start, the other is aborted
  EXPECT_CALL(*ctrl_1_mock_, starting(_)).Times(1);
  EXPECT_CALL(*ctrl_2_mock_, aborting(_)).Times(1);

  EXPECT_CALL(*ctrl_2_mock_, waiting(_)).Times(AtLeast(1));

  start_controllers = { "mock_ctrl_1", "mock_ctrl_2" };
  ASSERT_TRUE(cm_->switchController(start_controllers, stop_controllers, strictness, true, 1.0));
  ASSERT_TRUE(ctrl_1_mock_->isRunning());
  ASSERT_TRUE(ctrl_2_mock_->isAborted());
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
