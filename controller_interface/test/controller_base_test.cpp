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

#include <ros/ros.h>

#include <gmock/gmock.h>

using ::testing::StrictMock;
using ::testing::_;
using ::testing::DoAll;
using ::testing::InvokeWithoutArgs;
using ::testing::Return;

class ControllerMock : public controller_interface::ControllerBase
{
public:

  void initializeState()
  {
    state_ = INITIALIZED;
  }

  MOCK_METHOD1(starting, void(const ros::Time&));
  MOCK_METHOD2(update, void(const ros::Time&, const ros::Duration&));
  MOCK_METHOD1(stopping, void(const ros::Time&));
  MOCK_METHOD1(waiting, void(const ros::Time&));
  MOCK_METHOD1(aborting, void(const ros::Time&));
  MOCK_METHOD4(initRequest, bool(hardware_interface::RobotHW*, ros::NodeHandle&,
                                 ros::NodeHandle&, ClaimedResources&));
};

TEST(ControllerBaseAPI, DefaultStateTest)
{
  StrictMock<ControllerMock> controller;

  ASSERT_FALSE(controller.isInitialized());
  ASSERT_FALSE(controller.isRunning());
  ASSERT_FALSE(controller.isStopped());
  ASSERT_FALSE(controller.isWaiting());
  ASSERT_FALSE(controller.isAborted());
}

TEST(ControllerBaseAPI, IsInitializedTest)
{
  StrictMock<ControllerMock> controller;

  hardware_interface::RobotHW* robot_hw = nullptr;
  ros::NodeHandle* node_handle = nullptr;
  ControllerMock::ClaimedResources resources;
  const ros::Time time;

  EXPECT_CALL(controller, initRequest(_, _, _, _))
      .Times(1)
      .WillOnce(DoAll(InvokeWithoutArgs(&controller, &ControllerMock::initializeState),
                      Return(true)));

  // not initialized by default
  ASSERT_FALSE(controller.isInitialized());

  ASSERT_TRUE(controller.initRequest(robot_hw, *node_handle, *node_handle, resources));
  ASSERT_TRUE(controller.isInitialized());
}

TEST(ControllerBaseAPI, IsRunningTest)
{
  StrictMock<ControllerMock> controller;

  hardware_interface::RobotHW* robot_hw = nullptr;
  ros::NodeHandle* node_handle = nullptr;
  ControllerMock::ClaimedResources resources;
  const ros::Time time;

  EXPECT_CALL(controller, starting(_)).Times(1);
  EXPECT_CALL(controller, stopping(_)).Times(1);
  EXPECT_CALL(controller, initRequest(_, _, _, _))
      .Times(1)
      .WillOnce(DoAll(InvokeWithoutArgs(&controller, &ControllerMock::initializeState),
                      Return(true)));

  // not running by default
  ASSERT_FALSE(controller.isRunning());

  ASSERT_TRUE(controller.initRequest(robot_hw, *node_handle, *node_handle, resources));
  ASSERT_TRUE(controller.startRequest(time));
  ASSERT_TRUE(controller.isRunning());

  ASSERT_TRUE(controller.stopRequest(time));
  ASSERT_FALSE(controller.isRunning());
}

TEST(ControllerBaseAPI, IsStoppedTest)
{
  StrictMock<ControllerMock> controller;

  hardware_interface::RobotHW* robot_hw = nullptr;
  ros::NodeHandle* node_handle = nullptr;
  ControllerMock::ClaimedResources resources;
  const ros::Time time;

  EXPECT_CALL(controller, starting(_)).Times(1);
  EXPECT_CALL(controller, stopping(_)).Times(1);
  EXPECT_CALL(controller, initRequest(_, _, _, _))
      .Times(1)
      .WillOnce(DoAll(InvokeWithoutArgs(&controller, &ControllerMock::initializeState),
                      Return(true)));

  // not stopped by default
  ASSERT_FALSE(controller.isStopped());

  ASSERT_TRUE(controller.initRequest(robot_hw, *node_handle, *node_handle, resources));
  ASSERT_TRUE(controller.stopRequest(time));
  ASSERT_TRUE(controller.isStopped());

  ASSERT_TRUE(controller.startRequest(time));
  ASSERT_FALSE(controller.isStopped());
}

TEST(ControllerBaseAPI, IsWaitingTest)
{
  StrictMock<ControllerMock> controller;

  hardware_interface::RobotHW* robot_hw = nullptr;
  ros::NodeHandle* node_handle = nullptr;
  ControllerMock::ClaimedResources resources;
  const ros::Time time;

  EXPECT_CALL(controller, starting(_)).Times(1);
  EXPECT_CALL(controller, waiting(_)).Times(1);
  EXPECT_CALL(controller, initRequest(_, _, _, _))
      .Times(1)
      .WillOnce(DoAll(InvokeWithoutArgs(&controller, &ControllerMock::initializeState),
                      Return(true)));

  // not waiting by default
  ASSERT_FALSE(controller.isWaiting());

  ASSERT_TRUE(controller.initRequest(robot_hw, *node_handle, *node_handle, resources));
  ASSERT_TRUE(controller.waitRequest(time));
  ASSERT_TRUE(controller.isWaiting());

  ASSERT_TRUE(controller.startRequest(time));
  ASSERT_FALSE(controller.isWaiting());
}

TEST(ControllerBaseAPI, StartRequestTest)
{
  StrictMock<ControllerMock> controller;

  hardware_interface::RobotHW* robot_hw = nullptr;
  ros::NodeHandle* node_handle = nullptr;
  ControllerMock::ClaimedResources resources;
  const ros::Time time;

  EXPECT_CALL(controller, starting(_)).Times(2);
  EXPECT_CALL(controller, initRequest(_, _, _, _))
      .Times(1)
      .WillOnce(DoAll(InvokeWithoutArgs(&controller, &ControllerMock::initializeState),
                      Return(true)));

  // not initialized
  ASSERT_FALSE(controller.startRequest(time));

  ASSERT_TRUE(controller.initRequest(robot_hw, *node_handle, *node_handle, resources));
  ASSERT_TRUE(controller.startRequest(time));

  // can initialize multiple times
  ASSERT_TRUE(controller.startRequest(time));
}

TEST(ControllerBaseAPI, StopRequestTest)
{
  StrictMock<ControllerMock> controller;

  hardware_interface::RobotHW* robot_hw = nullptr;
  ros::NodeHandle* node_handle = nullptr;
  ControllerMock::ClaimedResources resources;
  const ros::Time time;

  EXPECT_CALL(controller, stopping(_)).Times(2);
  EXPECT_CALL(controller, initRequest(_, _, _, _))
      .Times(1)
      .WillOnce(DoAll(InvokeWithoutArgs(&controller, &ControllerMock::initializeState),
                      Return(true)));

  // not initialized
  ASSERT_FALSE(controller.stopRequest(time));

  ASSERT_TRUE(controller.initRequest(robot_hw, *node_handle, *node_handle, resources));
  ASSERT_TRUE(controller.stopRequest(time));

  // can stop multiple times
  ASSERT_TRUE(controller.stopRequest(time));
}

TEST(ControllerBaseAPI, WaitRequestTest)
{
  StrictMock<ControllerMock> controller;

  hardware_interface::RobotHW* robot_hw = nullptr;
  ros::NodeHandle* node_handle = nullptr;
  ControllerMock::ClaimedResources resources;
  const ros::Time time;

  EXPECT_CALL(controller, waiting(_)).Times(2);
  EXPECT_CALL(controller, initRequest(_, _, _, _))
      .Times(1)
      .WillOnce(DoAll(InvokeWithoutArgs(&controller, &ControllerMock::initializeState),
                      Return(true)));

  // not initialized
  ASSERT_FALSE(controller.waitRequest(time));

  ASSERT_TRUE(controller.initRequest(robot_hw, *node_handle, *node_handle, resources));
  ASSERT_TRUE(controller.waitRequest(time));

  // can wait multiple times
  ASSERT_TRUE(controller.waitRequest(time));
}

TEST(ControllerBaseAPI, AbortRequestTest)
{
  StrictMock<ControllerMock> controller;

  hardware_interface::RobotHW* robot_hw = nullptr;
  ros::NodeHandle* node_handle = nullptr;
  ControllerMock::ClaimedResources resources;
  const ros::Time time;

  EXPECT_CALL(controller, aborting(_)).Times(2);
  EXPECT_CALL(controller, initRequest(_, _, _, _))
      .Times(1)
      .WillOnce(DoAll(InvokeWithoutArgs(&controller, &ControllerMock::initializeState),
                      Return(true)));

  // not initialized
  ASSERT_FALSE(controller.abortRequest(time));

  ASSERT_TRUE(controller.initRequest(robot_hw, *node_handle, *node_handle, resources));
  ASSERT_TRUE(controller.abortRequest(time));

  // can abort multiple times
  ASSERT_TRUE(controller.abortRequest(time));
}

TEST(ControllerBaseAPI, UpdateRequestTest)
{
  StrictMock<ControllerMock> controller;

  hardware_interface::RobotHW* robot_hw = nullptr;
  ros::NodeHandle* node_handle = nullptr;
  ControllerMock::ClaimedResources resources;
  const ros::Time time;
  const ros::Duration period;

  EXPECT_CALL(controller, starting(_)).Times(1);
  EXPECT_CALL(controller, update(_, _)).Times(1);
  EXPECT_CALL(controller, initRequest(_, _, _, _))
      .Times(1)
      .WillOnce(DoAll(InvokeWithoutArgs(&controller, &ControllerMock::initializeState),
                      Return(true)));

  // update is not called unless state is RUNNING
  controller.updateRequest(time, period);

  ASSERT_TRUE(controller.initRequest(robot_hw, *node_handle, *node_handle, resources));
  ASSERT_TRUE(controller.startRequest(time));

  controller.updateRequest(time, period);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  int ret = RUN_ALL_TESTS();
  return ret;
}
