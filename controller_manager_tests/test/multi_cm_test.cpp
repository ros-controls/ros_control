// Author: Kelsey Hawkins
// Based on code by:
///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012, hiDOF INC.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of hiDOF Inc nor the names of its
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

//! /author Vijay Pradeep, Kelsey Hawkins

#include <ros/ros.h>
#include <gtest/gtest.h>

#include <controller_manager_msgs/LoadController.h>

using namespace controller_manager_msgs;

TEST(CMTests, spawnTestGood)
{
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<LoadController>("/controller_manager/load_controller");
  LoadController srv;
  srv.request.name = "multi_controller";
  bool call_success = client.call(srv);
  EXPECT_TRUE(call_success);
  EXPECT_TRUE(srv.response.ok);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ControllerManagerTestNode");

  ros::AsyncSpinner spinner(1);

  // wait for services
  ROS_INFO("Waiting for service");
  ros::service::waitForService("/controller_manager/load_controller");
  ROS_INFO("Start tests");
  spinner.start();

  return RUN_ALL_TESTS();
}
