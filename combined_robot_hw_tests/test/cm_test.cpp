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

/// \author Adolfo Rodríguez Tsouroukdissian
/// \author Vijay Pradeep
/// \author Toni Oliver

#include <ros/ros.h>
#include <gtest/gtest.h>

#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/ListControllerTypes.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/UnloadController.h>


using namespace controller_manager_msgs;

TEST(CMTests, loadUnloadOk)
{
  ros::NodeHandle nh;
  ros::ServiceClient load_client   = nh.serviceClient<LoadController>("/controller_manager/load_controller");
  ros::ServiceClient unload_client = nh.serviceClient<UnloadController>("/controller_manager/unload_controller");

  // Load single-interface controller
  {
    LoadController srv;
    srv.request.name = "my_controller";
    bool call_success = load_client.call(srv);
    ASSERT_TRUE(call_success);
    EXPECT_TRUE(srv.response.ok);
  }

  // Load multi-interface controller:
  // Two required interfaces
  {
    LoadController srv;
    srv.request.name = "vel_eff_controller";
    bool call_success = load_client.call(srv);
    ASSERT_TRUE(call_success);
    EXPECT_TRUE(srv.response.ok);
  }

  // Load multi-interface controller:
  // One required (and existing) interface and one optional (and non-existent) interface
  {
    LoadController srv;
    srv.request.name = "optional_interfaces_controller";
    bool call_success = load_client.call(srv);
    ASSERT_TRUE(call_success);
    EXPECT_TRUE(srv.response.ok);
  }

  // Unload single-interface controller
  {
    UnloadController srv;
    srv.request.name = "my_controller";
    bool call_success = unload_client.call(srv);
    ASSERT_TRUE(call_success);
    EXPECT_TRUE(srv.response.ok);
  }

  // Unload multi-interface controllers
  {
    UnloadController srv;
    srv.request.name = "vel_eff_controller";
    bool call_success = unload_client.call(srv);
    ASSERT_TRUE(call_success);
    EXPECT_TRUE(srv.response.ok);
  }
  {
    UnloadController srv;
    srv.request.name = "optional_interfaces_controller";
    bool call_success = unload_client.call(srv);
    ASSERT_TRUE(call_success);
    EXPECT_TRUE(srv.response.ok);
  }
}

TEST(CMTests, loadUnloadKo)
{
  ros::NodeHandle nh;
  ros::ServiceClient load_client   = nh.serviceClient<LoadController>("/controller_manager/load_controller");
  ros::ServiceClient unload_client = nh.serviceClient<UnloadController>("/controller_manager/unload_controller");

  // Load non-existent controller
  {
    LoadController srv;
    srv.request.name = "nonexistent_controller";
    bool call_success = load_client.call(srv);
    ASSERT_TRUE(call_success);
    EXPECT_FALSE(srv.response.ok);
  }

  // Load controller requesting non-existient HW interface
  {
    LoadController srv;
    srv.request.name = "non_existent_interface_controller";
    bool call_success = load_client.call(srv);
    ASSERT_TRUE(call_success);
    EXPECT_FALSE(srv.response.ok);
  }

  // Load controller requesting non-existent resource from valid HW interface
  {
    LoadController srv;
    srv.request.name = "non_existent_resource_controller";
    bool call_success = load_client.call(srv);
    ASSERT_TRUE(call_success);
    EXPECT_FALSE(srv.response.ok);
  }

  // Load multi-interface controller:
  // Two required HW interfaces, of which one is non-existent
  {
    LoadController srv;
    srv.request.name = "non_existing_multi_interface_controller";
    bool call_success = load_client.call(srv);
    ASSERT_TRUE(call_success);
    EXPECT_FALSE(srv.response.ok);
  }

  // Unload not loaded controller
  {
    UnloadController srv;
    srv.request.name = "my_controller";
    bool call_success = unload_client.call(srv);
    ASSERT_TRUE(call_success);
    EXPECT_FALSE(srv.response.ok);
  }
}

TEST(CMTests, switchController)
{
  ros::NodeHandle nh;
  ros::ServiceClient load_client   = nh.serviceClient<LoadController>("/controller_manager/load_controller");
  ros::ServiceClient unload_client = nh.serviceClient<UnloadController>("/controller_manager/unload_controller");
  ros::ServiceClient switch_client = nh.serviceClient<SwitchController>("/controller_manager/switch_controller");

  // Load controllers
  {
    LoadController srv;
    srv.request.name = "my_controller";
    load_client.call(srv);
    srv.request.name = "my_controller2";
    load_client.call(srv);
    srv.request.name = "vel_eff_controller";
    load_client.call(srv);
    srv.request.name = "self_conflict_controller";
    load_client.call(srv);
  }

  // Successful STRICT start
  {
    SwitchController srv;
    srv.request.start_controllers.push_back("my_controller");
    srv.request.strictness = srv.request.STRICT;
    bool call_success = switch_client.call(srv);
    ASSERT_TRUE(call_success);
    EXPECT_TRUE(srv.response.ok);
  }

  // Successful STRICT stop
  {
    SwitchController srv;
    srv.request.stop_controllers.push_back("my_controller");
    srv.request.strictness = srv.request.STRICT;
    bool call_success = switch_client.call(srv);
    ASSERT_TRUE(call_success);
    EXPECT_TRUE(srv.response.ok);
  }

  // Successful STRICT start
  {
    SwitchController srv;
    srv.request.start_controllers.push_back("vel_eff_controller");
    srv.request.strictness = srv.request.STRICT;
    bool call_success = switch_client.call(srv);
    ASSERT_TRUE(call_success);
    EXPECT_TRUE(srv.response.ok);
  }

  // Successful STRICT stop+start
  {
    SwitchController srv;
    srv.request.start_controllers.push_back("my_controller");
    srv.request.stop_controllers.push_back("vel_eff_controller");
    srv.request.strictness = srv.request.STRICT;
    bool call_success = switch_client.call(srv);
    ASSERT_TRUE(call_success);
    EXPECT_TRUE(srv.response.ok);
  }

  // Back to no running controllers
  {
    SwitchController srv;
    srv.request.stop_controllers.push_back("my_controller");
    srv.request.strictness = srv.request.STRICT;
    bool call_success = switch_client.call(srv);
    ASSERT_TRUE(call_success);
    EXPECT_TRUE(srv.response.ok);
  }

  // Unsuccessful STRICT start
  {
    SwitchController srv;
    srv.request.start_controllers.push_back("non_existent_controller");
    srv.request.strictness = srv.request.STRICT;
    bool call_success = switch_client.call(srv);
    ASSERT_TRUE(call_success);
    EXPECT_FALSE(srv.response.ok);
  }

  // Unsuccessful STRICT stop
  {
    SwitchController srv;
    srv.request.stop_controllers.push_back("non_existent_controller");
    srv.request.strictness = srv.request.STRICT;
    bool call_success = switch_client.call(srv);
    ASSERT_TRUE(call_success);
    EXPECT_FALSE(srv.response.ok);
  }

  // Unsuccessful STRICT switch: invalid stop
  {
    SwitchController srv;
    srv.request.start_controllers.push_back("my_controller");
    srv.request.stop_controllers.push_back("non_existent_controller");
    srv.request.strictness = srv.request.STRICT;
    bool call_success = switch_client.call(srv);
    ASSERT_TRUE(call_success);
    EXPECT_FALSE(srv.response.ok);
  }

  // Unsuccessful STRICT switch: invalid start
  {
    SwitchController srv;
    srv.request.start_controllers.push_back("non_existent_controller");
    srv.request.stop_controllers.push_back("my_controller");
    srv.request.strictness = srv.request.STRICT;
    bool call_success = switch_client.call(srv);
    ASSERT_TRUE(call_success);
    EXPECT_FALSE(srv.response.ok);
  }

  // Unsuccessful STRICT start: Resource conflict within single controller
  {
    SwitchController srv;
    srv.request.start_controllers.push_back("self_conflict_controller");
    srv.request.strictness = srv.request.STRICT;
    bool call_success = switch_client.call(srv);
    ASSERT_TRUE(call_success);
    EXPECT_FALSE(srv.response.ok);
  }

  // Unsuccessful STRICT start: Resource conflict between two controllers
  {
    SwitchController srv;
    srv.request.start_controllers.push_back("my_controller");
    srv.request.start_controllers.push_back("my_controller2");
    srv.request.strictness = srv.request.STRICT;
    bool call_success = switch_client.call(srv);
    ASSERT_TRUE(call_success);
    EXPECT_FALSE(srv.response.ok);
  }

  // Successful BEST_EFFORT switch: No-op
  {
      SwitchController srv;
      srv.request.start_controllers.push_back("non_existent_controller");
      srv.request.stop_controllers.push_back("non_existent_controller");
      srv.request.strictness = srv.request.BEST_EFFORT;
      bool call_success = switch_client.call(srv);
      ASSERT_TRUE(call_success);
      EXPECT_TRUE(srv.response.ok);
  }

  // Successful BEST_EFFORT switch: Partial success, only one started controller
  {
      SwitchController srv;
      srv.request.start_controllers.push_back("my_controller2");
      srv.request.start_controllers.push_back("non_existent_controller");
      srv.request.stop_controllers.push_back("non_existent_controller");
      srv.request.stop_controllers.push_back("my_controller");
      srv.request.strictness = srv.request.BEST_EFFORT;
      bool call_success = switch_client.call(srv);
      ASSERT_TRUE(call_success);
      EXPECT_TRUE(srv.response.ok);
  }

  // Successful BEST_EFFORT switch: Partial success, one started and one stopped controller
  {
      SwitchController srv;
      srv.request.start_controllers.push_back("my_controller");
      srv.request.start_controllers.push_back("non_existent_controller");
      srv.request.stop_controllers.push_back("non_existent_controller");
      srv.request.stop_controllers.push_back("my_controller2");
      srv.request.strictness = srv.request.BEST_EFFORT;
      bool call_success = switch_client.call(srv);
      ASSERT_TRUE(call_success);
      EXPECT_TRUE(srv.response.ok);
  }

  // Back to no running controllers
  {
    SwitchController srv;
    srv.request.stop_controllers.push_back("my_controller");
    srv.request.strictness = srv.request.STRICT;
    bool call_success = switch_client.call(srv);
    ASSERT_TRUE(call_success);
    EXPECT_TRUE(srv.response.ok);
  }

  // Unload controllers
  {
    UnloadController srv;
    srv.request.name = "my_controller";
    unload_client.call(srv);
    srv.request.name = "my_controller2";
    unload_client.call(srv);
    srv.request.name = "vel_eff_controller";
    unload_client.call(srv);
    srv.request.name = "self_conflict_controller";
    unload_client.call(srv);
  }
}

TEST(CMTests, stopBeforeUnload)
{
  ros::NodeHandle nh;
  ros::ServiceClient load_client   = nh.serviceClient<LoadController>("/controller_manager/load_controller");
  ros::ServiceClient unload_client = nh.serviceClient<UnloadController>("/controller_manager/unload_controller");
  ros::ServiceClient switch_client = nh.serviceClient<SwitchController>("/controller_manager/switch_controller");

  // Load controller
  {
    LoadController srv;
    srv.request.name = "my_controller";
    bool call_success = load_client.call(srv);
    ASSERT_TRUE(call_success);
    EXPECT_TRUE(srv.response.ok);
  }

  // Start controller
  {
    SwitchController srv;
    srv.request.start_controllers.push_back("my_controller");
    srv.request.strictness = srv.request.STRICT;
    bool call_success = switch_client.call(srv);
    ASSERT_TRUE(call_success);
    EXPECT_TRUE(srv.response.ok);
  }

  // Try to unload running controller and fail
  {
    UnloadController srv;
    srv.request.name = "my_controller";
    bool call_success = unload_client.call(srv);
    ASSERT_TRUE(call_success);
    EXPECT_FALSE(srv.response.ok);
  }

  // Stop controller
  {
    SwitchController srv;
    srv.request.stop_controllers.push_back("my_controller");
    srv.request.strictness = srv.request.STRICT;
    bool call_success = switch_client.call(srv);
    ASSERT_TRUE(call_success);
    EXPECT_TRUE(srv.response.ok);
  }

  // Unload stopped controller
  {
    UnloadController srv;
    srv.request.name = "my_controller";
    bool call_success = unload_client.call(srv);
    ASSERT_TRUE(call_success);
    EXPECT_TRUE(srv.response.ok);
  }
}

TEST(CMTests, listControllerTypes)
{
  ros::NodeHandle nh;
  ros::ServiceClient types_client = nh.serviceClient<ListControllerTypes>("/controller_manager/list_controller_types");

  ListControllerTypes srv;
  bool call_success = types_client.call(srv);
  ASSERT_TRUE(call_success);
  // Weak test that the number of available types and base classes is not lower than those defined in this test package
  EXPECT_GE(srv.response.types.size(), 3);
  EXPECT_GE(srv.response.base_classes.size(), 3);
}

TEST(CMTests, listControllers)
{
  ros::NodeHandle nh;
  ros::ServiceClient load_client   = nh.serviceClient<LoadController>("/controller_manager/load_controller");
  ros::ServiceClient unload_client = nh.serviceClient<UnloadController>("/controller_manager/unload_controller");
  ros::ServiceClient switch_client = nh.serviceClient<SwitchController>("/controller_manager/switch_controller");
  ros::ServiceClient list_client   = nh.serviceClient<ListControllers>("/controller_manager/list_controllers");

  // Load controllers
  {
    LoadController srv;
    srv.request.name = "my_controller";
    load_client.call(srv);
    srv.request.name = "vel_eff_controller";
    load_client.call(srv);
  }

  // Start one controller
  {
    SwitchController srv;
    srv.request.start_controllers.push_back("my_controller");
    srv.request.strictness = srv.request.STRICT;
    switch_client.call(srv);
  }

  // List controllers
  {
    ListControllers srv;
    bool call_success = list_client.call(srv);
    ASSERT_TRUE(call_success);
    ASSERT_EQ(srv.response.controller.size(), 2);

    ControllerState state1, state2;
    if (srv.response.controller[0].name == "my_controller")
    {
      state1 = srv.response.controller[0];
      state2 = srv.response.controller[1];
    }
    else
    {
      state1 = srv.response.controller[1];
      state2 = srv.response.controller[0];
    }

    EXPECT_EQ(state1.name, "my_controller");
    EXPECT_EQ(state1.state, "running");
    EXPECT_EQ(state1.type, "controller_manager_tests/EffortTestController");
    ASSERT_EQ(state1.claimed_resources.size(), 1);
    EXPECT_EQ(state1.claimed_resources[0].hardware_interface, "hardware_interface::EffortJointInterface");
    ASSERT_EQ(state1.claimed_resources[0].resources.size(), 2);
    EXPECT_EQ(state1.claimed_resources[0].resources[0], "hiDOF_joint1");
    EXPECT_EQ(state1.claimed_resources[0].resources[1], "hiDOF_joint2");

    EXPECT_EQ(state2.name, "vel_eff_controller");
    EXPECT_EQ(state2.state, "initialized");
    EXPECT_EQ(state2.type, "controller_manager_tests/VelEffController");
    EXPECT_EQ(state2.claimed_resources.size(), 2);
    EXPECT_EQ(state2.claimed_resources[0].hardware_interface, "hardware_interface::VelocityJointInterface");
    ASSERT_EQ(state2.claimed_resources[0].resources.size(), 2);
    EXPECT_EQ(state2.claimed_resources[0].resources[0], "test_joint1");
    EXPECT_EQ(state2.claimed_resources[0].resources[1], "test_joint2");
    EXPECT_EQ(state2.claimed_resources[1].hardware_interface, "hardware_interface::EffortJointInterface");
    ASSERT_EQ(state2.claimed_resources[1].resources.size(), 1);
    EXPECT_EQ(state2.claimed_resources[1].resources[0], "test_joint4");
  }

  // Stop running controller
  {
    SwitchController srv;
    srv.request.stop_controllers.push_back("my_controller");
    srv.request.strictness = srv.request.STRICT;
    switch_client.call(srv);
  }

  // Unload controllers
  {
    UnloadController srv;
    srv.request.name = "my_controller";
    unload_client.call(srv);
    srv.request.name = "vel_eff_controller";
    unload_client.call(srv);
  }
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
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
