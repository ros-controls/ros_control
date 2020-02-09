///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2015, Shadow Robot Company Ltd.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Shadow Robot Company Ltd. nor the names of its
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

#include <ros/ros.h>
#include <gtest/gtest.h>

#include <combined_robot_hw/combined_robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>

using combined_robot_hw::CombinedRobotHW;

TEST(CombinedRobotHWTests, combinationOk)
{
  ros::NodeHandle nh;

  CombinedRobotHW robot_hw;
  bool init_success = robot_hw.init(nh, nh);
  ASSERT_TRUE(init_success);

  hardware_interface::JointStateInterface*            js_interface = robot_hw.get<hardware_interface::JointStateInterface>();
  hardware_interface::EffortJointInterface*           ej_interface = robot_hw.get<hardware_interface::EffortJointInterface>();
  hardware_interface::VelocityJointInterface*         vj_interface = robot_hw.get<hardware_interface::VelocityJointInterface>();
  hardware_interface::ForceTorqueSensorInterface*     ft_interface = robot_hw.get<hardware_interface::ForceTorqueSensorInterface>();
  hardware_interface::HardwareInterface*              plain_hw_interface = robot_hw.get<hardware_interface::HardwareInterface>();
  hardware_interface::PositionJointInterface*         pj_interface = robot_hw.get<hardware_interface::PositionJointInterface>();

  ASSERT_TRUE(js_interface != nullptr);
  ASSERT_TRUE(ej_interface != nullptr);
  ASSERT_TRUE(vj_interface != nullptr);
  ASSERT_TRUE(ft_interface != nullptr);
  ASSERT_TRUE(plain_hw_interface != nullptr);

  // Test that no PositionJointInterface was found
  ASSERT_EQ(nullptr, pj_interface);

  // Test some handles from my_robot_hw_1
  hardware_interface::JointStateHandle js_handle = js_interface->getHandle("test_joint1");
  hardware_interface::JointHandle ej_handle = ej_interface->getHandle("test_joint1");
  hardware_interface::JointHandle vj_handle = vj_interface->getHandle("test_joint1");
  ASSERT_FLOAT_EQ(1.0, js_handle.getPosition());
  ASSERT_FLOAT_EQ(0.0, ej_handle.getVelocity());
  ASSERT_FLOAT_EQ(3.0, ej_handle.getCommand());

  // Test some handles from my_robot_hw_3
  js_handle = js_interface->getHandle("right_arm_joint_1");
  ej_handle = ej_interface->getHandle("right_arm_joint_1");
  vj_handle = vj_interface->getHandle("right_arm_joint_1");
  ASSERT_FLOAT_EQ(1.0, js_handle.getPosition());
  ASSERT_FLOAT_EQ(0.0, ej_handle.getVelocity());
  ASSERT_FLOAT_EQ(1.5, ej_handle.getCommand());

  // Test some handles from my_robot_hw_4
  hardware_interface::ForceTorqueSensorHandle ft_handle = ft_interface->getHandle("ft_sensor_1");
  ASSERT_FLOAT_EQ(0.2, ft_handle.getForce()[2]);

  // Test non-existent handle throws exception
  ASSERT_ANY_THROW(ej_interface->getHandle("non_existent_joint"));

  // Test read and write functions
  ros::Duration period(1.0);
  robot_hw.read(ros::Time::now(), period);
  js_handle = js_interface->getHandle("test_joint1");
  ASSERT_FLOAT_EQ(2.7, js_handle.getPosition());
  ASSERT_FLOAT_EQ(1.2, ft_handle.getForce()[2]);

  ej_handle = ej_interface->getHandle("test_joint1");
  ej_handle.setCommand(3.5);
  robot_hw.write(ros::Time::now(), period);
  ej_handle = ej_interface->getHandle("test_joint2");
  ASSERT_FLOAT_EQ(3.5, ej_handle.getCommand());
}

TEST(CombinedRobotHWTests, switchOk)
{
  ros::NodeHandle nh;

  CombinedRobotHW robot_hw;
  bool init_success = robot_hw.init(nh, nh);
  ASSERT_TRUE(init_success);

  // Test empty list (it is expected to work)
  {
    std::list<hardware_interface::ControllerInfo> start_list;
    std::list<hardware_interface::ControllerInfo> stop_list;
    ASSERT_TRUE(robot_hw.prepareSwitch(start_list, stop_list));
    ASSERT_NO_THROW(robot_hw.doSwitch(start_list, stop_list));
  }

  // Test failure
  {
    std::list<hardware_interface::ControllerInfo> start_list;
    std::list<hardware_interface::ControllerInfo> stop_list;
    hardware_interface::ControllerInfo controller_1;
    controller_1.name = "ctrl_1";
    controller_1.type = "some_type";
    hardware_interface::InterfaceResources iface_res_1;
    iface_res_1.hardware_interface = "hardware_interface::ForceTorqueSensorInterface";
    iface_res_1.resources.insert("ft_sensor_1");
    controller_1.claimed_resources.push_back(iface_res_1);
    start_list.push_back(controller_1);
    ASSERT_FALSE(robot_hw.prepareSwitch(start_list, stop_list));
    ASSERT_ANY_THROW(robot_hw.doSwitch(start_list, stop_list));
  }

  // Test existing interfaces and resources
  {
    std::list<hardware_interface::ControllerInfo> start_list;
    std::list<hardware_interface::ControllerInfo> stop_list;
    hardware_interface::ControllerInfo controller_1;
    controller_1.name = "ctrl_1";
    controller_1.type = "some_type";
    hardware_interface::InterfaceResources iface_res_1;
    iface_res_1.hardware_interface = "hardware_interface::EffortJointInterface";
    iface_res_1.resources.insert("test_joint1");
    iface_res_1.resources.insert("test_joint2");
    iface_res_1.resources.insert("test_joint3");
    iface_res_1.resources.insert("test_joint4");
    controller_1.claimed_resources.push_back(iface_res_1);
    hardware_interface::InterfaceResources iface_res_2;
    iface_res_2.hardware_interface = "hardware_interface::VelocityJointInterface";
    iface_res_2.resources.insert("test_joint1");
    iface_res_2.resources.insert("test_joint4");
    controller_1.claimed_resources.push_back(iface_res_1);
    start_list.push_back(controller_1);

    hardware_interface::ControllerInfo controller_2;
    hardware_interface::InterfaceResources iface_res_3;
    iface_res_3.hardware_interface = "hardware_interface::VelocityJointInterface";
    iface_res_3.resources.insert("test_joint3");
    iface_res_3.resources.insert("test_joint5");
    controller_2.claimed_resources.push_back(iface_res_3);
    start_list.push_back(controller_2);
    ASSERT_TRUE(robot_hw.prepareSwitch(start_list, stop_list));
    ASSERT_NO_THROW(robot_hw.doSwitch(start_list, stop_list));
  }

  // Test non-registered interfaces and resources
  // (this should also work, as CombinedRobotHW will filter out the non-registered ones)
  {
    std::list<hardware_interface::ControllerInfo> start_list;
    std::list<hardware_interface::ControllerInfo> stop_list;
    hardware_interface::ControllerInfo controller_1;
    controller_1.name = "ctrl_1";
    controller_1.type = "some_type";
    hardware_interface::InterfaceResources iface_res_1;
    iface_res_1.hardware_interface = "hardware_interface::NonRegisteredInterface";
    iface_res_1.resources.insert("test_joint1");
    iface_res_1.resources.insert("test_joint2");
    iface_res_1.resources.insert("test_joint3");
    iface_res_1.resources.insert("test_joint4");
    controller_1.claimed_resources.push_back(iface_res_1);
    hardware_interface::InterfaceResources iface_res_2;
    iface_res_2.hardware_interface = "hardware_interface::VelocityJointInterface";
    iface_res_2.resources.insert("test_joint1");
    iface_res_2.resources.insert("non_registered_joint1");
    controller_1.claimed_resources.push_back(iface_res_1);
    start_list.push_back(controller_1);

    hardware_interface::ControllerInfo controller_2;
    hardware_interface::InterfaceResources iface_res_3;
    iface_res_3.hardware_interface = "hardware_interface::VelocityJointInterface";
    iface_res_3.resources.insert("test_joint3");
    iface_res_3.resources.insert("non_registered_joint2");
    controller_2.claimed_resources.push_back(iface_res_3);
    start_list.push_back(controller_2);
    ASSERT_TRUE(robot_hw.prepareSwitch(start_list, stop_list));
    ASSERT_NO_THROW(robot_hw.doSwitch(start_list, stop_list));
  }

  // Test resource and controller filtering
  {
    std::list<hardware_interface::ControllerInfo> start_list;
    std::list<hardware_interface::ControllerInfo> stop_list;
    hardware_interface::ControllerInfo controller_1;
    controller_1.name = "ctrl_without_my_robot_hw_2_resources_in_one_of_two_ifaces";
    controller_1.type = "some_type";
    hardware_interface::InterfaceResources iface_res_1;
    // iface_res_1 should be filtered out when controller_1 is passed to my_robot_hw_2
    // as none of its resources belongs to my_robot_hw_2
    iface_res_1.hardware_interface = "hardware_interface::EffortJointInterface";
    iface_res_1.resources.insert("test_joint1");
    iface_res_1.resources.insert("test_joint2");
    iface_res_1.resources.insert("test_joint3");
    controller_1.claimed_resources.push_back(iface_res_1);
    hardware_interface::InterfaceResources iface_res_2;
    iface_res_2.hardware_interface = "hardware_interface::VelocityJointInterface";
    iface_res_2.resources.insert("test_joint1");
    iface_res_2.resources.insert("test_joint4");
    controller_1.claimed_resources.push_back(iface_res_1);
    start_list.push_back(controller_1);

    hardware_interface::ControllerInfo controller_2;
    // controller_2 should be filtered out when controller_2 is passed to my_robot_hw_2
    // as none of its resources belongs to my_robot_hw_2
    controller_2.name = "ctrl_without_my_robot_hw_2_resources";
    controller_2.type = "some_type";
    hardware_interface::InterfaceResources iface_res_3;
    iface_res_3.hardware_interface = "hardware_interface::VelocityJointInterface";
    iface_res_3.resources.insert("test_joint1");
    iface_res_3.resources.insert("test_joint2");
    controller_2.claimed_resources.push_back(iface_res_3);
    start_list.push_back(controller_2);
    ASSERT_TRUE(robot_hw.prepareSwitch(start_list, stop_list));
    ASSERT_NO_THROW(robot_hw.doSwitch(start_list, stop_list));
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "CombinedRobotHWTestNode");

  int ret = RUN_ALL_TESTS();

  ros::shutdown();
  return ret;
}
