///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2017, Clearpath Robotics Inc.
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

#pragma once


#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>

namespace controller_manager_tests
{

/**
 * This controller supplies an intentional extension point in the form of the virtual
 * "helper" function that the update method calls.
 */
typedef controller_interface::MultiInterfaceController<hardware_interface::VelocityJointInterface> BaseControllerInterface;
class ExtensibleController : public virtual BaseControllerInterface
{
public:
  bool init(hardware_interface::RobotHW* robot_hw,
            ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  virtual int helper();
  void update(const ros::Time&, const ros::Duration&) override;
};

/**
 * The derived controller not only overrides the virtual helper method, it also adds an additional
 * hardware interface, in order to demonstrate the flexbility of this mechanism.
 */
typedef controller_interface::MultiInterfaceController<
    hardware_interface::VelocityJointInterface, hardware_interface::EffortJointInterface> DerivedControllerInterface;
class DerivedController : public ExtensibleController, public DerivedControllerInterface
{
public:
  bool initRequest(hardware_interface::RobotHW* hw, ros::NodeHandle& nh, ros::NodeHandle& pnh,
      controller_interface::ControllerBase::ClaimedResources& cr) override;
  bool init(hardware_interface::RobotHW* robot_hw,
            ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  int helper() override;
};

}  // namespace controller_manager_tests
