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

#include <std_msgs/Duration.h>
#include <controller_manager_tests/pub_period_controller.h>

using namespace controller_manager_tests;

bool PubPeriodController::init(hardware_interface::JointStateInterface* /*hw*/, ros::NodeHandle& n)
{
  period_pub_ = n.advertise<std_msgs::Duration>("period", 1);
  return true;
}

void PubPeriodController::starting(const ros::Time& /*time*/)
{
  ROS_INFO("Starting JointState Controller");
}

void PubPeriodController::update(const ros::Time& /*time*/, const ros::Duration& period)
{
  std_msgs::Duration msg;
  msg.data = period;
  period_pub_.publish(msg);
}

void PubPeriodController::stopping(const ros::Time& /*time*/)
{
  ROS_INFO("Stopping JointState Controller");
}

PLUGINLIB_EXPORT_CLASS( controller_manager_tests::PubPeriodController, controller_interface::ControllerBase)
