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

/*
 * Author: Wim Meeussen
 */


#include "joint_state_controller/joint_state_controller.h"



namespace joint_state_controller
{

  bool JointStateController::init(hardware_interface::JointStateInterface* hw, ros::NodeHandle &n)
  {
    // get all joint states from the hardware interface
    const std::vector<std::string>& joint_names = hw->getJointNames();
    for (unsigned i=0; i<joint_names.size(); i++)
      ROS_INFO("Got joint %s", joint_names[i].c_str());


    for (unsigned i=0; i<joint_names.size(); i++)
      joint_state_.push_back(hw->getJointStateHandle(joint_names[i]));

    return true;
  }

  void JointStateController::starting(const ros::Time& time)
  {
    ROS_INFO("Starting JointState Controller");
  }

  void JointStateController::update(const ros::Time& time)
  {
    ROS_INFO("Update JointState Controller");
    for (unsigned i=0; i<joint_state_.size(); i++)
      ROS_INFO("JointState of joint %s: %f  %f   %f",
               joint_state_[i].getName().c_str(), joint_state_[i].getPosition(),
               joint_state_[i].getVelocity(), joint_state_[i].getEffort());
  }

  void JointStateController::stopping(const ros::Time& time)
  {
    ROS_INFO("Stopping JointState Controller");
  }

}


PLUGINLIB_DECLARE_CLASS(joint_state_controller, JointStateController, joint_state_controller::JointStateController, controller_interface::ControllerBase)
