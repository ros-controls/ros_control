/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc. & hiDOF, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage, hiDOF, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <effort_controllers/joint_position_controller.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(effort_controllers, JointPositionController, effort_controllers::JointPositionController, controller_interface::ControllerBase)

using namespace std;

namespace effort_controllers
{

JointPositionController::JointPositionController()
: joint_command_handle_(), command_(0),
  loop_count_(0),  initialized_(false), robot_(NULL), last_time_(0)
{
}

JointPositionController::~JointPositionController()
{
  sub_command_.shutdown();
}

bool JointPositionController::init(hardware_interface::JointEffortCommandInterface *robot, const std::string &joint_name,
				   const control_toolbox::Pid &pid)
{
  assert(robot);

  try
  {
    joint_command_handle_ = robot->getJointEffortCommand(joint_name);
  }
  catch(std::exception &e)
  {
    ROS_ERROR("JointPosition control ")
  }

  if (!joint_state_)
  {
    ROS_ERROR("JointPositionController could not find joint named \"%s\"\n",
              joint_name.c_str());
    return false;
  }
  if (!joint_state_->calibrated_)
  {
    ROS_ERROR("Joint %s not calibrated for JointPositionController", joint_name.c_str());
    return false;
  }

  pid_controller_ = pid;

  return true;
}

bool JointPositionController::init(hardware_interface::JointEffortCommandInterface *robot, ros::NodeHandle &n)
{
  assert(robot);
  node_ = n;

  std::string joint_name;
  if (!node_.getParam("joint", joint_name)) {
    ROS_ERROR("No joint given (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }

  control_toolbox::Pid pid;
  if (!pid.init(ros::NodeHandle(node_, "pid")))
    return false;

  controller_state_publisher_.reset(
    new realtime_tools::RealtimePublisher<pr2_controllers_msgs::JointControllerState>
    (node_, "state", 1));

  sub_command_ = node_.subscribe<std_msgs::Float64>("command", 1, &JointPositionController::setCommandCB, this);

  return init(robot, joint_name, pid);
}


void JointPositionController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min)
{
  pid_controller_.setGains(p,i,d,i_max,i_min);
}

void JointPositionController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
{
  pid_controller_.getGains(p,i,d,i_max,i_min);
}

std::string JointPositionController::getJointName()
{
  return joint_state_->joint_->name;
}

// Set the joint position command
void JointPositionController::setCommand(double cmd)
{
  command_ = cmd;
}

// Return the current position command
void JointPositionController::getCommand(double & cmd)
{
  cmd = command_;
}

void JointPositionController::starting(const ros::Time& time)
{
  //  last_time_ = time;
}

void JointPositionController::update(const ros::Time& time)
{
  if (!joint_state_->calibrated_)
    return;

  assert(robot_ != NULL);
  double error(0);
  assert(joint_state_->joint_);
  ros::Duration dt = time - last_time_;

  if (!initialized_)
  {
    initialized_ = true;
    command_ = joint_state_->position_;
    dt = ros::Duration(0,0);
  }

  if(joint_state_->joint_->type == urdf::Joint::REVOLUTE)
  {
    angles::shortest_angular_distance_with_limits(command_, joint_state_->position_, joint_state_->joint_->limits->lower, joint_state_->joint_->limits->upper,error);

  }
  else if(joint_state_->joint_->type == urdf::Joint::CONTINUOUS)
  {
    error = angles::shortest_angular_distance(command_, joint_state_->position_);
  }
  else //prismatic
  {
    error = joint_state_->position_ - command_;
  }

  //double commanded_effort = pid_controller_.updatePid(error, dt_);
  double commanded_effort = pid_controller_.updatePid(error, joint_state_->velocity_, dt_); // assuming desired velocity is 0
  joint_state_->commanded_effort_ = commanded_effort;

  if(loop_count_ % 10 == 0)
  {
    if(controller_state_publisher_ && controller_state_publisher_->trylock())
    {
      controller_state_publisher_->msg_.header.stamp = time;
      controller_state_publisher_->msg_.set_point = command_;
      controller_state_publisher_->msg_.process_value = joint_state_->position_;
      controller_state_publisher_->msg_.process_value_dot = joint_state_->velocity_;
      controller_state_publisher_->msg_.error = error;
      controller_state_publisher_->msg_.time_step = dt_.toSec();
      controller_state_publisher_->msg_.command = commanded_effort;

      double dummy;
      getGains(controller_state_publisher_->msg_.p,
               controller_state_publisher_->msg_.i,
               controller_state_publisher_->msg_.d,
               controller_state_publisher_->msg_.i_clamp,
               dummy);
      controller_state_publisher_->unlockAndPublish();
    }
  }
  loop_count_++;

  last_time_ = time;
}

void JointPositionController::setCommandCB(const std_msgs::Float64ConstPtr& msg)
{
  command_ = msg->data;
}

}
