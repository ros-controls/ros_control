///////////////////////////////////////////////////////////////////////////////
// Copyright 2019 FZI Forschungszentrum Informatik
// Created on behalf of Universal Robots A/S
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Willow Garage Inc, hiDOF Inc, nor the names of its
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
// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner exner@fzi.de
 * \date    2019-06-12
 *
 */
//----------------------------------------------------------------------

#include <controller_manager/controller_stopper.h>

#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ListControllers.h>

#include <ios>

namespace controller_manager{

ControllerStopper::ControllerStopper(const ros::NodeHandle& nh) : nh_(nh), priv_nh_("~"), robot_running_(true)
{
  // Subscribes to a robot's running state topic. Ideally this topic is latched and only publishes
  // on changes. However, this node only reacts on state changes, so a state published each cycle
  // would also be fine.
  robot_running_sub_ = nh_.subscribe("robot_running", 1, &ControllerStopper::robotRunningCallback, this);

  // Controller manager service to switch controllers
  controller_manager_srv_ = nh_.serviceClient<controller_manager_msgs::SwitchController>("controller_manager/"
                                                                                         "switch_controller");
  // Controller manager service to list controllers
  controller_list_srv_ = nh_.serviceClient<controller_manager_msgs::ListControllers>("controller_manager/"
                                                                                     "list_controllers");
  ROS_INFO_STREAM("Waiting for controller manager service to come up on " << nh_.resolveName("controller_manager/"
                                                                                             "switch_controller"));
  controller_manager_srv_.waitForExistence();
  ROS_INFO_STREAM("Service available.");
  ROS_INFO_STREAM("Waiting for controller list service to come up on " << nh_.resolveName("controller_manager/"
                                                                                          "list_controllers"));
  controller_list_srv_.waitForExistence();
  ROS_INFO_STREAM("Service available.");

  // Consistent controllers will not be stopped when the robot stops. Defaults to
  // ["joint_state_controller"]
  if (!priv_nh_.getParam("consistent_controllers", consistent_controllers_))
  {
    consistent_controllers_.push_back("joint_state_controller");
  }

  ROS_DEBUG("Waiting for running controllers");
  // Before we can work properly, we need to know which controllers there are
  while (stopped_controllers_.empty())
  {
    findStoppableControllers();
    ros::Duration(1).sleep();
  }
  ROS_DEBUG("Initialization finished");
}

void ControllerStopper::findStoppableControllers()
{
  controller_manager_msgs::ListControllers list_srv;
  controller_list_srv_.call(list_srv);
  stopped_controllers_.clear();
  for (auto& controller : list_srv.response.controller)
  {
    // Check if in consistent_controllers
    // Else:
    //   Add to stopped_controllers
    if (controller.state == "running")
    {
      auto it = std::find(consistent_controllers_.begin(), consistent_controllers_.end(), controller.name);
      if (it == consistent_controllers_.end())
      {
        stopped_controllers_.push_back(controller.name);
      }
    }
  }
}

void ControllerStopper::robotRunningCallback(const std_msgs::BoolConstPtr& msg)
{
  ROS_DEBUG_STREAM("robotRunningCallback with data " << std::boolalpha << msg->data);
  if (msg->data && !robot_running_)
  {
    ROS_DEBUG_STREAM("Starting controllers");
    controller_manager_msgs::SwitchController srv;
    srv.request.strictness = srv.request.STRICT;
    srv.request.start_controllers = stopped_controllers_;
    if (!controller_manager_srv_.call(srv))
    {
      ROS_ERROR_STREAM("Could not activate requested controllers");
    }
  }
  else if (!msg->data && robot_running_)
  {
    ROS_DEBUG_STREAM("Stopping controllers");
    //   stop all controllers except the once in consistent_controllers_
    findStoppableControllers();
    controller_manager_msgs::SwitchController srv;
    srv.request.strictness = srv.request.STRICT;
    srv.request.stop_controllers = stopped_controllers_;
    if (!controller_manager_srv_.call(srv))
    {
      ROS_ERROR_STREAM("Could not stop requested controllers");
    }
  }
  robot_running_ = msg->data;
}
} // namespace controller_manager
