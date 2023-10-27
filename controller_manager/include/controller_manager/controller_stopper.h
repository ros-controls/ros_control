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

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner exner@fzi.de
 * \date    2019-06-12
 *
 */
//----------------------------------------------------------------------
#ifndef CONTROLLER_STOPPER_CONTROLLER_STOPPER_H_INCLUDED
#define CONTROLLER_STOPPER_CONTROLLER_STOPPER_H_INCLUDED

#include <ros/ros.h>
#include <std_msgs/Bool.h>

namespace controller_manager {

class ControllerStopper {
  public:
  ControllerStopper() = delete;
  ControllerStopper(const ros::NodeHandle& nh);
  virtual ~ControllerStopper() = default;

  private:
  void robotRunningCallback(const std_msgs::BoolConstPtr& msg);

  /*!
   * \brief Queries running stoppable controllers.
   *
   * Queries the controller manager for running controllers and compares the result with the
   * consistent_controllers_. The remaining running controllers are stored in stopped_controllers_
   */
  void findStoppableControllers();

  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;
  ros::Subscriber robot_running_sub_;
  ros::ServiceClient controller_manager_srv_;
  ros::ServiceClient controller_list_srv_;

  std::vector<std::string> consistent_controllers_;
  std::vector<std::string> stopped_controllers_;

  bool robot_running_;
};
} // namespace controller_manager
#endif // ifndef CONTROLLER_STOPPER_CONTROLLER_STOPPER_H_INCLUDED
