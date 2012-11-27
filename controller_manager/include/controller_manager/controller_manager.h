///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012, hiDOF, INC & Willow Garage, Inc
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of hiDOF, Inc., Willow Garage, Inc., nor the names of its
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


#ifndef CONTROLLER_MANAGER_CONTROLLER_MANAGER_H
#define CONTROLLER_MANAGER_CONTROLLER_MANAGER_H

#include "controller_manager/controller_spec.h"
#include <pthread.h>
#include <map>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <tinyxml.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_utils/realtime_publisher.h>
#include <ros/node_handle.h>
#include <pluginlib/class_loader.h>
#include <controller_manager_msgs/ListControllerTypes.h>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/ReloadControllerLibraries.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/UnloadController.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ControllersStatistics.h>
#include <boost/thread/condition.hpp>
#include <controller_manager/controller_loader_interface.h>


namespace controller_manager{

class ControllerManager{

public:
  ControllerManager(hardware_interface::RobotHW *robot_hw,
                   const ros::NodeHandle& nh=ros::NodeHandle());
  virtual ~ControllerManager();

  // Real-time functions
  void update(const ros::Time& time, bool reset_controllers=false);

  // Non real-time functions
  bool loadController(const std::string& name);
  bool unloadController(const std::string &name);
  bool switchController(const std::vector<std::string>& start_controllers,
                        const std::vector<std::string>& stop_controllers,
                        const int strictness);

  // controllers_lock_ must be locked before calling
  virtual controller_interface::ControllerBase* getControllerByName(const std::string& name);

  void registerControllerLoader(boost::shared_ptr<ControllerLoaderInterface> controller_loader);

private:
  void getControllerNames(std::vector<std::string> &v);
  void getControllerSchedule(std::vector<size_t> &schedule);

  hardware_interface::RobotHW* robot_hw_;

  ros::NodeHandle controller_node_, cm_node_;

  typedef boost::shared_ptr<ControllerLoaderInterface> LoaderPtr;
  std::list<LoaderPtr> controller_loaders_;

  // for controller switching
  std::vector<controller_interface::ControllerBase*> start_request_, stop_request_;
  bool please_switch_;
  int switch_strictness_;

  // controller lists
  boost::mutex controllers_lock_;
  std::vector<ControllerSpec> controllers_lists_[2];
  int current_controllers_list_, used_by_realtime_;

  // for controller statistics
  Statistics pre_update_stats_;
  Statistics update_stats_;
  Statistics post_update_stats_;

  // for publishing constroller state
  void publishControllerStatistics();
  realtime_utils::RealtimePublisher<controller_manager_msgs::ControllersStatistics> pub_controller_stats_;
  ros::Duration publish_period_controller_stats_;
  ros::Time last_published_controller_stats_;

  // services to work with controllers
  bool listControllerTypesSrv(controller_manager_msgs::ListControllerTypes::Request &req,
                              controller_manager_msgs::ListControllerTypes::Response &resp);
  bool listControllersSrv(controller_manager_msgs::ListControllers::Request &req,
                          controller_manager_msgs::ListControllers::Response &resp);
  bool switchControllerSrv(controller_manager_msgs::SwitchController::Request &req,
                           controller_manager_msgs::SwitchController::Response &resp);
  bool loadControllerSrv(controller_manager_msgs::LoadController::Request &req,
                          controller_manager_msgs::LoadController::Response &resp);
  bool unloadControllerSrv(controller_manager_msgs::UnloadController::Request &req,
                         controller_manager_msgs::UnloadController::Response &resp);
  bool reloadControllerLibrariesSrv(controller_manager_msgs::ReloadControllerLibraries::Request &req,
                                    controller_manager_msgs::ReloadControllerLibraries::Response &resp);
  boost::mutex services_lock_;
  ros::ServiceServer srv_list_controllers_, srv_list_controller_types_, srv_load_controller_;
  ros::ServiceServer srv_unload_controller_, srv_switch_controller_, srv_reload_libraries_;
};

}
#endif
