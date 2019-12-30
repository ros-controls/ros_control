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

#pragma once


#include "controller_manager/controller_spec.h"
#include <cstdio>
#include <map>
#include <mutex>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <pluginlib/class_loader.hpp>
#include <controller_manager_msgs/ListControllerTypes.h>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/ReloadControllerLibraries.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/UnloadController.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager/controller_loader_interface.h>


namespace controller_manager{

/** \brief ROS Controller Manager and Runner
 *
 * This class advertises a ROS interface for loading, unloading, starting, and
 * stopping ros_control-based controllers. It also serializes execution of all
 * running controllers in \ref update.
 *
 */

class ControllerManager{

public:
  static constexpr bool WAIT_FOR_ALL_RESOURCES = false;
  static constexpr double INFINITE_TIMEOUT = 0.0;

  /** \brief Constructor
   *
   * \param robot_hw A pointer to a robot hardware interface
   * \param nh The ros::NodeHandle in whose namespace all ROS interfaces should
   * operate.
   */
  ControllerManager(hardware_interface::RobotHW *robot_hw,
                   const ros::NodeHandle& nh=ros::NodeHandle());
  virtual ~ControllerManager() = default;

  /** \name Real-Time Safe Functions
   *\{*/
  /** \brief Update all active controllers.
   *
   * When controllers are started or stopped (or switched), those calls are
   * made in this function.
   *
   * \param time The current time
   * \param period The change in time since the last call to \ref update
   * \param reset_controllers If \c true, stop and start all running
   * controllers before updating
   */
  void update(const ros::Time& time, const ros::Duration& period, bool reset_controllers=false);
  /*\}*/

  /** \name Non Real-Time Safe Functions
   *\{*/

  /** \brief Load a new controller by name.
   *
   * This dynamically loads a controller called \c name and initializes the
   * newly
   * loaded controller.
   *
   * It determines the controller type by accessing the ROS parameter "type" in
   * the namespace given by \c name relative to the namespace of \ref
   * root_nh_. It then initializes the controller with the
   * hardware_interface::RobotHW pointer \ref robot_hw_, the ros::NodeHandle
   * describing this namespace, and a reference to a std::set to retrieve the
   * resources needed by this controller.
   *
   * A controller cannot be loaded while already loaded. To re-load a
   * controller, first \ref unloadController and then \ref loadController.
   *
   * \param name The name of the controller as well as the ROS namespace under
   * which the controller should be loaded
   *
   * \returns True on success
   * \returns False on failure
   */
  bool loadController(const std::string& name);

  /** \brief Unload a controller by name
   *
   * \param name The name of the controller to unload. (The same as the one used in \ref loadController )
   *
   */
  bool unloadController(const std::string &name);

  /** \brief Switch multiple controllers simultaneously.
   *
   * \param start_controllers A vector of controller names to be started
   * \param stop_controllers A vector of controller names to be stopped
   * \param strictness How important it is that the requested controllers are
   * started and stopped.  The levels are defined in the
   * controller_manager_msgs/SwitchControllers service as either \c BEST_EFFORT
   * or \c STRICT.  \c BEST_EFFORT means that \ref switchController can still
   * succeed if a non-existent controller is requested to be stopped or started.
   * \param start_asap Start the controllers as soon as their resources
   * are ready, will wait for all resources to be ready otherwise.
   * \param timeout The timeout in seconds before aborting pending
   * controllers. Zero for infinite.
   */
  bool switchController(const std::vector<std::string>& start_controllers,
                        const std::vector<std::string>& stop_controllers,
                        const int strictness, bool start_asap = WAIT_FOR_ALL_RESOURCES,
                        double timeout = INFINITE_TIMEOUT);

  /** \brief Get a controller by name.
   *
   * \param name The name of a controller
   * \returns An up-casted pointer to the controller identified by \c name
   */
  virtual controller_interface::ControllerBase* getControllerByName(const std::string& name);

  /** \brief Register a controller loader.
   *
   * By default, the pluginlib-based \ref ControllerLoader is registered on
   * construction of this class. To load controllers through alternate means,
   * register alternate controller loaders here. Note, however, that when
   * controllers are loaded by \ref loadController the controller loaders are
   * queried in the order that they were registered. This means that if a
   * controller CAN be loaded by the pluginlib-based \ref ControllerLoader,
   * then it WILL, regardless of which other loaders are registered.
   *
   * \param controller_loader A pointer to the loader to be registered
   *
   */
  void registerControllerLoader(ControllerLoaderInterfaceSharedPtr controller_loader);
  /*\}*/


private:
  void getControllerNames(std::vector<std::string> &v);

  void manageSwitch(const ros::Time& time);
  void stopControllers(const ros::Time& time);
  void startControllers(const ros::Time& time);
  void startControllersAsap(const ros::Time& time);

  hardware_interface::RobotHW* robot_hw_;

  ros::NodeHandle root_nh_, cm_node_;

  std::list<ControllerLoaderInterfaceSharedPtr> controller_loaders_;

  /** \name Controller Switching
   *\{*/
  std::vector<controller_interface::ControllerBase*> start_request_, stop_request_;
  std::list<hardware_interface::ControllerInfo> switch_start_list_, switch_stop_list_;

  struct SwitchParams
  {
    bool do_switch      = {false};
    bool started        = {false};
    ros::Time init_time = {ros::TIME_MAX};

    // Switch options
    int strictness  = {0};
    bool start_asap = {false};
    double timeout  = {0.0};
  };

  SwitchParams switch_params_;

  /*\}*/

  /** \name Controllers List
   * The controllers list is double-buffered to avoid needing to lock the
   * real-time thread when switching controllers in the non-real-time thread.
   *\{*/
  /// Mutex protecting the current controllers list
  std::recursive_mutex controllers_lock_;
  /// Double-buffered controllers list
  std::vector<ControllerSpec> controllers_lists_[2];
  /// The index of the current controllers list
  int current_controllers_list_ = {0};
  /// The index of the controllers list being used in the real-time thread.
  int used_by_realtime_ = {-1};
  /*\}*/


  /** \name ROS Service API
   *\{*/
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
  std::mutex services_lock_;
  ros::ServiceServer srv_list_controllers_, srv_list_controller_types_, srv_load_controller_;
  ros::ServiceServer srv_unload_controller_, srv_switch_controller_, srv_reload_libraries_;
  /*\}*/
};

}
