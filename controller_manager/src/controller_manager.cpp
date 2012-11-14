////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012, hiDOF, INC and Willow Garage, Inc
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the names of Willow Garage, Inc., hiDOF Inc, nor the names of its
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

#include "controller_manager/controller_manager.h"
#include <algorithm>
#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>
#include <sstream>
#include <ros/console.h>


namespace controller_manager{


ControllerManager::ControllerManager(hardware_interface::HardwareInterface *hw, const ros::NodeHandle& nh) :
  hw_(hw),
  controller_node_(nh),
  cm_node_(nh, "controller_manager"),
  start_request_(0),
  stop_request_(0),
  please_switch_(false),
  current_controllers_list_(0),
  used_by_realtime_(-1),
  pub_controller_stats_(nh, "controller_statistics", 1),
  last_published_controller_stats_(ros::Time::now())
{
  printf("  *** ControllerManager(): A ***\n");

  // pre-allocate for realtime publishing
  pub_controller_stats_.msg_.controller.resize(0);

  printf("  *** ControllerManager(): B ***\n");

  // get the publish rate for controller state
  double publish_rate_controller_stats;
  cm_node_.param("controller_statistics_publish_rate", publish_rate_controller_stats, 1.0);

  printf("  *** ControllerManager(): C ***\n");

  publish_period_controller_stats_ = ros::Duration(1.0/fmax(0.000001, publish_rate_controller_stats));

  printf("  *** ControllerManager(): D ***\n");

  // create controller loader
  controller_loader_.reset(new pluginlib::ClassLoader<controller_interface::ControllerBase>("controller_interface",
                                                                                            "controller_interface::ControllerBase"));

  printf("  *** ControllerManager(): E ***\n");

  // Advertise services (this should be the last thing we do in init)
  srv_list_controllers_ = cm_node_.advertiseService("list_controllers", &ControllerManager::listControllersSrv, this);
  srv_list_controller_types_ = cm_node_.advertiseService("list_controller_types", &ControllerManager::listControllerTypesSrv, this);
  srv_load_controller_ = cm_node_.advertiseService("load_controller", &ControllerManager::loadControllerSrv, this);
  srv_unload_controller_ = cm_node_.advertiseService("unload_controller", &ControllerManager::unloadControllerSrv, this);
  srv_switch_controller_ = cm_node_.advertiseService("switch_controller", &ControllerManager::switchControllerSrv, this);
  srv_reload_libraries_ = cm_node_.advertiseService("reload_controller_libraries", &ControllerManager::reloadControllerLibrariesSrv, this);

  printf("  *** ControllerManager(): Done ***\n");
}


ControllerManager::~ControllerManager()
{}




// Must be realtime safe.
void ControllerManager::update(const ros::Time& time, bool reset_controllers)
{
  ros::Time start = ros::Time::now();

  used_by_realtime_ = current_controllers_list_;
  std::vector<ControllerSpec> &controllers = controllers_lists_[used_by_realtime_];

  // Restart all running controllers if motors are re-enabled
  if (reset_controllers){
    for (size_t i=0; i<controllers.size(); i++){
      if (controllers[i].c->isRunning()){
        controllers[i].c->stopRequest(time);
        controllers[i].c->startRequest(time);
      }
    }
  }


 // Update all controllers in scheduling order
  ros::Time start_update = ros::Time::now();
  pre_update_stats_.acc((start_update - start).toSec());
  for (size_t i=0; i<controllers.size(); i++){
    ros::Time start = ros::Time::now();
    controllers[i].c->updateRequest(time);
    ros::Time end = ros::Time::now();
    controllers[i].stats->acc((end - start).toSec());
    if (end - start > ros::Duration(0.001)){
      controllers[i].stats->num_control_loop_overruns++;
      controllers[i].stats->time_last_control_loop_overrun = end;
    }
  }
  ros::Time end_update = ros::Time::now();
  update_stats_.acc((end_update - start_update).toSec());

  ros::Time end = ros::Time::now();
  post_update_stats_.acc((end - end_update).toSec());

  // publish state
  publishControllerStatistics();

  // there are controllers to start/stop
  if (please_switch_)
  {
    // stop controllers
    for (unsigned int i=0; i<stop_request_.size(); i++)
      if (!stop_request_[i]->stopRequest(time))
        ROS_FATAL("Failed to stop controller in realtime loop. This should never happen.");

    // start controllers
    for (unsigned int i=0; i<start_request_.size(); i++)
      if (!start_request_[i]->startRequest(time))
        ROS_FATAL("Failed to start controller in realtime loop. This should never happen.");

    start_request_.clear();
    stop_request_.clear();
    please_switch_ = false;
  }
}

controller_interface::ControllerBase* ControllerManager::getControllerByName(const std::string& name)
{
  std::vector<ControllerSpec> &controllers = controllers_lists_[current_controllers_list_];
  for (size_t i = 0; i < controllers.size(); ++i)
  {
    if (controllers[i].name == name)
      return controllers[i].c.get();
  }
  return NULL;
}

void ControllerManager::getControllerNames(std::vector<std::string> &names)
{
  boost::mutex::scoped_lock guard(controllers_lock_);
  std::vector<ControllerSpec> &controllers = controllers_lists_[current_controllers_list_];
  for (size_t i = 0; i < controllers.size(); ++i)
  {
    names.push_back(controllers[i].name);
  }
}


bool ControllerManager::loadController(const std::string& name)
{
  ROS_DEBUG("Will load controller '%s'", name.c_str());

  // lock controllers
  boost::mutex::scoped_lock guard(controllers_lock_);

  // get reference to controller list
  int free_controllers_list = (current_controllers_list_ + 1) % 2;
  while (ros::ok() && free_controllers_list == used_by_realtime_){
    if (!ros::ok())
      return false;
    usleep(200);
  }
  std::vector<ControllerSpec>
    &from = controllers_lists_[current_controllers_list_],
    &to = controllers_lists_[free_controllers_list];
  to.clear();

  // Copy all controllers from the 'from' list to the 'to' list
  for (size_t i = 0; i < from.size(); ++i)
    to.push_back(from[i]);

  // Checks that we're not duplicating controllers
  for (size_t j = 0; j < to.size(); ++j)
  {
    if (to[j].name == name)
    {
      to.clear();
      ROS_ERROR("A controller named '%s' was already loaded inside the controller manager", name.c_str());
      return false;
    }
  }

  ros::NodeHandle c_node;
  // Constructs the controller
  try{
    c_node = ros::NodeHandle(controller_node_, name);
  }
  catch(std::exception &e) {
    ROS_ERROR("Exception thrown while constructing nodehandle for controller with name '%s':\n%s", name.c_str(), e.what());
    return false;
  }
  catch(...){
    ROS_ERROR("Exception thrown while constructing nodehandle for controller with name '%s'", name.c_str());
    return false;
  }
  boost::shared_ptr<controller_interface::ControllerBase> c;
  std::string type;
  if (c_node.getParam("type", type))
  {
    ROS_DEBUG("Constructing controller '%s' of type '%s'", name.c_str(), type.c_str());
    try {
      c = controller_loader_->createInstance(type);
    }
    catch (const std::runtime_error &ex)
    {
      ROS_ERROR("Could not load class %s: %s", type.c_str(), ex.what());
    }
  }
  else
  {
    ROS_ERROR("Could not load controller '%s' because the type was not specified. Did you load the controller configuration on the parameter server?", name.c_str());
    to.clear();
    return false;
  }

  // checks if controller was constructed
  if (!c)
  {
    ROS_ERROR("Could not load controller '%s' because controller type '%s' does not exist.",  name.c_str(), type.c_str());
    ROS_ERROR("Use 'rosservice call controller_manager/list_controller_types' to get the available types");
    to.clear();
    return false;
  }

  // Initializes the controller
  ROS_DEBUG("Initializing controller '%s'", name.c_str());
  bool initialized;
  try{
    initialized = c->initRequest(hw_, c_node);
  }
  catch(std::exception &e){
    ROS_ERROR("Exception thrown while initializing controller %s.\n%s", name.c_str(), e.what());
    initialized = false;
  }
  catch(...){
    ROS_ERROR("Exception thrown while initializing controller %s", name.c_str());
    initialized = false;
  }
  if (!initialized)
  {
    to.clear();
    ROS_ERROR("Initializing controller '%s' failed", name.c_str());
    return false;
  }
  ROS_DEBUG("Initialized controller '%s' succesful", name.c_str());

  // Adds the controller to the new list
  to.resize(to.size() + 1);
  to[to.size()-1].type = type;
  to[to.size()-1].name = name;
  to[to.size()-1].c = c;

  // Resize controller state vector
  pub_controller_stats_.lock();
  pub_controller_stats_.msg_.controller.resize(to.size());
  for (size_t i=0; i<to.size(); i++){
    pub_controller_stats_.msg_.controller[i].name = to[i].name;
    pub_controller_stats_.msg_.controller[i].type = to[i].type;
  }

  // Destroys the old controllers list when the realtime thread is finished with it.
  int former_current_controllers_list_ = current_controllers_list_;
  current_controllers_list_ = free_controllers_list;
  while (ros::ok() && used_by_realtime_ == former_current_controllers_list_){
    if (!ros::ok())
      return false;
    usleep(200);
  }
  from.clear();
  pub_controller_stats_.unlock();

  ROS_DEBUG("Successfully load controller '%s'", name.c_str());
  return true;
}




bool ControllerManager::unloadController(const std::string &name)
{
  ROS_DEBUG("Will unload controller '%s'", name.c_str());

  // lock the controllers
  boost::mutex::scoped_lock guard(controllers_lock_);

  // get reference to controller list
  int free_controllers_list = (current_controllers_list_ + 1) % 2;
  while (ros::ok() && free_controllers_list == used_by_realtime_){
    if (!ros::ok())
      return false;
    usleep(200);
  }
  std::vector<ControllerSpec>
    &from = controllers_lists_[current_controllers_list_],
    &to = controllers_lists_[free_controllers_list];
  to.clear();

  // Transfers the running controllers over, skipping the one to be removed and the running ones.
  bool removed = false;
  for (size_t i = 0; i < from.size(); ++i)
  {
    if (from[i].name == name){
      if (from[i].c->isRunning()){
        to.clear();
        ROS_ERROR("Could not unload controller with name %s because it is still running",
                  name.c_str());
        return false;
      }
      removed = true;
    }
    else
      to.push_back(from[i]);
  }

  // Fails if we could not remove the controllers
  if (!removed)
  {
    to.clear();
    ROS_ERROR("Could not unload controller with name %s because no controller with this name exists",
              name.c_str());
    return false;
  }

  // Resize controller state vector
  ROS_DEBUG("Resizing controller state vector");
  pub_controller_stats_.lock();
  pub_controller_stats_.msg_.controller.resize(to.size());
  for (size_t i=0; i<to.size(); i++){
    pub_controller_stats_.msg_.controller[i].name = to[i].name;
    pub_controller_stats_.msg_.controller[i].type = to[i].type;
  }

  // Destroys the old controllers list when the realtime thread is finished with it.
  ROS_DEBUG("Realtime switches over to new controller list");
  int former_current_controllers_list_ = current_controllers_list_;
  current_controllers_list_ = free_controllers_list;
  while (ros::ok() && used_by_realtime_ == former_current_controllers_list_){
    if (!ros::ok())
      return false;
    usleep(200);
  }
  ROS_DEBUG("Destruct controller");
  from.clear();
  ROS_DEBUG("Destruct controller finished");
  pub_controller_stats_.unlock();

  ROS_DEBUG("Successfully unloaded controller '%s'", name.c_str());
  return true;
}



bool ControllerManager::switchController(const std::vector<std::string>& start_controllers,
                                         const std::vector<std::string>& stop_controllers,
                                         int strictness)
{
  if (!stop_request_.empty() || !start_request_.empty())
    ROS_FATAL("The switch controller stop and start list are not empty that the beginning of the swithcontroller call. This should not happen.");

  if (strictness == 0){
    ROS_WARN("Controller Manager: To switch controllers you need to specify a strictness level of STRICT or BEST_EFFORT. Defaulting to BEST_EFFORT.");
    strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;
  }

  ROS_DEBUG("switching controllers:");
  for (unsigned int i=0; i<start_controllers.size(); i++)
    ROS_DEBUG(" - starting controller %s", start_controllers[i].c_str());
  for (unsigned int i=0; i<stop_controllers.size(); i++)
    ROS_DEBUG(" - stopping controller %s", stop_controllers[i].c_str());

  // lock controllers
  boost::mutex::scoped_lock guard(controllers_lock_);

  controller_interface::ControllerBase* ct;
  // list all controllers to stop
  for (unsigned int i=0; i<stop_controllers.size(); i++)
  {
    ct = getControllerByName(stop_controllers[i]);
    if (ct == NULL){
      if (strictness ==  controller_manager_msgs::SwitchController::Request::STRICT){
        ROS_ERROR("Could not stop controller with name %s because no controller with this name exists",
                  stop_controllers[i].c_str());
        stop_request_.clear();
        return false;
      }
      else{
        ROS_DEBUG("Could not stop controller with name %s because no controller with this name exists",
                  stop_controllers[i].c_str());
      }
    }
    else{
      ROS_DEBUG("Found controller %s that needs to be stopped in list of controllers",
                stop_controllers[i].c_str());
      stop_request_.push_back(ct);
    }
  }
  ROS_DEBUG("Stop request vector has size %i", (int)stop_request_.size());

  // list all controllers to start
  for (unsigned int i=0; i<start_controllers.size(); i++)
  {
    ct = getControllerByName(start_controllers[i]);
    if (ct == NULL){
      if (strictness ==  controller_manager_msgs::SwitchController::Request::STRICT){
        ROS_ERROR("Could not start controller with name %s because no controller with this name exists",
                  start_controllers[i].c_str());
        stop_request_.clear();
        start_request_.clear();
        return false;
      }
      else{
        ROS_DEBUG("Could not start controller with name %s because no controller with this name exists",
                  start_controllers[i].c_str());
      }
    }
    else{
      ROS_DEBUG("Found controller %s that needs to be started in list of controllers",
                start_controllers[i].c_str());
      start_request_.push_back(ct);
    }
  }
  ROS_DEBUG("Start request vector has size %i", (int)start_request_.size());

  // start the atomic controller switching
  switch_strictness_ = strictness;
  please_switch_ = true;

  // wait until switch is finished
  ROS_DEBUG("Request atomic controller switch from realtime loop");
  while (ros::ok() && please_switch_){
    if (!ros::ok())
      return false;
    usleep(100);
  }
  ROS_DEBUG("Successfully switched controllers");
  return true;
}



void ControllerManager::publishControllerStatistics()
{
  ros::Time now = ros::Time::now();
  if (now > last_published_controller_stats_ + publish_period_controller_stats_)
  {
    if (pub_controller_stats_.trylock())
    {
      while (last_published_controller_stats_ + publish_period_controller_stats_ < now)
        last_published_controller_stats_ += publish_period_controller_stats_;

      // controller state
      std::vector<ControllerSpec> &controllers = controllers_lists_[used_by_realtime_];
      assert(pub_controller_stats_.msg_.controller.size() == controllers.size());
      for (unsigned int i = 0; i < controllers.size(); ++i)
      {
        controller_manager_msgs::ControllerStatistics *out = &pub_controller_stats_.msg_.controller[i];
        out->timestamp = now;
        out->running = controllers[i].c->isRunning();
        out->max_time = ros::Duration(boost::accumulators::max(controllers[i].stats->acc));
        out->mean_time = ros::Duration(boost::accumulators::mean(controllers[i].stats->acc));
        out->variance_time = ros::Duration(std::sqrt(boost::accumulators::variance(controllers[i].stats->acc)));
        out->num_control_loop_overruns = controllers[i].stats->num_control_loop_overruns;
        out->time_last_control_loop_overrun = controllers[i].stats->time_last_control_loop_overrun;
      }

      pub_controller_stats_.msg_.header.stamp = ros::Time::now();

      pub_controller_stats_.unlockAndPublish();
    }
  }
}



bool ControllerManager::reloadControllerLibrariesSrv(
  controller_manager_msgs::ReloadControllerLibraries::Request &req,
  controller_manager_msgs::ReloadControllerLibraries::Response &resp)
{
  // lock services
  ROS_DEBUG("reload libraries service called");
  boost::mutex::scoped_lock guard(services_lock_);
  ROS_DEBUG("reload libraries service locked");

  // only reload libraries if no controllers are running
  std::vector<std::string> controllers;
  getControllerNames(controllers);
  if (!controllers.empty() && !req.force_kill){
    ROS_ERROR("Controller manager: Cannot reload controller libraries because there are still %i controllers running", (int)controllers.size());
    resp.ok = false;
    return true;
  }

  // kill running controllers if requested
  if (!controllers.empty()){
    ROS_INFO("Controller manager: Killing all running controllers");
    std::vector<std::string> empty;
    if (!switchController(empty,controllers, controller_manager_msgs::SwitchController::Request::BEST_EFFORT)){
      ROS_ERROR("Controller manager: Cannot reload controller libraries because failed to stop running controllers");
      resp.ok = false;
      return true;
    }
    for (unsigned int i=0; i<controllers.size(); i++){
      if (!unloadController(controllers[i])){
        ROS_ERROR("Controller manager: Cannot reload controller libraries because failed to unload controller %s",
                  controllers[i].c_str());
        resp.ok = false;
        return true;
      }
    }
    getControllerNames(controllers);
  }
  assert(controllers.empty());

  // create new controller loader
  controller_loader_.reset(new pluginlib::ClassLoader<controller_interface::ControllerBase>("controller_interface",
                                                                                            "controller_interface::Controller"));
  ROS_INFO("Controller manager: reloaded controller libraries");
  resp.ok = true;

  ROS_DEBUG("reload libraries service finished");
  return true;
}


bool ControllerManager::listControllerTypesSrv(
  controller_manager_msgs::ListControllerTypes::Request &req,
  controller_manager_msgs::ListControllerTypes::Response &resp)
{
  // pretend to use the request
  (void) req;

  // lock services
  ROS_DEBUG("list types service called");
  boost::mutex::scoped_lock guard(services_lock_);
  ROS_DEBUG("list types service locked");

  resp.types = controller_loader_->getDeclaredClasses();

  ROS_DEBUG("list types service finished");
  return true;
}


bool ControllerManager::listControllersSrv(
  controller_manager_msgs::ListControllers::Request &req,
  controller_manager_msgs::ListControllers::Response &resp)
{
  // pretend to use the request
  (void) req;

  // lock services
  ROS_DEBUG("list controller service called");
  boost::mutex::scoped_lock services_guard(services_lock_);
  ROS_DEBUG("list controller service locked");

  // lock controllers to get all names/types/states
  boost::mutex::scoped_lock controller_guard(controllers_lock_);
  std::vector<ControllerSpec> &controllers = controllers_lists_[current_controllers_list_];
  resp.name.resize(controllers.size());
  resp.type.resize(controllers.size());
  resp.state.resize(controllers.size());

  for (size_t i = 0; i < controllers.size(); ++i)
  {
    // add controller state
    resp.name[i] = controllers[i].name;
    resp.type[i] = controllers[i].type;
    if (controllers[i].c->isRunning())
      resp.state[i] = "running";
    else
      resp.state[i] = "stopped";
  }

  ROS_DEBUG("list controller service finished");
  return true;
}


bool ControllerManager::loadControllerSrv(
  controller_manager_msgs::LoadController::Request &req,
  controller_manager_msgs::LoadController::Response &resp)
{
  // lock services
  ROS_DEBUG("loading service called for controller %s ",req.name.c_str());
  boost::mutex::scoped_lock guard(services_lock_);
  ROS_DEBUG("loading service locked");

  resp.ok = loadController(req.name);

  ROS_DEBUG("loading service finished for controller %s ",req.name.c_str());
  return true;
}


bool ControllerManager::unloadControllerSrv(
  controller_manager_msgs::UnloadController::Request &req,
  controller_manager_msgs::UnloadController::Response &resp)
{
  // lock services
  ROS_DEBUG("unloading service called for controller %s ",req.name.c_str());
  boost::mutex::scoped_lock guard(services_lock_);
  ROS_DEBUG("unloading service locked");

  resp.ok = unloadController(req.name);

  ROS_DEBUG("unloading service finished for controller %s ",req.name.c_str());
  return true;
}


bool ControllerManager::switchControllerSrv(
  controller_manager_msgs::SwitchController::Request &req,
  controller_manager_msgs::SwitchController::Response &resp)
{
  // lock services
  ROS_DEBUG("switching service called");
  boost::mutex::scoped_lock guard(services_lock_);
  ROS_DEBUG("switching service locked");

  resp.ok = switchController(req.start_controllers, req.stop_controllers, req.strictness);

  ROS_DEBUG("switching service finished");
  return true;
}

}
