///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018, Clearpath Robotics
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the names of Clearpath Robotics nor the names of its
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

/** \author Mike Purvis */

#ifndef CONTROLLER_INTERFACE_COMPOSITE_CONTROLLER_H
#define CONTROLLER_INTERFACE_COMPOSITE_CONTROLLER_H

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>

namespace controller_interface
{

/**
 * \brief Merge together two ClaimedResources vectors, detecting whether the
 * any of the merged-in resources were already present. The merge will be
 * fully executed even if the function returns false (eg, indicating a conflict).
 *
 * \param[out] target ClaimedResources to merge resources into.
 *
 * \param source ClaimedResources to draw resources from.
 *
 * \return true if no resources were duplicated between the target and source.
 */
static bool mergeClaimedResources(ControllerBase::ClaimedResources& target,
                                  const ControllerBase::ClaimedResources& source)
{
  bool conflict = false;
  for (const auto& claimed : source)
  {
    // Check if there's a ClaimedResource of this interface type already in the target.
    bool resource_in_target = false;
    for (auto& target_claimed : target)
    {
      if (target_claimed.hardware_interface == claimed.hardware_interface)
      {
        resource_in_target = true;

        size_t size_before = target_claimed.resources.size();
        target_claimed.resources.insert(claimed.resources.begin(),
                                        claimed.resources.end());

        // Since it's a set, if the resulting size isn't big enough, we know that
        // one or more of the inserted items was already present.
        if (target_claimed.resources.size() < size_before + claimed.resources.size())
        {
          conflict = true;
        }

        break;
      }
    }

    if (!resource_in_target)
    {
      // There isn't, so append the whole ClaimedResource to the vector.
      target.push_back(claimed);
    }
  }
  return !conflict;
}


template <typename... Controllers>
class CompositeController: public virtual ControllerBase, public Controllers...
{
public:
  CompositeController()
  {
    state_ = CONSTRUCTED;
  }

  virtual ~CompositeController()
  {
  }

protected:
  /** \brief This init function is called to initialize the composite controller
   * from a non-realtime thread. This is called after all sub-controllers have
   * already initialized successfully, in order.
   *
   * \param root_nh A NodeHandle in the root of the controller manager namespace.
   * This is where the ROS interfaces are setup (publishers, subscribers, services).
   *
   * \param controller_nh A NodeHandle in the namespace of the controller.
   * This is where the controller-specific configuration resides.
   *
   * \returns True if initialization was successful and the controller
   * is ready to be started.
   */
  virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
  {
    return true;
  }

  /**
   * \brief Calls the starting() methods on all sub-controllers, in order.
   *
   * This default implementation of starting may be overridden if a behaviour
   * more complicated than simply initializing the sub-controllers in order
   * is desired.
   */
  void starting(const ros::Time& time) override
  {
    callStarting<Controllers...>(time);
  }

  /**
   * \brief Calls the starting() methods on all sub-controllers, in order.
   *
   * This default implementation of update may be overridden if a behaviour
   * more complicated than simply initializing the sub-controllers in order
   * is desired. If update is overridden, you can call through to this method
   * or call the individual update methods of your sub-controllers.
   */
  void update(const ros::Time& time, const ros::Duration& period) override
  {
    callUpdate<Controllers...>(time, period);
  }

  /**
   * \brief Calls the starting() methods on all sub-controllers, in order.
   *
   * This default implementation of stopping may be overridden if a behaviour
   * more complicated than simply initializing the sub-controllers in order
   * is desired.
   */
  void stopping(const ros::Time& time) override
  {
    callStopping<Controllers...>(time);
  }

  /**
   * \brief Initialize the sub-controllers from a RobotHW pointer, gathering up
   * the total claimed resources across all of them.
   *
   * \param robot_hw The robot hardware abstraction.
   *
   * \param root_nh A NodeHandle in the root of the controller manager namespace.
   * This is where the ROS interfaces are setup (publishers, subscribers, services).
   *
   * \param controller_nh A NodeHandle in the namespace of the controller.
   * This is where the controller-specific configuration resides.
   *
   * \param[out] claimed_resources The resources claimed by all sub-controllers.
   * They can belong to multiple hardware interfaces.
   *
   * \returns True if initialization was successful and the composite controller
   * is ready to be started.
   */
  virtual bool initRequest(hardware_interface::RobotHW* robot_hw,
                           ros::NodeHandle&             root_nh,
                           ros::NodeHandle&             controller_nh,
                           ClaimedResources&            claimed_resources)
  {
    // check if construction finished cleanly
    if (state_ != CONSTRUCTED){
      ROS_ERROR("Cannot initialize this controller because it failed to be constructed.");
      return false;
    }

    if (callInitRequest<Controllers...>(robot_hw, root_nh, controller_nh, claimed_resources) &&
        init(root_nh, controller_nh))
    {
      state_ = INITIALIZED;
      return true;
    }

    ROS_ERROR("One or more components of the composite controller failed to initialize.");
    return false;
  }

private:
  template<class Controller>
  void callStarting(const ros::Time& time)
  {
    Controller::starting(time);
  }

  template<class Controller1, class Controller2, class... More>
  void callStarting(const ros::Time& time)
  {
    callStarting<Controller1>(time);
    callStarting<Controller2, More...>(time);
  }

  template<class Controller>
  void callUpdate(const ros::Time& time, const ros::Duration& period)
  {
    Controller::update(time, period);
  }

  template<class Controller1, class Controller2, class... More>
  void callUpdate(const ros::Time& time, const ros::Duration& period)
  {
    callUpdate<Controller1>(time, period);
    callUpdate<Controller2, More...>(time, period);
  }

  template<class Controller>
  void callStopping(const ros::Time& time)
  {
    Controller::stopping(time);
  }

  template<class Controller1, class Controller2, class... More>
  void callStopping(const ros::Time& time)
  {
    callStopping<Controller1>(time);
    callStopping<Controller2, More...>(time);
  }

  template<class Controller>
  bool callInitRequest(hardware_interface::RobotHW* robot_hw,
                        ros::NodeHandle&             root_nh,
                        ros::NodeHandle&             controller_nh,
                        ClaimedResources&            all_claimed_resources)
  {
    ClaimedResources claimed_resources;
    bool ret = Controller::initRequest(robot_hw, root_nh, controller_nh, claimed_resources);
    if (state_ != INITIALIZED)
    {
      return false;
    }

    // Revert the global state variable so that follow-on controllers see an uninitialized state
    // and can also initialize themselves.
    state_ = CONSTRUCTED;

    // Add the resources claimed by this specific controller to the overall list, checking
    // first that they aren't already in there (as that would be a conflict).
    if (!mergeClaimedResources(all_claimed_resources, claimed_resources))
    {
      ROS_ERROR_STREAM("Unable to initialize composed controller; one or more resources "
                       "claimed by multiple sub-controllers.");
      return false;
    }

    return ret;
  }

  template<class Controller1, class Controller2, class... More>
  bool callInitRequest(hardware_interface::RobotHW* robot_hw,
                        ros::NodeHandle&             root_nh,
                        ros::NodeHandle&             controller_nh,
                        ClaimedResources&            all_claimed_resources)
  {
    return callInitRequest<Controller1>(robot_hw, root_nh, controller_nh, all_claimed_resources) &&
      callInitRequest<Controller2, More...>(robot_hw, root_nh, controller_nh, all_claimed_resources);
  }

  CompositeController(const CompositeController& c);
  CompositeController& operator =(const CompositeController& c);
};

} // namespace

#endif
