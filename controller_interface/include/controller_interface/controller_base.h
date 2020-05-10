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

/*
 * Author: Wim Meeussen
 */

#pragma once


#include <ros/node_handle.h>
#include <hardware_interface/robot_hw.h>
#include <memory>

namespace controller_interface
{

/** \brief Abstract %Controller Interface
 *
 *
 *
 */

class ControllerBase
{
public:
  ControllerBase() = default;
  virtual ~ControllerBase() = default;
  ControllerBase(const ControllerBase&) = delete;
  ControllerBase& operator=(const ControllerBase&) = delete;
  ControllerBase(ControllerBase&&) = delete;
  ControllerBase& operator=(ControllerBase&&) = delete;

  /** \name Real-Time Safe Functions
   *\{*/

  /** \brief This is called from within the realtime thread just before the
   * first call to \ref update
   *
   * \param time The current time
   */
  virtual void starting(const ros::Time& /*time*/) {}

  /** \brief This is called periodically by the realtime thread when the controller is running
   *
   * \param time The current time
   * \param period The time passed since the last call to \ref update
   */
  virtual void update(const ros::Time& time, const ros::Duration& period) = 0;

  /** \brief This is called from within the realtime thread just after the last
   * update call before the controller is stopped
   *
   * \param time The current time
   */
  virtual void stopping(const ros::Time& /*time*/) {}

  /** \brief This is called from within the realtime thread while the controller is
   * waiting to start
   *
   * \param time The current time
   */
  virtual void waiting(const ros::Time& /*time*/) {}

  /** \brief This is called from within the realtime thread when the controller needs
   * to be aborted
   *
   * \param time The current time
   */
  virtual void aborting(const ros::Time& /*time*/) {}

  /** \brief Check if the controller is initialized
   * \returns true if the controller is initialized
   */
  bool isInitialized() const
  {
    return state_ == ControllerState::INITIALIZED;
  }

  /** \brief Check if the controller is running
   * \returns true if the controller is running
   */
  bool isRunning() const
  {
    return state_ == ControllerState::RUNNING;
  }

  /** \brief Check if the controller is stopped
   * \returns true if the controller is stopped
   */
  bool isStopped() const
  {
    return state_ == ControllerState::STOPPED;
  }

  /** \brief Check if the controller is waiting
   * \returns true if the controller is waiting
   */
  bool isWaiting() const
  {
    return state_ == ControllerState::WAITING;
  }

  /** \brief Check if the controller is aborted
   * \returns true if the controller is aborted
   */
  bool isAborted() const
  {
    return state_ == ControllerState::ABORTED;
  }

  /// Calls \ref update only if this controller is running.
  void updateRequest(const ros::Time& time, const ros::Duration& period)
  {
    if (state_ == ControllerState::RUNNING)
    {
      update(time, period);
    }
  }

  /// Calls \ref starting unless this controller is just constructed
  bool startRequest(const ros::Time& time)
  {
    // start works from any state, except CONSTRUCTED
    if (state_ != ControllerState::CONSTRUCTED)
    {
      starting(time);
      state_ = ControllerState::RUNNING;
      return true;
    }
    else
    {
      ROS_FATAL("Failed to start controller. It is not initialized.");
      return false;
    }
  }

  /// Calls \ref stopping unless this controller is just constructed
  bool stopRequest(const ros::Time& time)
  {
    // stop works from any state, except CONSTRUCTED
    if (state_ != ControllerState::CONSTRUCTED)
    {
      stopping(time);
      state_ = ControllerState::STOPPED;
      return true;
    }
    else
    {
      ROS_FATAL("Failed to stop controller. It is not initialized.");
      return false;
    }
  }

  /// Calls \ref waiting unless this controller is just constructed
  bool waitRequest(const ros::Time& time)
  {
    // wait works from any state, except CONSTRUCTED
    if (state_ != ControllerState::CONSTRUCTED)
    {
      waiting(time);
      state_ = ControllerState::WAITING;
      return true;
    }
    else
    {
      ROS_FATAL("Failed to wait controller. It is not initialized.");
      return false;
    }
  }

  /// Calls \ref abort unless this controller is just constructed
  bool abortRequest(const ros::Time& time)
  {
    // abort works from any state, except CONSTRUCTED
    if (state_ != ControllerState::CONSTRUCTED)
    {
      aborting(time);
      state_ = ControllerState::ABORTED;
      return true;
    }
    else
    {
      ROS_FATAL("Failed to abort controller. It is not initialized.");
      return false;
    }
  }

  /*\}*/

  /** \name Non Real-Time Safe Functions
   *\{*/

  typedef std::vector<hardware_interface::InterfaceResources> ClaimedResources;

  /** \brief Request that the controller be initialized
   *
   * \param robot_hw The robot hardware abstraction.
   *
   * \param root_nh A NodeHandle in the root of the controller manager namespace.
   * This is where the ROS interfaces are setup (publishers, subscribers, services).
   *
   * \param controller_nh A NodeHandle in the namespace of the controller.
   * This is where the controller-specific configuration resides.
   *
   * \param[out] claimed_resources The resources claimed by this controller.
   * They can belong to multiple hardware interfaces.
   *
   * \returns True if initialization was successful and the controller
   * is ready to be started.
   */
  virtual bool initRequest(hardware_interface::RobotHW* robot_hw,
                           ros::NodeHandle&             root_nh,
                           ros::NodeHandle&             controller_nh,
                           ClaimedResources&            claimed_resources) = 0;

  /*\}*/

  enum class ControllerState
  {
    CONSTRUCTED,
    INITIALIZED,
    RUNNING,
    STOPPED,
    WAITING,
    ABORTED
  };

  /// The current execution state of the controller
  ControllerState state_ = ControllerState::CONSTRUCTED;

};

typedef std::shared_ptr<ControllerBase> ControllerBaseSharedPtr;

}
