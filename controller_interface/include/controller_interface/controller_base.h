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


#ifndef CONTROLLER_INTERFACE_CONTROLLER_BASE_H
#define CONTROLLER_INTERFACE_CONTROLLER_BASE_H

#include <ros/node_handle.h>
#include <hardware_interface/robot_hw.h>


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
  ControllerBase(): state_(CONSTRUCTED){}
  virtual ~ControllerBase(){}

  /** \name Real-Time Safe Functions
   *\{*/

  /** \brief This is called from within the realtime thread just before the
   * first call to \ref update
   *
   * \param time The current time
   */
  virtual void starting(const ros::Time& time) {};

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
  virtual void stopping(const ros::Time& time) {};

  /** \brief Check if the controller is running
   * \returns true if the controller is running
   */
  bool isRunning()
  {
    return (state_ == RUNNING);
  }

  /// Calls \ref update only if this controller is running and the update period has elapsed
  void updateRequest(const ros::Time& time, const ros::Duration& min_period)
  {
    if (state_ != RUNNING) return;

    // get time since last update (update period)
    ros::Duration period;
    if (update_period_.isZero()) {
      period = min_period;
    } else {
      if (!last_update_time_.isZero() && (time >= last_update_time_)) {
        period = time - last_update_time_;
      } else {
        period = update_period_;
      }
    }

    // update only if update period has elapsed
    if (period >= update_period_)
    {
      update(time, period);
      last_update_time_ = time;
    }
  }

  /// Calls \ref starting only if this controller is initialized or already running
  bool startRequest(const ros::Time& time)
  {
    // start succeeds even if the controller was already started
    if (state_ == RUNNING || state_ == INITIALIZED){
      starting(time);
      state_ = RUNNING;
      last_update_time_ = ros::Time();
      return true;
    }
    else
      return false;
  }

  /// Calls \ref stopping only if this controller is initialized or already running
  bool stopRequest(const ros::Time& time)
  {
    // stop succeeds even if the controller was already stopped
    if (state_ == RUNNING || state_ == INITIALIZED){
      stopping(time);
      state_ = INITIALIZED;
      return true;
    }
    else
      return false;
  }

  /** \brief Returns the desired update period of this controller.
   *
   * \returns The update period.
   */
  const ros::Duration& getUpdatePeriod() const
  {
      return update_period_;
  }

  /** \brief Returns the desired update period of this controller.
   *
   * \param period The update period.
   * \note The controller will never be updated faster than the controller manager.
   */
  void setUpdatePeriod(const ros::Duration& period)
  {
      update_period_ = period;
  }

  /** \brief Returns the timestamp of the last controller update.
   *
   * \returns The last update timestamp. If the controller has not been updated since it was started, the return value will be ros::Time().
   */
  const ros::Time& getLastUpdateTime() const
  {
      return last_update_time_;
  }

  /*\}*/

  /** \name Non Real-Time Safe Functions
   *\{*/

  /// Get the name of this controller's hardware interface type
  virtual std::string getHardwareInterfaceType() const = 0;

  /** \brief Request that the controller be initialized
   *
   * \param hw The hardware interface to the robot.
   *
   * \param root_nh A NodeHandle in the root of the controller manager namespace.
   * This is where the ROS interfaces are setup (publishers, subscribers, services).
   *
   * \param controller_nh A NodeHandle in the namespace of the controller.
   * This is where the controller-specific configuration resides.
   *
   * \param[out] claimed_resources The resources claimed by this controller.
   *
   * \returns True if initialization was successful and the controller
   * is ready to be started.
   */
  virtual bool initRequest(hardware_interface::RobotHW* hw, ros::NodeHandle& root_nh, ros::NodeHandle &controller_nh,
                           std::set<std::string>& claimed_resources) = 0;

  /*\}*/

  /// The current execution state of the controller
  enum {CONSTRUCTED, INITIALIZED, RUNNING} state_;


private:
  ControllerBase(const ControllerBase &c);
  ControllerBase& operator =(const ControllerBase &c);

  /// The update period of the controller
  ros::Duration update_period_;

  /// The last update time of this controller
  ros::Time last_update_time_;

};

}


#endif
