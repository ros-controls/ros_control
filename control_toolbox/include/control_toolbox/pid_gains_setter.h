/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Exposes a ROS interface for tuning a set of pid controllers.
//
// Author: Stuart Glaser

#ifndef PID_GAINS_SETTER_H
#define PID_GAINS_SETTER_H

#include <vector>
#include <string>
#include "ros/node_handle.h"
#include "control_toolbox/pid.h"
#include "control_toolbox/SetPidGains.h"

namespace control_toolbox {

/**
   \brief Sets up services for quickly changing the gains for a \ref
   control_toolbox::Pid Pid object.

   The PidGainsSetter class provides services for changing the gains
   of Pid objects over ROS.  It advertise the "set_gains" service in the NodeHandle's
   namespace, with a type of control_toolbox/SetPidGains.

   To use the object, add pids to the gains setter and then call
   advertise().  The PidGainsSetter will then update the gains of all
   the Pid objects when you call the set_gains service.  (If you wish
   to have Pids with different gains, then you should use multiple
   PidGainsSetter objects).

 \verbatim
 ros::NodeHandle node;
 control_toolbox::Pid a, b, c;
 control_toolbox::PidGainsSetter pid_gains_setter;

 pid_gains_setter.add(&a);
 pid_gains_setter.add(&b).add(&c);
 pid_gains_setter.advertise(node);
 \endverbatim

   ROS API

   - \b set_gains [control_toolbox::SetPidGains] - Updates the gains
     of all the Pid objects that have been added.
 */
class PidGainsSetter
{
public:
  PidGainsSetter() {}
  ~PidGainsSetter();

  /**
   * \brief Adds a Pid object.
   *
   * Adds a Pid object to be modified when new gains are set over the service.
   */
  PidGainsSetter& add(Pid *pid);

  /**
   * \brief Advertises the "set_gains" service, initializing the PidGainsSetter
   */
  void advertise(const ros::NodeHandle &n);

  /**
   * \brief Advertises the "set_gains" service, initializing the PidGainsSetter
   */
  void advertise(const std::string &ns) { advertise(ros::NodeHandle(ns)); }

  bool setGains(control_toolbox::SetPidGains::Request &req,
                control_toolbox::SetPidGains::Response &resp);

private:
  ros::NodeHandle node_;
  ros::ServiceServer serve_set_gains_;
  std::vector<Pid*> pids_;
};

}

#endif
