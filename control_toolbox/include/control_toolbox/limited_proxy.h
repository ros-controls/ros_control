/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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
#ifndef CONTROL_TOOLBOX_LIMITED_PROXY_H
#define CONTROL_TOOLBOX_LIMITED_PROXY_H

namespace control_toolbox {

class LimitedProxy
{
 public:
  // Controller parameter values
  double mass_;                 // Estimate of the joint mass
  double Kd_;                   // Damping gain
  double Kp_;                   // Position gain
  double Ki_;                   // Integral gain
  double Ficl_;                 // Integral force clamp
  double effort_limit_;         // Limit on output force
  double vel_limit_;            // Limit on velocity
  double pos_upper_limit_;      // Upper position bound
  double pos_lower_limit_;      // Lower position bound
  double lambda_proxy_;         // Bandwidth of proxy reconvergence
  double acc_converge_;         // Acceleration of proxy reconvergence

  
 LimitedProxy()
   : mass_(0.0), Kd_(0.0), Kp_(0.0), Ki_(0.0), Ficl_(0.0),
     effort_limit_(0.0), vel_limit_(0.0),
     pos_upper_limit_(0.0), pos_lower_limit_(0.0),
     lambda_proxy_(0.0), acc_converge_(0.0)
  {
  }

  void reset(double pos_act, double vel_act);

  double update(double pos_des, double vel_des, double acc_des,
		double pos_act, double vel_act, double dt);

 private:
  // Controller state values
  double last_proxy_pos_;       // Proxy position
  double last_proxy_vel_;       // Proxy velocity
  double last_proxy_acc_;       // Proxy acceleration

  double last_vel_error_;       // Velocity error
  double last_pos_error_;       // Position error
  double last_int_error_;       // Integral error
};
  
} // namespace

#endif
