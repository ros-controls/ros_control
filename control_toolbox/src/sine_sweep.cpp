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

// Original version: Melonee Wise <mwise@willowgarage.com>

#include <math.h>
#include <control_toolbox/sine_sweep.h>

namespace control_toolbox {

SineSweep::SineSweep()
{
  K_=0.0;
  L_=0.0;
  amplitude_=0.0;
  duration_ = ros::Duration(0.0);
  cmd_ = 0.0;
}

SineSweep::~SineSweep()
{
}

bool SineSweep::init(double start_freq, double end_freq, double duration, double amplitude)
{
  if (start_freq > end_freq)
    return false;
  if (duration < 0 || amplitude < 0)
    return false;
  
  amplitude_ = amplitude;
  duration_ = ros::Duration(duration);
  //calculate the angular fequencies
  start_angular_freq_ =2*M_PI*start_freq;
  end_angular_freq_ =2*M_PI*end_freq;
  
  //calculate the constants
  K_ = (start_angular_freq_*duration)/log(end_angular_freq_/start_angular_freq_);
  L_ = (duration)/log(end_angular_freq_/start_angular_freq_);
  
  //zero out the command
  cmd_ = 0.0;

  return true;
}

double SineSweep::update( ros::Duration dt)
{
  if(dt<=duration_)
  {
    cmd_= amplitude_*sin(K_*(exp((dt.toSec())/(L_))-1));
  }
  else
  {
    cmd_=0.0;
  }

  return cmd_;
}
}

