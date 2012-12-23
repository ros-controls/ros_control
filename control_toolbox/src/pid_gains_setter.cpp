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

#include "control_toolbox/pid_gains_setter.h"

namespace control_toolbox {

PidGainsSetter::~PidGainsSetter()
{
  serve_set_gains_.shutdown();
}

PidGainsSetter& PidGainsSetter::add(Pid *pid)
{
  assert(pid);
  pids_.push_back(pid);
  return *this;
}

void PidGainsSetter::advertise(const ros::NodeHandle &n)
{
  node_ = n;
  serve_set_gains_ = node_.advertiseService("set_gains", &PidGainsSetter::setGains, this);
}

bool PidGainsSetter::setGains(control_toolbox::SetPidGains::Request &req,
                              control_toolbox::SetPidGains::Response &resp)
{
  for (size_t i = 0; i < pids_.size(); ++i)
    pids_[i]->setGains(req.p, req.i, req.d, req.i_clamp, -req.i_clamp);
  node_.setParam("p", req.p);
  node_.setParam("i", req.i);
  node_.setParam("d", req.d);
  node_.setParam("i_clamp", req.i_clamp);
  return true;
}

}
