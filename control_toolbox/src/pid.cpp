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

#include "control_toolbox/pid.h"
#include "tinyxml.h"

namespace control_toolbox {

Pid::Pid(double p, double i, double d, double i_max, double i_min) :
  p_gain_(p), i_gain_(i), d_gain_(d), i_max_(i_max), i_min_(i_min)
{
  this->reset();
}

Pid::~Pid()
{
}

void Pid::initPid(double p, double i, double d, double i_max, double i_min)
{
  this->setGains(p,i,d,i_max,i_min);
  reset();
}

void Pid::reset()
{
  p_error_last_ = 0.0;
  p_error_ = 0.0;
  d_error_ = 0.0;
  i_term_  = 0.0;
  cmd_ = 0.0;
}

void Pid::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
{
  p = p_gain_;
  i = i_gain_;
  d = d_gain_;
  i_max = i_max_;
  i_min = i_min_;
}

void Pid::setGains(double p, double i, double d, double i_max, double i_min)
{
  p_gain_ = p;
  i_gain_ = i;
  d_gain_ = d;
  i_max_ = i_max;
  i_min_ = i_min;
}

bool Pid::initParam(const std::string& prefix)
{
  ros::NodeHandle nh(prefix);
  return this->init(nh);
}

bool Pid::initXml(TiXmlElement *config)
{
  p_gain_ = config->Attribute("p") ? atof(config->Attribute("p")) : 0.0;
  i_gain_ = config->Attribute("i") ? atof(config->Attribute("i")) : 0.0;
  d_gain_ = config->Attribute("d") ? atof(config->Attribute("d")) : 0.0;
  double i_clamp;
  i_clamp = config->Attribute("iClamp") ? atof(config->Attribute("iClamp")) : 0.0;
  i_max_ = std::abs(i_clamp);
  i_min_ = -std::abs(i_clamp);

  reset();
  return true;
}

bool Pid::init(const ros::NodeHandle &node)
{
  ros::NodeHandle n(node);
  if (!n.getParam("p", p_gain_)) {
    ROS_ERROR("No p gain specified for pid.  Namespace: %s", n.getNamespace().c_str());
    return false;
  }
  n.param("i", i_gain_, 0.0);
  n.param("d", d_gain_, 0.0);

  double i_clamp;
  n.param("i_clamp", i_clamp, 0.0);
  i_max_ = std::abs(i_clamp);
  i_min_ = -std::abs(i_clamp);

  reset();
  return true;
}

double Pid::computeCommand(double error, ros::Duration dt)
{
  double p_term, d_term;
  p_error_ = error; // this is error = target - state

  if (dt == ros::Duration(0.0) || std::isnan(error) || std::isinf(error))
    return 0.0;

  // Calculate proportional contribution to command
  p_term = p_gain_ * p_error_;

  //Calculate integral contribution to command
  i_term_ = i_term_ + i_gain_ * dt.toSec() * p_error_;

  // Limit i_term_ so that the limit is meaningful in the output
  i_term_ = std::max( i_min_, std::min( i_term_, i_max_) );

  // Calculate the derivative error
  if (dt.toSec() > 0.0)
  {
    d_error_ = (p_error_ - p_error_last_) / dt.toSec();
    p_error_last_ = p_error_;
  }
  // Calculate derivative contribution to command
  d_term = d_gain_ * d_error_;
  cmd_ = p_term + i_term_ + d_term;

  return cmd_;
}

double Pid::updatePid(double error, ros::Duration dt)
{
  double p_term, d_term;
  p_error_ = error; //this is pError = pState-pTarget

  if (dt == ros::Duration(0.0) || std::isnan(error) || std::isinf(error))
    return 0.0;

  // Calculate proportional contribution to command
  p_term = p_gain_ * p_error_;

  //Calculate integral contribution to command
  i_term_ = i_term_ + i_gain_ * dt.toSec() * p_error_;

  // Limit i_term_ so that the limit is meaningful in the output
  i_term_ = std::max( i_min_, std::min( i_term_, i_max_) );

  // Calculate the derivative error
  if (dt.toSec() > 0.0)
  {
    d_error_ = (p_error_ - p_error_last_) / dt.toSec();
    p_error_last_ = p_error_;
  }
  // Calculate derivative contribution to command
  d_term = d_gain_ * d_error_;
  cmd_ = - p_term - i_term_ - d_term;

  return cmd_;
}

double Pid::computeCommand(double error, double error_dot, ros::Duration dt)
{
  double p_term, d_term;
  p_error_ = error; // this is error = target - state
  d_error_ = error_dot;

  if (dt == ros::Duration(0.0) || std::isnan(error) || std::isinf(error) || std::isnan(error_dot) || std::isinf(error_dot))
    return 0.0;


  // Calculate proportional contribution to command
  p_term = p_gain_ * p_error_;

  //Calculate integral contribution to command
  i_term_ = i_term_ + i_gain_ * dt.toSec() * p_error_;

  // Limit i_term_ so that the limit is meaningful in the output
  i_term_ = std::max( i_min_, std::min( i_term_, i_max_) );

  // Calculate derivative contribution to command
  d_term = d_gain_ * d_error_;
  cmd_ = p_term + i_term_ + d_term;

  return cmd_;
}

double Pid::updatePid(double error, double error_dot, ros::Duration dt)
{
  double p_term, d_term;
  p_error_ = error; //this is pError = pState-pTarget
  d_error_ = error_dot;

  if (dt == ros::Duration(0.0) || std::isnan(error) || std::isinf(error) || std::isnan(error_dot) || std::isinf(error_dot))
    return 0.0;


  // Calculate proportional contribution to command
  p_term = p_gain_ * p_error_;

  //Calculate integral contribution to command
  i_term_ = i_term_ + i_gain_ * dt.toSec() * p_error_;

  // Limit i_term_ so that the limit is meaningful in the output
  i_term_ = std::max( i_min_, std::min( i_term_, i_max_) );

  // Calculate derivative contribution to command
  d_term = d_gain_ * d_error_;
  cmd_ = - p_term - i_term_ - d_term;

  return cmd_;
}



void Pid::setCurrentCmd(double cmd)
{
  cmd_ = cmd;
}

double Pid::getCurrentCmd()
{
  return cmd_;
}

void Pid::getCurrentPIDErrors(double *pe, double *ie, double *de)
{
  *pe = p_error_;
  *ie = i_gain_ ? i_term_/i_gain_ : 0.0;
  *de = d_error_;
}

}
