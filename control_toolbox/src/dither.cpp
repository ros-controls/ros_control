/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

// Original version: Kevin Watts <watts@willowgarage.com>

#include <control_toolbox/dither.h>

namespace control_toolbox {

Dither::Dither() : amplitude_(0), has_saved_value_(false)
{

}

Dither::~Dither()
{
}

double Dither::update()
{
  if (has_saved_value_)
  {
    has_saved_value_ = false;
    return saved_value_;
  }

  // Generates gaussian random noise using the polar method.
  double v1, v2, r;
  for (int i = 0; i < 100; ++i)
  {
    v1 = 2.0 * erand48(seed_) - 1.0;  //  [-1, 1]
    v2 = 2.0 * erand48(seed_) - 1.0;  //  [-1, 1]
    r = v1*v1 + v2*v2;
    if (r <= 1.0)
      break;
  }
  if (r > 1.0)
    r = 1.0;

  double f = sqrt(-2.0 * log(r) / r);
  double current = amplitude_ * f * v1;
  saved_value_ = amplitude_ * f * v2;
  has_saved_value_ = true;

  return current;
}

}
