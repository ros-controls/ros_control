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


#ifndef CTRL_TOOLBOX_SINESWEEP_H
#define CTRL_TOOLBOX_SINESWEEP_H

#include <ros/ros.h>

namespace control_toolbox {
/***************************************************/
/*! \class SineSweep
    \brief Generates a sine sweep for frequency analysis of a joint

    This class basically calculates the output for
    a sine sweep. Where the signal is a sine wave, 
    whose frequency is exponentially increased from 
    \f$\omega_1\f$ to \f$\omega_2\f$ over \f$T\f$ seconds.<br> 
    
    \f$s(n) = A \sin [ K(e^{\delta t/L} - 1) ]\f$	<br>

    where:<br>
    \f$K = \frac{\omega_1T}{\ln \frac{\omega_2}{\omega_1} }\f$<br> 
    \f$L = \frac{T}{\ln \frac{\omega_2}{\omega_1} }\f$.<br>

*/
/***************************************************/

class SineSweep
{
public:

  /*!
   * \brief Constructor
   */
  SineSweep();

  /*!
   * \brief Destructor.
   */
  ~SineSweep();

  /*!
   * \brief Update the SineSweep loop with nonuniform time step size.
   *
   * \param dt Change in time since last call
   */
  double update(ros::Duration dt);

  /*!
   * \brief Intializes everything and calculates the constants for the sweep.
   *
   * \param start_freq  Start frequency of the sweep, \f$\omega_1\f$ .
   * \param end_freq  End frequency of the sweep, \f$\omega_2\f$.
   * \param duration  The duration of the sweep, \f$T\f$.
   * \param amplitude The amplitude of the sweep, \f$A\f$.
   */
  bool init(double start_freq, double end_freq, double duration, double amplitude);

private:
  double amplitude_;                        /**< Amplitude of the sweep. */
  ros::Duration duration_;                  /**< Duration of the sweep. */
  double start_angular_freq_;               /**< Start angular frequency of the sweep. */
  double end_angular_freq_;                 /**< End angular frequency of the sweep. */
  double K_;                                /**< Constant \f$K\f$. */
  double L_;                                /**< Constant \f$L\f$.*/
  double cmd_;                              /**< Command to send. */
};
}

#endif
