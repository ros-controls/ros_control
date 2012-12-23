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

#include <control_toolbox/limited_proxy.h>
#include <algorithm>
#include <cmath>

namespace control_toolbox {


// CONTINUOUS SECOND ORDER PROXY DYNAMICS
//
// Calculate the 2nd order dynamics with which the proxy will
// converge.  In particular, this calculates the acceleration a as a
// function of position p and velocity v:
//
//   a = a(p,v)
//
// It also calculates the partial dervatives da/dp and dq/dv.  The
// parameters are
//
//   lam	Bandwidth of convergence
//   acon	Acceleration available for convergence
//
// The dynamics are split into a local region (small positions) with
// classic linear dynamics and a global region (large positions) with
// nonlinear dynamics.  The regions and nonlinear dynamics are
// designed to allow convergence without overshoot with a limited
// acceleration of acon: the system will converge asymptotically along
// the critical line
//
//   v^2 = 2*acon * ( abs(p) - acon/lam^2 )
//
// which uses a constant acceleration of
//
//   a = - acon * sign(p)

static void calcDynamics2ndorder(double &a, double &dadp, double &dadv,
				 double p, double v, double lam, double acon)
{
  double lam2 = lam*lam;	// Lambda squared

  // Separate 3 regions: large positive, small, and large negative positions
  if (lam2*p > 3*acon)
  {
    // Large position position: Nonlinear dynamics
    a    = - 2.0*lam*v - sqrt(8.0*acon*(+lam2*p-acon)) + acon;
    dadv = - 2.0*lam;
    dadp =             - lam2 * sqrt(2.0*acon/(+lam2*p-acon));
  }
  else if (lam2*p > -3*acon)
  {
    // Small position: Use linear dynamics
    a    = - 2.0*lam*v - lam2*p;
    dadv = - 2.0*lam;
    dadp =             - lam2;
  }
  else
  {
    // Large negative position: Nonlinear dynamics
    a    = - 2.0*lam*v + sqrt(8.0*acon*(-lam2*p-acon)) - acon;
    dadv = - 2.0*lam;
    dadp =             - lam2 * sqrt(2.0*acon/(-lam2*p-acon));
  }

  // Note we don't explicitly limit the acceleration to acon.  First
  // such limits are more effectively imposed by force saturations.
  // Second, if the proxy starts with very large velocities, higher
  // accelerations may be helpful.  The dynamics are simply designed
  // not to need more than acon to avoid overshoot.
  return;
}



// CONTINUOUS FIRST ORDER PROXY DYNAMICS
//
// Calculate the 1st order dynamics with which the proxy will
// converge.  In particular, this calculates the acceleration a as a
// function of velocity v (no position dependence).
//
//   a = a(v)
//
// It also calculates the partial dervative dq/dv.  The parameters are
//
//   lam	Bandwidth of convergence
//   acon	Acceleration available for convergence
//
// This uses basic linear dynamics, ignoring acon, as no overshoot can
// occur in a first order system.

static void calcDynamics1storder(double &a, double &dadv,
				 double v, double lam, double acon)
{
  a    = - lam*v;
  dadv = - lam;

  return;
}


// CONTROLLER RESET
//
// Reset the state of the controller.

void LimitedProxy::reset(double pos_act, double vel_act)
{
  // Place the proxy at the actual position, which sets the error to zero.
  last_proxy_pos_ = pos_act;
  last_proxy_vel_ = vel_act;
  last_proxy_acc_ = 0.0;

  last_vel_error_ = 0.0;
  last_pos_error_ = 0.0;
  last_int_error_ = 0.0;
}


// CONTROLLER UPDATE
//
// Adjust the proxy and compute the controller force.  We use a
// trapezoidal integration scheme for maximal stability and best
// accuracy.

double LimitedProxy::update(double pos_des, double vel_des, double acc_des,
			    double pos_act, double vel_act, double dt)
{
  // Get the parameters.  This ensures that they can not change during
  // the calculations and are non-negative!
  double mass = abs(mass_);		// Estimate of the joint mass
  double Kd   = abs(Kd_);		// Damping gain
  double Kp   = abs(Kp_);		// Position gain
  double Ki   = abs(Ki_);		// Integral gain
  double Ficl = abs(Ficl_);		// Integral force clamp
  double Flim = abs(effort_limit_);	// Limit on output force
  double vlim = abs(vel_limit_);	// Limit on velocity
  double pmax = pos_upper_limit_;	// Upper position bound
  double pmin = pos_lower_limit_;	// Lower position bound
  double lam  = abs(lambda_proxy_);	// Bandwidth of proxy reconvergence
  double acon = abs(acc_converge_);	// Acceleration of proxy reconvergence

  // For numerical stability, upper bound the bandwidth by 2/dt.
  // Note this is safe for dt==0.
  if (lam * dt > 2.0)
    lam = 2.0/dt;

  // Other useful terms.
  double dt2 = dt * dt;			// Time step squared
  double dt3 = dt * dt * dt;		// Time step cubed

  // State values: current and last cycle.
  double pos_pxy;			// Current proxy position
  double vel_pxy;			// Current proxy velocity
  double acc_pxy;			// Current proxy acceleration
  double vel_err;			// Current velocity error
  double pos_err;			// Current position error
  double int_err;			// Current integral error

  double last_pos_pxy = last_proxy_pos_;
  double last_vel_pxy = last_proxy_vel_;
  double last_acc_pxy = last_proxy_acc_;
  double last_vel_err = last_vel_error_;
  double last_pos_err = last_pos_error_;
  double last_int_err = last_int_error_;

  // Output value
  double force;				// Controller output force


  // Step 1: Have the proxy track (and reconverge to) the desired
  // motion.  We use the bandwidth lambda as a general enable switch:
  // Only implement the full proxy behavior if lambda is positive.
  // (Would not make sense if the bandwidth is zero..)
  if (lam > 0.0)
  {
    double pnom;		// Nominal position (for lineariztion)
    double vnom;		// Nominal velocity (for lineariztion)
    double anom;		// Nominal/linearized acceleration value
    double dadp;		// Partial derivative w.r.t. position
    double dadv;		// Partial derivative w.r.t. velocity

    double acc_hi;		// Upper acceleration boundary value
    double acc_lo;		// Lower acceleration boundary value

    // Using trapezoidal integration for the proxy, we need to solve
    // the implicit equation for the acceleration.  To do so, we
    // have to linearize the equation around zero acceleration.  And
    // for this, we need to determine a new (nominal) proxy position
    // and velocity assuming zero acceleration.
    vnom = last_vel_pxy + dt/2 * (last_acc_pxy + 0.0 );
    pnom = last_pos_pxy + dt/2 * (last_vel_pxy + vnom);

    // Calculate the proxy acceleration to track the desired
    // trajectory = desired position/velocity/acceleration.
    // Appropriate to trapezoidal integration, first compute the
    // linearized dynamics at the nominal position/velocity, then
    // solve the solve the implicit equation.  Note the partial
    // derivatives are negative, so the denominator is guaranteed to
    // be positive.
    calcDynamics2ndorder(anom, dadp, dadv, pnom-pos_des, vnom-vel_des, lam, acon);
    acc_pxy = (acc_des + anom) / (1.0 - dadv*dt/2 - dadp*dt2/4);

    // Limit the new proxy position (if a non-zero position range is
    // given).  Calculate the acceleration that would be needed to
    // stop at the upper and lower position limits.  To stop in
    // time, we should never apply more acceleration than the first
    // or less than the second.  Hence saturate the proxy
    // acceleration accordingly.
#if 0
    // Comment out preferred by Stu to avoid parameters pmin/pmax.
    if (pmax - pmin > 0.0)
    {
      // Upper limit.
      calcDynamics2ndorder(anom, dadp, dadv, pnom-pmax, vnom, lam, acon);
      acc_hi = anom / (1.0 - dadv*dt/2 - dadp*dt2/4);

      // Lower limit.
      calcDynamics2ndorder(anom, dadp, dadv, pnom-pmin, vnom, lam, acon);
      acc_lo = anom / (1.0 - dadv*dt/2 - dadp*dt2/4);

      // Saturate between the lower and upper values.
      acc_pxy = std::min(std::max(acc_pxy, acc_lo), acc_hi);
    }
#endif

    // Limit the new proxy velocity (if a velocity limit is given).
    // Calculate the acceleration that would be needed to converge
    // to the upper and lower velocity limit.  To avoid the limits,
    // we should never apply more than the first or less than the
    // second, hence saturate the proxy accordingly.
    if (vlim > 0.0)
    {
      // Upper limit.
      calcDynamics1storder(anom, dadv, vnom-vlim, lam, acon);
      acc_hi = anom / (1.0 - dadv*dt/2);

      // Lower limit.
      calcDynamics1storder(anom, dadv, vnom+vlim, lam, acon);
      acc_lo = anom / (1.0 - dadv*dt/2);

      // Saturate between the lower and upper values.
      acc_pxy = std::min(std::max(acc_pxy, acc_lo), acc_hi);
    }

    // Do not limit the new proxy acceleration to any constant max
    // acceleration value.  If the acceleration is high, it may
    // cause force saturations which Step 3 detects and corrects.
    // As a result, accelerations can be higher for joint
    // configurations with low inertia and vice versa.

    // Finally integrate the proxy over the time step using the
    // (nonzero) computed proxy acceleration.
    vel_pxy = last_vel_pxy + dt/2 * (last_acc_pxy + acc_pxy);
    pos_pxy = last_pos_pxy + dt/2 * (last_vel_pxy + vel_pxy);
  }
  else
  {
    // The proxy dynamics are turned off, so just set it to track
    // the desired exactly.
    acc_pxy = acc_des;
    vel_pxy = vel_des;
    pos_pxy = pos_des;
  }


  // Step 2: Calculate the controller based on the proxy motion.
  // First compute the velocity, position, and integral errors.  Note
  // we do NOT limit the integration or integral error until after the
  // below adjustments!
  vel_err = vel_act - vel_pxy;
  pos_err = pos_act - pos_pxy;
  int_err = last_int_err + dt/2 * (last_pos_err + pos_err);

  // Calculate the controller force.  This includes an acceleration
  // feedforward term (so the actual robot will track the proxy as
  // best possible), a regular PD, and an integral term which is
  // clamped to a maximum value.
  force = mass*acc_pxy - Kd*vel_err - Kp*pos_err - std::min(std::max(Ki*int_err, -Ficl), Ficl);


  // Step 3: If the controller force were to exceed the force limits,
  // adjust the proxy to reduce the required force.  This effectively
  // drags the proxy with the actual when the controller can not
  // create the forces required for tracking.  We use the force
  // limit as an enable switch: only adjust if the limit is positive.
  // (A force limit of zero would make no sense.)
  if (Flim > 0.0)
  {
    double Fpd;		// PD force from the un-adjusted errors
    double Fi;		// Unclamped and un-adjusted integral force

    // Saturate the force to the known force limits.
    force = std::min(std::max(force, -Flim), Flim);

    // Calculate the PD force and the unclamped (unsaturated)
    // integral force which the un-adjusted proxy would provide.
    Fpd = mass*acc_pxy - Kd*vel_err - Kp*pos_err;
    Fi  = - Ki*int_err;

    // If the mass is non-zero, calculate an acceleration-based
    // proxy adjustment.
    if (mass > 0.0)
    {
      double da;		// Acceleration delta (change)

      // Compute the acceleration delta assuming the integral term
      // does not saturate.
      da = (force - Fpd - Fi) / (mass + Kd*dt/2 + Kp*dt2/4 + Ki*dt3/8);

      // Check for clamping/saturation on the adjusted integral
      // force and re-compute the delta appropriately.  There is
      // no need to re-check for clamping after recomputation, as
      // the new delta will only increase and can not undo the
      // saturation.
      if      (Fi+da*Ki*dt3/8 >  Ficl)  da=(force-Fpd-Ficl)/(mass+Kd*dt/2+Kp*dt2/4);
      else if (Fi+da*Ki*dt3/8 < -Ficl)  da=(force-Fpd+Ficl)/(mass+Kd*dt/2+Kp*dt2/4);

      // Adjust the acceleration, velocity, position, and integral
      // states.
      acc_pxy += da;
      vel_pxy += da * dt/2;
      pos_pxy += da * dt2/4;

      vel_err -= da * dt/2;
      pos_err -= da * dt2/4;
      int_err -= da * dt3/8;
    }

    // If the mass is zero and the damping gain is nonzero, we have
    // to adjust the force by shifting the proxy velocity.
    else if (Kd > 0.0)
    {
      double dv;		// Velocity delta (change)

      // Compute the velocity delta assuming the integral term
      // does not saturate.
      dv = (force - Fpd - Fi) / (Kd + Kp*dt/2 + Ki*dt2/4);

      // Check for clamping/saturation on the adjusted integral
      // force and re-compute the delta appropriately.  There is
      // no need to re-check for clamping after recomputation, as
      // the new delta will only increase and can not undo the
      // saturation.
      if      (Fi+dv*Ki*dt2/4 >  Ficl)  dv=(force-Fpd-Ficl)/(Kd+Kp*dt/2);
      else if (Fi+dv*Ki*dt2/4 < -Ficl)  dv=(force-Fpd+Ficl)/(Kd+Kp*dt/2);

      // Adjust the velocity, position, and integral states.  Do
      // not alter the acceleration.
      vel_pxy += dv;
      pos_pxy += dv * dt/2;

      vel_err -= dv;
      pos_err -= dv * dt/2;
      int_err -= dv * dt2/4;
    }

    // If the mass and damping gain are both zero and the position
    // gain is nonzero, we have to adjust the force by shifting the
    // proxy position.
    else if (Kp > 0.0)
    {
      double dp;		// Position delta (change)

      // Compute the velocity delta assuming the integral term
      // does not saturate.
      dp = (force - Fpd - Fi) / (Kp + Ki*dt/2);

      // Check for clamping/saturation on the adjusted integral
      // force and re-compute the delta appropriately.  There is
      // no need to re-check for clamping after recomputation, as
      // the new delta will only increase and can not undo the
      // saturation.
      if      (Fi+dp*Ki*dt/2 >  Ficl)  dp=(force-Fpd-Ficl)/(Kp);
      else if (Fi+dp*Ki*dt/2 < -Ficl)  dp=(force-Fpd+Ficl)/(Kp);

      // Adjust the position and integral states.  Do not alter
      // the acceleration or velocity.
      pos_pxy += dp;

      pos_err -= dp;
      int_err -= dp * dt/2;
    }

    // If the mass, damping, and position gain are all zero, there
    // isn't much we can do...
  }


  // Step 4: Clean up
  // (a) Stop the position error integration (limit the integral error) if
  //     the integrator clamp is in effect.  Note this is safe for Ki==0.
  if      (Ki * int_err >  Ficl)   int_err =  Ficl / Ki;
  else if (Ki * int_err < -Ficl)   int_err = -Ficl / Ki;

  // (b) Remember the state.
  last_proxy_pos_ = pos_pxy;
  last_proxy_vel_ = vel_pxy;
  last_proxy_acc_ = acc_pxy;
  last_vel_error_ = vel_err;
  last_pos_error_ = pos_err;
  last_int_error_ = int_err;

  // (c) Return the controller force.
  return(force);
}

} // namespace
