///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
// Copyright (c) 2008, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of PAL Robotics S.L. nor the names of its
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

/// \author Adolfo Rodriguez Tsouroukdissian

#ifndef JOINT_LIMITS_INTERFACE_JOINT_LIMITS_INTERFACE_H
#define JOINT_LIMITS_INTERFACE_JOINT_LIMITS_INTERFACE_H

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>

#include <ros/duration.h>

#include <hardware_interface/internal/resource_manager.h>
#include <hardware_interface/joint_command_interface.h>

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface_exception.h>

namespace joint_limits_interface
{

namespace internal
{

template<typename T>
T saturate(const T val, const T min_val, const T max_val)
{
  return std::min(std::max(val, min_val), max_val);
}

}

/** \brief A handle used to enforce position and velocity limits of a position-controlled joint that does not have
    soft limits. */
class PositionJointSaturationHandle
{
public:
  PositionJointSaturationHandle(const hardware_interface::JointHandle& jh, const JointLimits& limits)
  {
    jh_ = jh;
    limits_ = limits;

    if (limits_.has_position_limits)
    {
      min_pos_limit_ = limits_.min_position;
      max_pos_limit_ = limits_.max_position;
    }
    else
    {
      min_pos_limit_ = -std::numeric_limits<double>::max();
      max_pos_limit_ = std::numeric_limits<double>::max();
    }

    prev_cmd_ = std::numeric_limits<double>::quiet_NaN();
  }

  /** \return Joint name. */
  std::string getName() const {return jh_.getName();}

  /**
   * \brief Enforce position and velocity limits for a joint that is not subject to soft limits.
   *
   * \param period Control period.
   */
  void enforceLimits(const ros::Duration& period)
  {
    if (std::isnan(prev_cmd_))
      prev_cmd_ = jh_.getPosition();

    double min_pos, max_pos;
    if (limits_.has_velocity_limits)
    {
      const double delta_pos = limits_.max_velocity * period.toSec();
      min_pos = std::max(prev_cmd_ - delta_pos, min_pos_limit_);
      max_pos = std::min(prev_cmd_ + delta_pos, max_pos_limit_);
    }
    else
    {
      min_pos = min_pos_limit_;
      max_pos = max_pos_limit_;
    }

    const double cmd = internal::saturate(jh_.getCommand(), min_pos, max_pos);
    jh_.setCommand(cmd);
    prev_cmd_ = cmd;
  }

  /**
   * \brief Reset state, in case of mode switch or e-stop
   */
  void reset(){
    prev_cmd_ = std::numeric_limits<double>::quiet_NaN();
  }

private:
  hardware_interface::JointHandle jh_;
  JointLimits limits_;
  double min_pos_limit_, max_pos_limit_;
  double prev_cmd_;
};

/**
 * \brief A handle used to enforce position and velocity limits of a position-controlled joint.
 *
 * This class implements a very simple position and velocity limits enforcing policy, and tries to impose the least
 * amount of requisites on the underlying hardware platform.
 * This lowers considerably the entry barrier to use it, but also implies some limitations.
 *
 * <b>Requisites</b>
 * - Position (for non-continuous joints) and velocity limits specification.
 * - Soft limits specification. The \c k_velocity parameter is \e not used.
 *
 * <b>Open loop nature</b>
 *
 * Joint position and velocity limits are enforced in an open-loop fashion, that is, the command is checked for
 * validity without relying on the actual position/velocity values.
 *
 * - Actual position values are \e not used because in some platforms there might be a substantial lag
 *   between sending a command and executing it (propagate command to hardware, reach control objective,
 *   read from hardware).
 *
 * - Actual velocity values are \e not used because of the above reason, and because some platforms might not expose
 *   trustworthy velocity measurements, or none at all.
 *
 * The downside of the open loop behavior is that velocity limits will not be enforced when recovering from large
 * position tracking errors. Only the command is guaranteed to comply with the limits specification.
 *
 * \note: This handle type is \e stateful, ie. it stores the previous position command to estimate the command
 * velocity.
 */

// TODO: Leverage %Reflexxes Type II library for acceleration limits handling?
class PositionJointSoftLimitsHandle
{
public:
  PositionJointSoftLimitsHandle()
    : prev_cmd_(std::numeric_limits<double>::quiet_NaN())
  {}

  PositionJointSoftLimitsHandle(const hardware_interface::JointHandle& jh,
                                const JointLimits&                     limits,
                                const SoftJointLimits&                 soft_limits)
    : jh_(jh),
      limits_(limits),
      soft_limits_(soft_limits),
      prev_cmd_(std::numeric_limits<double>::quiet_NaN())
  {
    if (!limits.has_velocity_limits)
    {
      throw JointLimitsInterfaceException("Cannot enforce limits for joint '" + getName() +
                                           "'. It has no velocity limits specification.");
    }
  }

  /** \return Joint name. */
  std::string getName() const {return jh_.getName();}

  /**
   * \brief Enforce position and velocity limits for a joint subject to soft limits.
   *
   * If the joint has no position limits (eg. a continuous joint), only velocity limits will be enforced.
   * \param period Control period.
   */
  void enforceLimits(const ros::Duration& period)
  {
    assert(period.toSec() > 0.0);

    using internal::saturate;

    // Current position
    // TODO: Doc!
    if (std::isnan(prev_cmd_)) {prev_cmd_ = jh_.getPosition();} // Happens only once at initialization
    const double pos = prev_cmd_;

    // Velocity bounds
    double soft_min_vel;
    double soft_max_vel;

    if (limits_.has_position_limits)
    {
      // Velocity bounds depend on the velocity limit and the proximity to the position limit
      soft_min_vel = saturate(-soft_limits_.k_position * (pos - soft_limits_.min_position),
                              -limits_.max_velocity,
                               limits_.max_velocity);

      soft_max_vel = saturate(-soft_limits_.k_position * (pos - soft_limits_.max_position),
                              -limits_.max_velocity,
                               limits_.max_velocity);
    }
    else
    {
      // No position limits, eg. continuous joints
      soft_min_vel = -limits_.max_velocity;
      soft_max_vel =  limits_.max_velocity;
    }

    // Position bounds
    const double dt = period.toSec();
    double pos_low  = pos + soft_min_vel * dt;
    double pos_high = pos + soft_max_vel * dt;

    if (limits_.has_position_limits)
    {
      // This extra measure safeguards against pathological cases, like when the soft limit lies beyond the hard limit
      pos_low  = std::max(pos_low,  limits_.min_position);
      pos_high = std::min(pos_high, limits_.max_position);
    }

    // Saturate position command according to bounds
    const double pos_cmd = saturate(jh_.getCommand(),
                                    pos_low,
                                    pos_high);
    jh_.setCommand(pos_cmd);

    // Cache variables
    prev_cmd_ = jh_.getCommand();
  }

  /**
   * \brief Reset state, in case of mode switch or e-stop
   */
  void reset(){
    prev_cmd_ = std::numeric_limits<double>::quiet_NaN();
  }

private:
  hardware_interface::JointHandle jh_;
  JointLimits limits_;
  SoftJointLimits soft_limits_;

  double prev_cmd_;
};

/** \brief A handle used to enforce position, velocity, and effort limits of an effort-controlled joint that does not
    have soft limits. */
class EffortJointSaturationHandle
{
public:
  EffortJointSaturationHandle(const hardware_interface::JointHandle& jh, const JointLimits& limits)
    : jh_(jh)
    , limits_(limits)
  {
    if (!limits.has_velocity_limits)
    {
      throw JointLimitsInterfaceException("Cannot enforce limits for joint '" + getName() +
                                           "'. It has no velocity limits specification.");
    }
    if (!limits.has_effort_limits)
    {
      throw JointLimitsInterfaceException("Cannot enforce limits for joint '" + getName() +
                                          "'. It has no efforts limits specification.");
    }
  }

  /** \return Joint name. */
  std::string getName() const {return jh_.getName();}

  /**
   * \brief Enforce position, velocity, and effort limits for a joint that is not subject to soft limits.
   */
  void enforceLimits(const ros::Duration& /* period */)
  {
    double min_eff = -limits_.max_effort;
    double max_eff = limits_.max_effort;

    if (limits_.has_position_limits)
    {
      const double pos = jh_.getPosition();
      if (pos < limits_.min_position)
        min_eff = 0;
      else if (pos > limits_.max_position)
        max_eff = 0;
    }

    const double vel = jh_.getVelocity();
    if (vel < -limits_.max_velocity)
      min_eff = 0;
    else if (vel > limits_.max_velocity)
      max_eff = 0;

    jh_.setCommand(internal::saturate(jh_.getCommand(), min_eff, max_eff));
  }

private:
  hardware_interface::JointHandle jh_;
  JointLimits limits_;
};

/** \brief A handle used to enforce position, velocity and effort limits of an effort-controlled joint. */

// TODO: This class is untested!. Update unit tests accordingly.
class EffortJointSoftLimitsHandle
{
public:
  EffortJointSoftLimitsHandle() {}

  EffortJointSoftLimitsHandle(const hardware_interface::JointHandle& jh,
                              const JointLimits&                     limits,
                              const SoftJointLimits&                 soft_limits)
  : jh_(jh),
    limits_(limits),
    soft_limits_(soft_limits)
  {
    if (!limits.has_velocity_limits)
    {
      throw JointLimitsInterfaceException("Cannot enforce limits for joint '" + getName() +
                                           "'. It has no velocity limits specification.");
    }
    if (!limits.has_effort_limits)
    {
      throw JointLimitsInterfaceException("Cannot enforce limits for joint '" + getName() +
                                           "'. It has no effort limits specification.");
    }
  }

  /** \return Joint name. */
  std::string getName() const {return jh_.getName();}

  /**
   * \brief Enforce position, velocity and effort limits for a joint subject to soft limits.
   *
   * If the joint has no position limits (eg. a continuous joint), only velocity and effort limits will be enforced.
   */
  void enforceLimits(const ros::Duration& /*period*/)
  {
    using internal::saturate;

    // Current state
    const double pos = jh_.getPosition();
    const double vel = jh_.getVelocity();

    // Velocity bounds
    double soft_min_vel;
    double soft_max_vel;

    if (limits_.has_position_limits)
    {
      // Velocity bounds depend on the velocity limit and the proximity to the position limit
      soft_min_vel  = saturate(-soft_limits_.k_position * (pos - soft_limits_.min_position),
                               -limits_.max_velocity,
                                limits_.max_velocity);

      soft_max_vel = saturate(-soft_limits_.k_position * (pos - soft_limits_.max_position),
                              -limits_.max_velocity,
                               limits_.max_velocity);
    }
    else
    {
      // No position limits, eg. continuous joints
      soft_min_vel = -limits_.max_velocity;
      soft_max_vel =  limits_.max_velocity;
    }

    // Effort bounds depend on the velocity and effort bounds
    const double soft_min_eff = saturate(-soft_limits_.k_velocity * (vel - soft_min_vel),
                                         -limits_.max_effort,
                                          limits_.max_effort);

    const double soft_max_eff = saturate(-soft_limits_.k_velocity * (vel - soft_max_vel),
                                         -limits_.max_effort,
                                          limits_.max_effort);

    // Saturate effort command according to bounds
    const double eff_cmd = saturate(jh_.getCommand(),
                                    soft_min_eff,
                                    soft_max_eff);
    jh_.setCommand(eff_cmd);
  }

private:
  hardware_interface::JointHandle jh_;
  JointLimits limits_;
  SoftJointLimits soft_limits_;
};


/** \brief A handle used to enforce velocity and acceleration limits of a velocity-controlled joint. */
class VelocityJointSaturationHandle
{
public:
  VelocityJointSaturationHandle () {}

  VelocityJointSaturationHandle(const hardware_interface::JointHandle& jh, const JointLimits& limits)
    : jh_(jh),
      limits_(limits)
  {
    if (!limits.has_velocity_limits)
    {
      throw JointLimitsInterfaceException("Cannot enforce limits for joint '" + getName() +
                                           "'. It has no velocity limits specification.");
    }
  }

  /** \return Joint name. */
  std::string getName() const {return jh_.getName();}

  /**
   * \brief Enforce joint velocity and acceleration limits.
   * \param period Control period.
   */
  void enforceLimits(const ros::Duration& period)
  {
    using internal::saturate;

    // Velocity bounds
    double vel_low;
    double vel_high;

    if (limits_.has_acceleration_limits)
    {
      assert(period.toSec() > 0.0);
      const double vel = jh_.getVelocity();
      const double dt  = period.toSec();

      vel_low  = std::max(vel - limits_.max_acceleration * dt, -limits_.max_velocity);
      vel_high = std::min(vel + limits_.max_acceleration * dt,  limits_.max_velocity);
    }
    else
    {
      vel_low  = -limits_.max_velocity;
      vel_high =  limits_.max_velocity;
    }

    // Saturate velocity command according to limits
    const double vel_cmd = saturate(jh_.getCommand(),
                                    vel_low,
                                    vel_high);
    jh_.setCommand(vel_cmd);
  }

private:
  hardware_interface::JointHandle jh_;
  JointLimits limits_;
};

/** \brief A handle used to enforce position, velocity, and acceleration limits of a velocity-controlled joint. */
class VelocityJointSoftLimitsHandle
{
public:
  VelocityJointSoftLimitsHandle(const hardware_interface::JointHandle& jh, const JointLimits& limits,
                                const SoftJointLimits& soft_limits)
  {
    jh_ = jh;
    limits_ = limits;
    soft_limits_ = soft_limits;
    if (limits.has_velocity_limits)
      max_vel_limit_ = limits.max_velocity;
    else
      max_vel_limit_ = std::numeric_limits<double>::max();
  }

  /** \return Joint name. */
  std::string getName() const {return jh_.getName();}

  /**
   * \brief Enforce position, velocity, and acceleration limits for a velocity-controlled joint subject to soft limits.
   *
   * \param period Control period.
   */
  void enforceLimits(const ros::Duration& period)
  {
    using internal::saturate;

    double min_vel, max_vel;
    if (limits_.has_position_limits)
    {
      // Velocity bounds depend on the velocity limit and the proximity to the position limit.
      const double pos = jh_.getPosition();
      min_vel = saturate(-soft_limits_.k_position * (pos - soft_limits_.min_position),
                         -max_vel_limit_, max_vel_limit_);
      max_vel = saturate(-soft_limits_.k_position * (pos - soft_limits_.max_position),
                         -max_vel_limit_, max_vel_limit_);
    }
    else
    {
      min_vel = -max_vel_limit_;
      max_vel = max_vel_limit_;
    }

    if (limits_.has_acceleration_limits)
    {
      const double vel = jh_.getVelocity();
      const double delta_t = period.toSec();
      min_vel = std::max(vel - limits_.max_acceleration * delta_t, min_vel);
      max_vel = std::min(vel + limits_.max_acceleration * delta_t, max_vel);
    }

    jh_.setCommand(saturate(jh_.getCommand(), min_vel, max_vel));
  }

private:
  hardware_interface::JointHandle jh_;
  JointLimits limits_;
  SoftJointLimits soft_limits_;
  double max_vel_limit_;
};

/**
 * \brief Interface for enforcing joint limits.
 *
 * \tparam HandleType %Handle type. Must implement the following methods:
 *  \code
 *   void enforceLimits();
 *   std::string getName() const;
 *  \endcode
 */
template <class HandleType>
class JointLimitsInterface : public hardware_interface::ResourceManager<HandleType>
{
public:
  HandleType getHandle(const std::string& name)
  {
    // Rethrow exception with a meaningful type
    try
    {
      return this->hardware_interface::ResourceManager<HandleType>::getHandle(name);
    }
    catch(const std::logic_error& e)
    {
      throw JointLimitsInterfaceException(e.what());
    }
  }

  /** \name Real-Time Safe Functions
   *\{*/
  /** \brief Enforce limits for all managed handles. */
  void enforceLimits(const ros::Duration& period)
  {
    typedef typename hardware_interface::ResourceManager<HandleType>::ResourceMap::iterator ItratorType;
    for (ItratorType it = this->resource_map_.begin(); it != this->resource_map_.end(); ++it)
    {
      it->second.enforceLimits(period);
    }
  }
  /*\}*/
};

/** Interface for enforcing limits on a position-controlled joint through saturation. */
class PositionJointSaturationInterface : public JointLimitsInterface<PositionJointSaturationHandle> {
public:
  /** \name Real-Time Safe Functions
   *\{*/
  /** \brief Reset all managed handles. */
  void reset()
  {
    typedef hardware_interface::ResourceManager<PositionJointSaturationHandle>::ResourceMap::iterator ItratorType;
    for (ItratorType it = this->resource_map_.begin(); it != this->resource_map_.end(); ++it)
    {
      it->second.reset();
    }
  }
  /*\}*/
};

/** Interface for enforcing limits on a position-controlled joint with soft position limits. */
class PositionJointSoftLimitsInterface : public JointLimitsInterface<PositionJointSoftLimitsHandle> {
public:
  /** \name Real-Time Safe Functions
   *\{*/
  /** \brief Reset all managed handles. */
  void reset()
  {
    typedef hardware_interface::ResourceManager<PositionJointSoftLimitsHandle>::ResourceMap::iterator ItratorType;
    for (ItratorType it = this->resource_map_.begin(); it != this->resource_map_.end(); ++it)
    {
      it->second.reset();
    }
  }
  /*\}*/
};

/** Interface for enforcing limits on an effort-controlled joint through saturation. */
class EffortJointSaturationInterface : public JointLimitsInterface<EffortJointSaturationHandle> {};

/** Interface for enforcing limits on an effort-controlled joint with soft position limits. */
class EffortJointSoftLimitsInterface : public JointLimitsInterface<EffortJointSoftLimitsHandle> {};

/** Interface for enforcing limits on a velocity-controlled joint through saturation. */
class VelocityJointSaturationInterface : public JointLimitsInterface<VelocityJointSaturationHandle> {};

/** Interface for enforcing limits on a velocity-controlled joint with soft position limits. */
class VelocityJointSoftLimitsInterface : public JointLimitsInterface<VelocityJointSoftLimitsHandle> {};

}

#endif
