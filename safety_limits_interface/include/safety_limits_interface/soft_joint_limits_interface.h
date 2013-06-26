///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
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

/// \author Adolfo Rodriguez Tsouroukdissian

#ifndef SAFETY_LIMITS_INTERFACE_SOFT_JOINT_LIMITS_INTERFACE_H
#define SAFETY_LIMITS_INTERFACE_SOFT_JOINT_LIMITS_INTERFACE_H

#include <algorithm>
#include <cassert>

#include <boost/shared_ptr.hpp>

#include <ros/duration.h>

#include <urdf_interface/joint.h>

#include <hardware_interface/internal/resource_manager.h>
#include <hardware_interface/joint_command_interface.h>

namespace safety_limits_interface
{

namespace internal
{

template<typename T>
T saturate(const T val, const T min_val, const T max_val)
{
  return std::min(std::max(val, min_val), max_val);
}

}


/// An exception related to a \ref SafetyLimitsInterface
// TODO: Update ref above!
class JointLimitsInterfaceException: public std::exception
{
public:
  JointLimitsInterfaceException(const std::string& message)
    : msg(message) {}

  virtual ~JointLimitsInterfaceException() throw() {}

  virtual const char* what() const throw()
  {
    return msg.c_str();
  }

private:
  std::string msg;
};

/**
 * TODO
 */
class JointSoftLimitsHandle
{
public:
  /** \return Joint name. */
  std::string getName() const {return jh_.getName();}

protected:
  typedef boost::shared_ptr<const urdf::Joint> UrdfJointPtr;

  JointSoftLimitsHandle() {}

  // TODO: Missing limits/safety specification throws here, or makes enforceLimits() be a no-op (and print warning)
  JointSoftLimitsHandle(const hardware_interface::JointHandle& jh, UrdfJointPtr urdf_joint)
    : jh_(jh),
      urdf_joint_(urdf_joint)
  {
    if (!urdf_joint)
    {
      throw JointLimitsInterfaceException("Cannot enforce limits for joint '" + getName() + "'. URDF joint pointer is null.");
    }
    if (!urdf_joint_->limits)
    {
      throw JointLimitsInterfaceException("Cannot enforce limits for joint '" + getName() +
                                           "'. URDF joint does not have limits specification.");
    }
  }

  hardware_interface::JointHandle jh_;
  boost::shared_ptr<const urdf::Joint> urdf_joint_;
};

/** \brief A handle used to enforce position and velocity limits of a position-controlled joint.
 * TODO
 */

// TODO: Drop explicit dependency on URDF?

class PositionJointSoftLimitsHandle : public JointSoftLimitsHandle
{
public:
  PositionJointSoftLimitsHandle() {}

  PositionJointSoftLimitsHandle(const hardware_interface::JointHandle& jh, UrdfJointPtr urdf_joint)
    : JointSoftLimitsHandle(jh, urdf_joint)
  {
    if (!urdf_joint_->safety)
    {
      throw JointLimitsInterfaceException("Cannot enforce limits for joint '" + getName() +
                                           "'. URDF joint does not have safety specification.");
    }
  }

  /**
   * \brief Enforce joint limits. TODO
   * \param period Control period
   */
  void enforceLimits(const ros::Duration& period)
  {
    assert(period.toSec() > 0.0);
    assert(urdf_joint_ && urdf_joint_->limits && urdf_joint_->safety);

    using internal::saturate;

    // Current position
    const double pos = jh_.getPosition();

    // Velocity bounds
    double vel_low  = -urdf_joint_->limits->velocity;
    double vel_high =  urdf_joint_->limits->velocity;

    const bool has_pos_bounds = urdf_joint_->type == urdf::Joint::REVOLUTE || urdf_joint_->type == urdf::Joint::PRISMATIC;
    if (has_pos_bounds)
    {
      // Velocity bounds depend on the velocity limit and the proximity to the position limit
      const double pos_soft_min = urdf_joint_->safety->soft_lower_limit;
      const double pos_soft_max = urdf_joint_->safety->soft_upper_limit;
      const double vel_max      = urdf_joint_->limits->velocity;
      const double kp           = urdf_joint_->safety->k_position;

      vel_low  = saturate(-kp * (pos - pos_soft_min), -vel_max, vel_max);
      vel_high = saturate(-kp * (pos - pos_soft_max), -vel_max, vel_max);
    }

    // Position bounds
    const double dt = period.toSec();
    double pos_low  = pos + vel_low  * dt;
    double pos_high = pos + vel_high * dt;

    if (has_pos_bounds)
    {
      // This extra measure safeguards against pathological cases, like when the soft limit lies beyond the hard limit
      pos_low = std::max(pos_low, urdf_joint_->limits->lower);
      pos_high = std::min(pos_high, urdf_joint_->limits->upper);
    }

    // Saturate position command according to bounds
    const double pos_cmd = saturate(jh_.getCommand(), pos_low, pos_high);
    jh_.setCommand(pos_cmd);
  }
};

/**
 * TODO
 */
class EffortJointSoftLimitsHandle : public JointSoftLimitsHandle
{
public:
  EffortJointSoftLimitsHandle() {}

  EffortJointSoftLimitsHandle(const hardware_interface::JointHandle& jh, UrdfJointPtr urdf_joint)
    : JointSoftLimitsHandle(jh, urdf_joint)
  {
    if (!urdf_joint_->safety)
    {
      throw JointLimitsInterfaceException("Cannot enforce limits for joint '" + getName() +
                                           "'. URDF joint does not have safety specification.");
    }
  }

  /**
   * \brief Enforce joint limits. If velocity or
   * \param period Control period
   */
  void enforceLimits(const ros::Duration& /*period*/)
  {
    assert(urdf_joint_ && urdf_joint_->limits && urdf_joint_->safety);

    using internal::saturate;

    // Current state
    const double pos = jh_.getPosition();
    const double vel = jh_.getVelocity();

    // Velocity bounds
    double vel_low  = -urdf_joint_->limits->velocity;
    double vel_high =  urdf_joint_->limits->velocity;

    if (urdf_joint_->type == urdf::Joint::REVOLUTE || urdf_joint_->type == urdf::Joint::PRISMATIC)
    {
      // Velocity bounds depend on the velocity limit and the proximity to the position limit
      const double pos_soft_max = urdf_joint_->safety->soft_upper_limit;
      const double pos_soft_min = urdf_joint_->safety->soft_lower_limit;
      const double vel_max      = urdf_joint_->limits->velocity;
      const double kp           = urdf_joint_->safety->k_position;

      vel_high = saturate(-kp * (pos - pos_soft_max), -vel_max, vel_max);
      vel_low  = saturate(-kp * (pos - pos_soft_min), -vel_max, vel_max);
    }

    // Effort bounds
    const double eff_max = urdf_joint_->limits->effort;
    const double kv      = urdf_joint_->safety->k_velocity;

    const double eff_high = saturate(-kv * (vel - vel_high), -eff_max, eff_max);
    const double eff_low  = saturate(-kv * (vel - vel_low), -eff_max, eff_max);

    // Saturate effort command according to bounds
    const double eff_cmd = saturate(jh_.getCommand(), eff_low, eff_high);
    jh_.setCommand(eff_cmd);
  }
};

/**
 * TODO
 */
class VelocityJointSaturationHandle : public JointSoftLimitsHandle
{
public:
  VelocityJointSaturationHandle () {}

  VelocityJointSaturationHandle(const hardware_interface::JointHandle& jh, UrdfJointPtr urdf_joint)
    : JointSoftLimitsHandle(jh, urdf_joint)
  {}

  /**
   * \brief Enforce joint limits. If velocity or
   * \param period Control period
   */
  void enforceLimits(const ros::Duration& /*period*/)
  {
    assert(urdf_joint_ && urdf_joint_->limits);

    using internal::saturate;

    // Saturate velocity command according to limits
    const double vel_max = urdf_joint_->limits->velocity;
    const double vel_cmd = saturate(jh_.getCommand(), -vel_max, vel_max);
    jh_.setCommand(vel_cmd);
  }
};

/** \brief TODO
 */
template <class HandleType>
class JointLimitsInterface : public hardware_interface::ResourceManager<HandleType>
{
public:
  HandleType getHandle(const std::string& name)
  {
    // Rethrow exception with a meanungful type
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

/** Interface for enforcing limits on a position-controlled joint with soft position limits. */
class PositionJointSoftLimitsInterface : public JointLimitsInterface<PositionJointSoftLimitsHandle> {};

/** Interface for enforcing limits on an effort-controlled joint with soft position limits. */
class EffortJointSoftLimitsInterface : public JointLimitsInterface<EffortJointSoftLimitsHandle> {};

/** Interface for enforcing limits on a velocity-controlled joint through saturation. */
class VelocityJointSaturationInterface : public JointLimitsInterface<VelocityJointSaturationHandle> {};

}

#endif
