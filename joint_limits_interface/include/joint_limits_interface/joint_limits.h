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

#ifndef JOINT_LIMITS_INTERFACE_JOINT_LIMITS_H
#define JOINT_LIMITS_INTERFACE_JOINT_LIMITS_H

namespace joint_limits_interface
{

struct JointLimits
{
  JointLimits()
    : min_position(0.0),
      max_position(0.0),
      max_velocity(0.0),
      max_acceleration(0.0),
      max_jerk(0.0),
      max_effort(0.0),
      has_position_limits(false),
      has_velocity_limits(false),
      has_acceleration_limits(false),
      has_jerk_limits(false),
      has_effort_limits(false),
      angle_wraparound(false)
  {}

public:

  // Helper function for debuggging, not realtime safe
  void print()
  {
    std::cout << "min_position " << min_position << std::endl;
    std::cout << "max_position " << max_position << std::endl;
    std::cout << "max_velocity " << max_velocity << std::endl;
    std::cout << "max_acceleration " << max_acceleration << std::endl;
    std::cout << "max_jerk " << max_jerk << std::endl;
    std::cout << "max_effort " << max_effort << std::endl;

    std::cout << "has_position_limits " << has_position_limits << std::endl;
    std::cout << "has_velocity_limits " << has_velocity_limits << std::endl;
    std::cout << "has_acceleration_limits " << has_acceleration_limits << std::endl;
    std::cout << "has_jerk_limits " << has_jerk_limits << std::endl;
    std::cout << "has_effort_limits " << has_effort_limits << std::endl;
    std::cout << "angle_wraparound " << angle_wraparound << std::endl;
  }

  double min_position;
  double max_position;
  double max_velocity;
  double max_acceleration;
  double max_jerk;
  double max_effort;

  bool   has_position_limits;
  bool   has_velocity_limits;
  bool   has_acceleration_limits;
  bool   has_jerk_limits;
  bool   has_effort_limits;
  bool   angle_wraparound;
};

struct SoftJointLimits
{
  SoftJointLimits()
    : min_position(0.0),
      max_position(0.0),
      k_position(0.0),
      k_velocity(0.0)
  {}

  double min_position;
  double max_position;
  double k_position;
  double k_velocity;
};

}

#endif
