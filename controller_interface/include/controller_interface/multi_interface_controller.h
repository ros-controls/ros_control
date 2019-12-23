///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2015, PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the names of PAL Robotics S.L. nor the names of its
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

/** \author Mike Purvis */

#ifndef CONTROLLER_INTERFACE_MULTI_INTERFACE_CONTROLLER_H
#define CONTROLLER_INTERFACE_MULTI_INTERFACE_CONTROLLER_H

// TODO: Decide in the future if/when to deprecate MultiInterfaceController.
// #warning MultiInterfaceController is deprecated. Please use Controller instead.

#include <controller_interface/controller.h>
#include <controller_interface/internal/robothw_interfaces.h>
#include <ros/ros.h>

namespace controller_interface
{

/**
 * \brief Adapter class providing the old MultiInterfaceController API.
 */
template <class... Interfaces>
class MultiInterfaceController: public Controller<Interfaces...>
{
public:
  using Controller<Interfaces...>::Controller;

  /** \name Non Real-Time Safe Functions
   *\{*/

  virtual bool init(hardware_interface::RobotHW* /*robot_hw*/,
                    ros::NodeHandle& /*controller_nh*/)
  {
    return true;
  }

  virtual bool init(hardware_interface::RobotHW* /*robot_hw*/,
                    ros::NodeHandle& /*root_nh*/,
                    ros::NodeHandle& /*controller_nh*/)
  {
    return true;
  }

private:
  /**
   * \brief Overrides the new-API init function in the base class,
   * and calls through to the virtual method with the old API.
   */
  bool init(Interfaces*... interfaces,
            ros::NodeHandle& controller_nh) override
  {
    internal::populateInterfaces(&robot_hw_ctrl_, interfaces...);
    return init(&robot_hw_ctrl_, controller_nh);
  }

  /**
   * \brief Overrides the new-API init function in the base class,
   * and calls through to the virtual method with the old API.
   */
  bool init(Interfaces*... interfaces,
            ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh) override
  {
    internal::populateInterfaces(&robot_hw_ctrl_, interfaces...);
    return init(&robot_hw_ctrl_, root_nh, controller_nh);
  }

  /**
   * Robot hardware abstraction containing only the subset of interfaces requested by the controller.
   */
  hardware_interface::RobotHW robot_hw_ctrl_;
};

} // namespace

#endif
