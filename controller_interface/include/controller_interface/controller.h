///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012, hiDOF INC.
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

/*
 * Author: Wim Meeussen
 */

#pragma once


#include <controller_interface/controller_base.h>
#include <hardware_interface/internal/demangle_symbol.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/hardware_interface.h>
#include <ros/ros.h>


namespace controller_interface
{

/** \brief %Controller with a specific hardware interface
 *
 * \tparam T The hardware interface type used by this controller. This enforces
 * semantic compatibility between the controller and the hardware it's meant to
 * control.
 */
template <class T>
class Controller: public virtual ControllerBase
{
public:

  /** \brief The init function is called to initialize the controller from a
   * non-realtime thread with a pointer to the hardware interface, itself,
   * instead of a pointer to a RobotHW.
   *
   * \param hw The specific hardware interface used by this controller.
   *
   * \param controller_nh A NodeHandle in the namespace from which the controller
   * should read its configuration, and where it should set up its ROS
   * interface.
   *
   * \returns True if initialization was successful and the controller
   * is ready to be started.
   */
  virtual bool init(T* /*hw*/, ros::NodeHandle& /*controller_nh*/) {return true;};

  /** \brief The init function is called to initialize the controller from a
   * non-realtime thread with a pointer to the hardware interface, itself,
   * instead of a pointer to a RobotHW.
   *
   * \param hw The specific hardware interface used by this controller.
   *
   * \param root_nh A NodeHandle in the root of the controller manager namespace.
   * This is where the ROS interfaces are setup (publishers, subscribers, services).
   *
   * \param controller_nh A NodeHandle in the namespace of the controller.
   * This is where the controller-specific configuration resides.
   *
   * \returns True if initialization was successful and the controller
   * is ready to be started.
   */
  virtual bool init(T* /*hw*/, ros::NodeHandle& /*root_nh*/, ros::NodeHandle& /*controller_nh*/) {return true;};


protected:
  /** \brief Initialize the controller from a RobotHW pointer
   *
   * This calls \ref init with the hardware interface for this controller if it
   * can extract the correct interface from \c robot_hw.
   *
   */
  bool initRequest(hardware_interface::RobotHW* robot_hw,
                   ros::NodeHandle&             root_nh,
                   ros::NodeHandle&             controller_nh,
                   ClaimedResources&            claimed_resources) override
  {
    // check if construction finished cleanly
    if (state_ != CONSTRUCTED){
      ROS_ERROR("Cannot initialize this controller because it failed to be constructed");
      return false;
    }

    // get a pointer to the hardware interface
    T* hw = robot_hw->get<T>();
    if (!hw)
    {
      ROS_ERROR("This controller requires a hardware interface of type '%s'."
                " Make sure this is registered in the hardware_interface::RobotHW class.",
                getHardwareInterfaceType().c_str());
      return false;
    }

    // return which resources are claimed by this controller
    hw->clearClaims();
    if (!init(hw, controller_nh) || !init(hw, root_nh, controller_nh))
    {
      ROS_ERROR("Failed to initialize the controller");
      return false;
    }
    hardware_interface::InterfaceResources iface_res(getHardwareInterfaceType(), hw->getClaims());
    claimed_resources.assign(1, iface_res);
    hw->clearClaims();

    // success
    state_ = INITIALIZED;
    return true;
  }

  /// Get the name of this controller's hardware interface type
  std::string getHardwareInterfaceType() const
  {
    return hardware_interface::internal::demangledTypeName<T>();
  }
};

}
