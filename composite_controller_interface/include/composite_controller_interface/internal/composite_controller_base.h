/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2014, Igor Kalevatykh, Bauman Moscow State Technical University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the Bauman Moscow State Technical University,
 *      nor the names of its contributors may be used to endorse or promote
 *      products derived from this software without specific prior written
 *      permission.
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

/// \author Igor Kalevatykh <kalevatykhia@gmail.com>

#ifndef COMPOSITE_CONTROLLER_INTERFACE__INTERNAL__COMPOSITE_CONTROLLER_BASE_H
#define COMPOSITE_CONTROLLER_INTERFACE__INTERNAL__COMPOSITE_CONTROLLER_BASE_H

#include <ros/ros.h>
#include <ros/console.h>
#include <controller_interface/controller.h>
#include <composite_controller_interface/internal/composite_hardware_interface.h>

namespace composite_controller_interface
{

namespace internal
{

/** \brief Controller base class with a composite hardware interface
 *
 * \tparam T The COMPOSITE hardware interface type used by this controller. This
 * enforces semantic compatibility between the controller and the hardware it's
 * meant to control.
 */
template<class T>
  class CompositeControllerBase : public controller_interface::Controller<T>
  {
  public:

    virtual ~CompositeControllerBase()
    {
    }

    /** \brief Initialize the controller from a RobotHW pointer
     *
     * This try extract from \c robot_hw all interfaces that composing \ref T.
     *
     */
    virtual bool initRequest(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                             ros::NodeHandle &controller_nh, std::set<std::string> &claimed_resources)
    {
      if (!composite_hw_.init(robot_hw))
      {
        ROS_ERROR("This controller requires a hardware interface of type '%s'."
                  " Make sure all sub interfaces is registered in the hardware_interface::RobotHW class.",
                  controller_interface::Controller<T>::getHardwareInterfaceType().c_str());
        return false;
      }

      hardware_interface::RobotHW robot_hw_tmp;
      robot_hw_tmp.registerInterface(&composite_hw_);

      return controller_interface::Controller<T>::initRequest(&robot_hw_tmp, root_nh, controller_nh, claimed_resources);
    }

  private:
    T composite_hw_;
  };

}

}

#endif

