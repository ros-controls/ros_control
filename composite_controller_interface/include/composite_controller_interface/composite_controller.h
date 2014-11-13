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

#ifndef COMPOSITE_CONTROLLER_INTERFACE__COMPOSITE_CONTROLLER_H
#define COMPOSITE_CONTROLLER_INTERFACE__COMPOSITE_CONTROLLER_H


#include <ros/ros.h>
#include <ros/console.h>
#include <controller_interface/controller.h>
#include <composite_controller_interface/internal/composite_controller_base.h>
#include <boost/preprocessor/repetition.hpp>


namespace composite_controller_interface
{

//TODO: Use variadic template when C++11 will be available
#define COMPOSITE_CONTROLLER_INTERFACE__COMPOSITE_CONTROLLER(z, n, _)                                                     \
  template<BOOST_PP_ENUM_PARAMS(n, class T)>                                                                              \
    class CompositeController##n :                                                                                        \
        public internal::CompositeControllerBase<internal::HardwareInterface##n <BOOST_PP_ENUM_PARAMS(n, T)> >            \
    {                                                                                                                     \
    public:                                                                                                               \
      virtual ~CompositeController##n() { }                                                                               \
                                                                                                                          \
      virtual bool init(BOOST_PP_ENUM_BINARY_PARAMS(n, T, *hw), ros::NodeHandle &controller_nh)                           \
      { return true; }                                                                                                    \
                                                                                                                          \
      virtual bool init(BOOST_PP_ENUM_BINARY_PARAMS(n, T, *hw), ros::NodeHandle& root_nh, ros::NodeHandle &controller_nh) \
      { return true; }                                                                                                    \
                                                                                                                          \
    private:                                                                                                              \
      typedef internal::HardwareInterface##n <BOOST_PP_ENUM_PARAMS(n, T)> TypeHW;                                         \
                                                                                                                          \
      bool init(TypeHW* hw, ros::NodeHandle &controller_nh)                                                               \
      {                                                                                                                   \
        return init(BOOST_PP_REPEAT(n,COMPOSITE_CONTROLLER_INTERFACE__GET_HW_MACRO,~) controller_nh);                     \
      }                                                                                                                   \
                                                                                                                          \
      bool init(TypeHW* hw, ros::NodeHandle& root_nh, ros::NodeHandle &controller_nh)                                     \
      {                                                                                                                   \
        return init(BOOST_PP_REPEAT(n,COMPOSITE_CONTROLLER_INTERFACE__GET_HW_MACRO,~) root_nh, controller_nh);            \
      }                                                                                                                   \
    };

#define COMPOSITE_CONTROLLER_INTERFACE__GET_HW_MACRO(z, i, _)  hw->get##i(),

/** \brief Controller class with a composite hardware interface
 *
 * \tparam T1 - Tn The composite hardware interfaces types used by this controller.
 * This enforces semantic compatibility between the controller and the hardware it's
 * meant to control.
 */
BOOST_PP_REPEAT_FROM_TO(2, 7, COMPOSITE_CONTROLLER_INTERFACE__COMPOSITE_CONTROLLER, ~)

}

#endif
