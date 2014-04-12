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

#ifndef COMPOSITE_CONTROLLER_INTERFACE__INTERNAL__COMPOSITE_HARDWARE_INTERFACE_H
#define COMPOSITE_CONTROLLER_INTERFACE__INTERNAL__COMPOSITE_HARDWARE_INTERFACE_H

#include <controller_interface/controller.h>
#include <boost/preprocessor/repetition.hpp>
#include <boost/range/algorithm.hpp>

namespace composite_controller_interface
{

namespace internal
{

/** \brief Abstract Composite Hardware Interface
 *
 */
class CompositeHardwareInterface
{
public:

  virtual ~CompositeHardwareInterface() { }

  /// Clear the resources this interface is claiming
  void clearClaims()
  {
    boost::range::for_each(sub_ifaces_, std::mem_fun(&hardware_interface::HardwareInterface::clearClaims));
  }

  /// Get the list of resources this interface is currently claiming
  std::set<std::string> getClaims() const
  {
    std::set<std::string> claims;
    for (InterfaceVector::const_iterator it = sub_ifaces_.begin(); it != sub_ifaces_.end(); ++it)
    {
      boost::range::copy((*it)->getClaims(), std::inserter(claims, claims.end()));
    }
    return claims;
  }

protected:

  /** \brief Add hardware interface pointer with type \ref T to sub interfaces cash.
   *
   * \param ifaces Source interface manager
   * \returns True if \ref ifaces contains interface with a type \ref T
   *
   */
  template<class T>
    bool registerSubInterface(hardware_interface::InterfaceManager* ifaces)
    {
      T* hw = ifaces->get<T>();
      return hw ? sub_ifaces_.push_back(hw), true : false;
    }

  typedef std::vector<hardware_interface::HardwareInterface*> InterfaceVector;
  InterfaceVector sub_ifaces_;
};

//TODO: Use variadic template when C++11 will be available
#define COMPOSITE_CONTROLLER_INTERFACE__COMPOSITE_HW(z, n, _)                   \
  template<BOOST_PP_ENUM_PARAMS(n, class T)>                                    \
    class HardwareInterface##n : public CompositeHardwareInterface              \
    {                                                                           \
    public:                                                                     \
      bool init(hardware_interface::RobotHW* robot_hw)                          \
      {                                                                         \
        sub_ifaces_.clear();                                                    \
        BOOST_PP_REPEAT(n, COMPOSITE_CONTROLLER_INTERFACE__HW_REG_MACRO, ~)     \
        return true;                                                            \
      }                                                                         \
                                                                                \
      BOOST_PP_REPEAT(n, COMPOSITE_CONTROLLER_INTERFACE__HW_GET_FUNC_MACRO, ~)  \
    };

#define COMPOSITE_CONTROLLER_INTERFACE__HW_REG_MACRO(z, i, _) \
  if(!registerSubInterface<T##i>(robot_hw)) return false;

#define COMPOSITE_CONTROLLER_INTERFACE__HW_GET_FUNC_MACRO(z, i, _) \
  T##i* get##i() { return static_cast<T##i *>(sub_ifaces_[i]); }

BOOST_PP_REPEAT_FROM_TO(2, 7, COMPOSITE_CONTROLLER_INTERFACE__COMPOSITE_HW, ~)

}
}

#endif

