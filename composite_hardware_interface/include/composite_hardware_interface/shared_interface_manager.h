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

#ifndef COMPOSITE_HARDWARE_INTERFACE__SHARED_INTERFACE_MANAGER_H
#define COMPOSITE_HARDWARE_INTERFACE__SHARED_INTERFACE_MANAGER_H

#include <hardware_interface/robot_hw.h>
#include <boost/ptr_container/ptr_vector.hpp>

namespace composite_hardware_interface
{

/** \brief Interface Cash
 *
 * Collect and register interfaces used thru all robot devices.
 *
 */
class SharedInterfaceManager
{
public:

  SharedInterfaceManager(hardware_interface::RobotHW& manager) :
      manager_(manager)
  {
  }

  /** \brief Return interface by type.
   *
   *  If interface with type \ref T registered in the manager, the function
   *  returns a pointer to its mapped value.
   *
   *  If interface with type \ref T does not registered in the manager, the
   *  function insert a new interface with that type to the cash, register
   *  it in the manger and returns a pointer to its mapped value.
   *
   * \returns Pointer to the interface
   */
  template<class T>
    T* get()
    {
      T* iface = manager_.get<T>();
      if (iface == NULL)
      {
        iface = new T();
        interfaces_.push_back(static_cast<hardware_interface::HardwareInterface*>(iface));
        manager_.registerInterface(iface);
      }
      return iface;
    }

private:
  hardware_interface::RobotHW& manager_;
  boost::ptr_vector<hardware_interface::HardwareInterface> interfaces_;
};

}

#endif

