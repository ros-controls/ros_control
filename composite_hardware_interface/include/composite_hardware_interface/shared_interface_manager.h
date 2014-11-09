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

  /** \return Vector of resource names registered to interface \ref TIface. */
  template<class TIface>
    std::vector<std::string> getNames() const
    {
      if(manager_.get<TIface>() == NULL)
      {
        return std::vector<std::string>();
      }

      return manager_.get<TIface>()->getNames();
    }

  /**
   * \brief Register a new resource at interface \ref TIface.
   *
   * If interface with type \ref TIface does not exists, the new one will be created and registered in the manger.
   * If the resource name already exists, the previously stored resource value will be replaced with \e val.
   *
   * \param handle Resource value. Its type should implement a <tt>std::string getName()</tt> method.
   */
  template<class TIface, class ResourceHandle>
    bool registerHandle(const ResourceHandle& handle)
    {
      TIface* iface = manager_.get<TIface>();

      if (iface == NULL)
      {
        iface = new TIface();
        interfaces_.push_back(static_cast<hardware_interface::HardwareInterface*>(iface));
        manager_.registerInterface(iface);
      }

      iface->registerHandle(handle);

      if(registered_cb_)
      {
        registered_cb_(handle.getName());
      }
    }

  /**
   * \brief Get a resource handle by name from interface \ref TIface.
   * \param name Resource name.
   * \param handle Returns resource associated to \e name. If the resource name is not found, an exception is thrown.
   */
  template<class TIface, class ResourceHandle>
    void getHandle(const std::string& name, ResourceHandle& handle)
    {
      if(manager_.get<TIface>() == NULL)
      {
        throw std::logic_error("Could not find resource '" + name + "' in '" +
                               hardware_interface::internal::demangledTypeName(*this) + "'.");
      }

      handle = manager_.get<TIface>()->getHandle(name);
    }

  /** \brief Set callback function that called when a new handle registered. */
  void setRegisteredCB(boost::function<void(const std::string &res_name)> register_cb)
  {
    registered_cb_ = register_cb;
  }

private:
  hardware_interface::RobotHW& manager_;
  boost::ptr_vector<hardware_interface::HardwareInterface> interfaces_;
  boost::function<void(const std::string &res_name)> registered_cb_;
};

}

#endif

