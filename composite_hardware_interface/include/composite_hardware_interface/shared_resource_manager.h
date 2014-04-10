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

#ifndef COMPOSITE_HARDWARE_INTERFACE__SHARED_RESOURCE_MANAGER_H
#define COMPOSITE_HARDWARE_INTERFACE__SHARED_RESOURCE_MANAGER_H

#include <boost/shared_ptr.hpp>

namespace composite_hardware_interface
{

/** \brief Resource Cash
 *
 * Collect resources used thru all robot devices.
 */
class SharedResourceManager
{
public:

  /** \brief Return interface by type.
   *
   * \param resource_name Name of the resource
   * \returns True if cash contain resource
   */
  bool exists(const std::string& resource_name) const
  {
    return resources_.count(resource_name);
  }

  /** \brief Return resource by name.
   *
   *  If resource_name matches the key of an element in the cash, the function
   *  returns a pointer to its mapped value.
   *
   *  If resource_name does not match the key of any element in the cash, the
   *  function inserts a new element with that key and returns a pointer to its
   *  mapped value.
   *
   * \param resource_name Name of the resource
   * \returns Pinter to the resource
   */
  template<class T>
    T* get(const std::string& resource_name)
    {
      if(!exists(resource_name))
      {
        resources_[resource_name] = boost::shared_ptr<void>(new T, Deleter<T>());
      }

      T* res = static_cast<T*>(resources_[resource_name].get());
      return res;
    }

private:
  std::map<std::string, boost::shared_ptr<void> > resources_;

  template<class T>
    struct Deleter
    {
      void operator()(void* obj) { delete static_cast<T*>(obj); }
    };
};

}

#endif

