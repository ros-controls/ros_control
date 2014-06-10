///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
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

/// \author Wim Meussen, Adolfo Rodriguez Tsouroukdissian

#ifndef HARDWARE_INTERFACE_INTERFACE_MANAGER_H
#define HARDWARE_INTERFACE_INTERFACE_MANAGER_H

#include <map>
#include <string>

#include <ros/console.h>

#include <hardware_interface/internal/demangle_symbol.h>

namespace hardware_interface
{

class InterfaceManager
{
public:
  /**
   * \brief Register an interface.
   *
   * This associates the name of the type of interface to be registered with
   * the given pointer.
   *
   * \tparam T The interface type
   * \param iface A pointer to the interface to store
   */
  template<class T>
  void registerInterface(T* iface)
  {
    const std::string iface_name = internal::demangledTypeName<T>();
    if (interfaces_.find(iface_name) != interfaces_.end())
    {
      ROS_WARN_STREAM("Replacing previously registered interface '" << iface_name << "'.");
    }
    interfaces_[internal::demangledTypeName<T>()] = iface;
  }

  /**
   * \brief Get an interface.
   *
   * Since this class only stores one interface per type, this returns a
   * pointer to the requested interface type. If the interface type is not
   * registered, it will return \c NULL.
   *
   * \tparam T The interface type
   * \return A pointer to the stored interface of type \c T or \c NULL
   */
  template<class T>
  T* get()
  {
    InterfaceMap::iterator it = interfaces_.find(internal::demangledTypeName<T>());
    if (it == interfaces_.end())
      return NULL;

    T* iface = static_cast<T*>(it->second);
    if (!iface)
    {
      ROS_ERROR_STREAM("Failed reconstructing type T = '" << internal::demangledTypeName<T>().c_str() <<
                       "'. This should never happen");
      return NULL;
    }
    return iface;
  }

protected:
  typedef std::map<std::string, void*> InterfaceMap;
  InterfaceMap interfaces_;
};

} // namespace

#endif // header guard
