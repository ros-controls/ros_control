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
#include <vector>
#include <string>

#include <ros/console.h>

#include <hardware_interface/internal/demangle_symbol.h>

namespace hardware_interface
{

// SFINAE workaround, so that we have reflection inside the template functions
template <typename T>
struct check_is_hw_resource_manager {
  // variable definitions for compiler-time logic
  typedef struct {} yes;
  typedef yes no[2];

  // method called if C is a HardwareResourceManager
  template <typename C>
  static yes& testHWRM(typename C::hw_resource_manager*);
 
  // method called if C is not a HardwareResourceManager
  template <typename>
  static no& testHWRM(...);
 
  // check_is_hw_resource_manager<T>::value == true when T is a HardwareResourceManager
  static const bool value = (sizeof(testHWRM<T>(0)) == sizeof(yes));
 
  // method called if C is a HardwareResourceManager
  template <typename C>
  static yes& callCM(typename std::vector<C*>& managers, C* result, typename C::hw_resource_manager*) 
  { 
    std::vector<typename C::hw_resource_manager*> managers_in;
    // we have to typecase back to base class
    for(typename std::vector<C*>::iterator it = managers.begin(); it != managers.end(); ++it)
      managers_in.push_back(static_cast<typename C::hw_resource_manager*>(*it));
    C::concatManagers(managers_in, result); 
  }
 
  // method called if C is not a HardwareResourceManager
  template <typename C>
  static no& callCM(typename std::vector<C*>& managers, C* result, ...) {}

  // calls HardwareResourceManager::concatManagers if C is a HardwareResourceManager
  static const void callConcatManagers(typename std::vector<T*>& managers, T* result) 
  { callCM<T>(managers, result, 0); }
};

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
   * \param hw A pointer to the interface to store
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
    std::string type_name = internal::demangledTypeName<T>();
    std::vector<void*> iface_data = findInterfaceData(type_name);
    std::vector<T*> iface_list;
    for(std::vector<void*>::iterator it = iface_data.begin(); it != iface_data.end(); ++it) {
      T* iface = static_cast<T*>(*it);
      if (!iface)
      {
        ROS_ERROR_STREAM("Failed reconstructing type T = '" << type_name.c_str() <<
                         "'. This should never happen");
        return NULL;
      }
      iface_list.push_back(iface);
    }

    if(iface_data.size() == 0)
      return NULL;

    if(iface_data.size() == 1)
      return iface_list.front();

    // if we're here, we have multiple interfaces, and thus we must construct a new
    // combined interface, or return one already constructed
    T* iface_combo;
    InterfaceMap::iterator it_combo = interfaces_combo_.find(type_name);
    if(it_combo != interfaces_combo_.end()) {
      // there exists a combined interface
      iface_combo = static_cast<T*>(it_combo->second);
    } else {
      // no existing combined interface
      if(check_is_hw_resource_manager<T>::value) {
        // it is a HardwareResourceManager

        // create a new combined interface
        iface_combo = new T; 
        // concat all of the hardware resource managers together
        check_is_hw_resource_manager<T>::callConcatManagers(iface_list, iface_combo);
        // save the combined interface for if this is called again
        interfaces_combo_[type_name] = iface_combo; 
      } else {
        // it is not a HardwareResourceManager
        ROS_ERROR("You cannot register multiple interfaces of the same type which are "
                  "not of type HardwareResourceManager. There is no established protocol "
                  "for combining them.");
        iface_combo = NULL;
      }
    }
    return iface_combo;
  }

  /**
   * \brief Get generic pointers to interfaces with type_name.
   *
   * This is used as a polymorphic lookup for the templated
   * get() call, which can't be virtual.
   * By default, this method returns a list with the only element the interface
   * found in the internal variable interfaces_.
   * Derived classes may return more interfaces.
   *
   * \param type_name The name of the interface types stored.
   * \return List of generic pointers to the interfaces found.
   */
  virtual std::vector<void*> findInterfaceData(std::string type_name)
  {
    std::vector<void*> iface_data;
    InterfaceMap::iterator it = interfaces_.find(type_name);
    if (it != interfaces_.end())
      iface_data.push_back(it->second);
    return iface_data;
  }

protected:
  typedef std::map<std::string, void*> InterfaceMap;
  InterfaceMap interfaces_;
  InterfaceMap interfaces_combo_;
};

} // namespace

#endif // header guard
