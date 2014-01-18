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

/// \author Wim Meussen, Adolfo Rodriguez Tsouroukdissian, Kelsey P. Hawkins

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
struct CheckIsResourceManager {
  // variable definitions for compiler-time logic
  typedef struct {} yes;
  typedef yes no[2];

  // method called if C is a ResourceManager
  template <typename C>
  static yes& testHWRM(typename C::resource_manager_type*);
 
  // method called if C is not a ResourceManager
  template <typename>
  static no& testHWRM(...);
 
  // CheckIsResourceManager<T>::value == true when T is a ResourceManager
  static const bool value = (sizeof(testHWRM<T>(0)) == sizeof(yes));
 
  // method called if C is a ResourceManager
  template <typename C>
  static yes& callCM(typename std::vector<C*>& managers, C* result, typename C::resource_manager_type*) 
  { 
    std::vector<typename C::resource_manager_type*> managers_in;
    // we have to typecase back to base class
    for(typename std::vector<C*>::iterator it = managers.begin(); it != managers.end(); ++it)
      managers_in.push_back(static_cast<typename C::resource_manager_type*>(*it));
    C::concatManagers(managers_in, result); 
  }
 
  // method called if C is not a HardwareResourceManager
  template <typename C>
  static no& callCM(typename std::vector<C*>& managers, C* result, ...) {}

  // calls ResourceManager::concatManagers if C is a ResourceManager
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

  void registerInterfaceManager(InterfaceManager* iface_man)
  {
    interface_managers_.push_back(iface_man);
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
    std::vector<T*> iface_list;

    // look for interfaces registered here
    InterfaceMap::iterator it = interfaces_.find(type_name);
    if (it != interfaces_.end()) {
      T* iface = static_cast<T*>(it->second);
      if (!iface) {
        ROS_ERROR_STREAM("Failed reconstructing type T = '" << type_name.c_str() <<
                         "'. This should never happen");
        return NULL;
      }
      iface_list.push_back(iface);
    }

    // look for interfaces registered in the registered hardware
    for(InterfaceManagerVector::iterator it = interface_managers_.begin(); 
        it != interface_managers_.end(); ++it) {
      T* iface = (*it)->get<T>();
      if (iface) 
        iface_list.push_back(iface);
    }

    if(iface_list.size() == 0)
      return NULL;

    if(iface_list.size() == 1)
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
      if(CheckIsResourceManager<T>::value) {
        // it is a ResourceManager

        // create a new combined interface (FIXME: never deallocated)
        iface_combo = new T; 
        // concat all of the resource managers together
        CheckIsResourceManager<T>::callConcatManagers(iface_list, iface_combo);
        // save the combined interface for if this is called again
        interfaces_combo_[type_name] = iface_combo; 
      } else {
        // it is not a ResourceManager
        ROS_ERROR("You cannot register multiple interfaces of the same type which are "
                  "not of type ResourceManager. There is no established protocol "
                  "for combining them.");
        iface_combo = NULL;
      }
    }
    return iface_combo;
  }

protected:
  typedef std::map<std::string, void*> InterfaceMap;
  typedef std::vector<InterfaceManager*> InterfaceManagerVector;

  InterfaceMap interfaces_;
  InterfaceMap interfaces_combo_;
  InterfaceManagerVector interface_managers_;
};

} // namespace

#endif // header guard
