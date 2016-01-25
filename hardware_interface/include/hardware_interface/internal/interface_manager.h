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

/// \author Wim Meussen, Adolfo Rodriguez Tsouroukdissian, Kelsey P. Hawkins, Toni Oliver

#ifndef HARDWARE_INTERFACE_INTERFACE_MANAGER_H
#define HARDWARE_INTERFACE_INTERFACE_MANAGER_H

#include <map>
#include <string>
#include <vector>
#include <boost/ptr_container/ptr_vector.hpp>

#include <ros/console.h>

#include <hardware_interface/internal/demangle_symbol.h>
#include <hardware_interface/internal/resource_manager.h>

namespace hardware_interface
{
// SFINAE workaround, so that we have reflection inside the template functions
template <typename T>
struct CheckIsResourceManager {
  // variable definitions for compiler-time logic
  typedef char yes[1];
  typedef char no[2];

  // method called if C is a ResourceManager
  template <typename C>
  static yes& testRM(typename C::resource_manager_type*);

  // method called if C is not a ResourceManager
  template <typename>
  static no& testRM(...);

  // CheckIsResourceManager<T>::value == true when T is a ResourceManager
  static const bool value = (sizeof(testRM<T>(0)) == sizeof(yes));

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

  // method called if C is not a ResourceManager
  template <typename C>
  static no& callCM(typename std::vector<C*>& managers, C* result, ...) {}

  // calls ResourceManager::concatManagers if C is a ResourceManager
  static const void callConcatManagers(typename std::vector<T*>& managers, T* result)
  { callCM<T>(managers, result, 0); }


  // method called if C is a ResourceManager
  template <typename C>
  static std::vector<std::string> callGR(C* iface, typename C::resource_manager_type*)
  {
    return iface->getNames();
  }

  // method called if C is not a ResourceManager
  template <typename C>
  static std::vector<std::string> callGR(T* iface, ...) {}

  // calls ResourceManager::concatManagers if C is a ResourceManager
  static std::vector<std::string> callGetResources(T* iface)
  { return callGR<T>(iface, 0); }
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
    interfaces_[iface_name] = iface;

    std::vector<std::string> resources;
    if(CheckIsResourceManager<T>::value)
    {
      // it is a ResourceManager. Get the names of the resources
      resources = CheckIsResourceManager<T>::callGetResources(iface);
    }
    resources_[iface_name] = resources;
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
    if(it_combo != interfaces_combo_.end() &&
        num_ifaces_registered_[type_name] == iface_list.size()) {
      // there exists a combined interface with the same number of interfaces combined
      // (since you cannot unregister interfaces, this will be guaranteed to be the
      //  same interfaces from previous calls)
      iface_combo = static_cast<T*>(it_combo->second);
    } else {
      // no existing combined interface
      if(CheckIsResourceManager<T>::value) {
        // it is a ResourceManager

        // create a new combined interface
        iface_combo = new T;
        // save the new interface pointer to allow for its correct destruction
        interface_destruction_list_.push_back(reinterpret_cast<ResourceManagerBase*>(iface_combo));
        // concat all of the resource managers together
        CheckIsResourceManager<T>::callConcatManagers(iface_list, iface_combo);
        // save the combined interface for if this is called again
        interfaces_combo_[type_name] = iface_combo;
        num_ifaces_registered_[type_name] = iface_list.size();
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

  /** \return Vector of interface names registered to this instance. */
  std::vector<std::string> getNames() const
  {
    std::vector<std::string> out;
    out.reserve(interfaces_.size());
    for(InterfaceMap::const_iterator it = interfaces_.begin(); it != interfaces_.end(); ++it)
    {
      out.push_back(it->first);
    }
    return out;
  }

  /**
   * \brief Get the resource names registered to an interface, specified by type
   * (as this class only stores one interface per type)
   *
   * \param iface_type A string with the demangled type name of the interface
   * \return A vector of resource names registered to this interface
   */
  std::vector<std::string> getInterfaceResources(std::string iface_type) const
  {
    std::vector<std::string> out;
    ResourceMap::const_iterator it = resources_.find(iface_type);
    if(it != resources_.end())
    {
      out = it->second;
    }
    return out;
  }

protected:
  typedef std::map<std::string, void*> InterfaceMap;
  typedef std::vector<InterfaceManager*> InterfaceManagerVector;
  typedef std::map<std::string, size_t> SizeMap;
  typedef std::map<std::string, std::vector<std::string> > ResourceMap;

  InterfaceMap interfaces_;
  InterfaceMap interfaces_combo_;
  InterfaceManagerVector interface_managers_;
  SizeMap num_ifaces_registered_;
  boost::ptr_vector<ResourceManagerBase> interface_destruction_list_;
  /// This will allow us to check the resources based on the demangled type name of the interface
  ResourceMap resources_;
};

} // namespace

#endif // header guard
