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

#pragma once


#include <map>
#include <string>
#include <vector>

#include <ros/console.h>

#include <hardware_interface/internal/demangle_symbol.h>
#include <hardware_interface/internal/resource_manager.h>

namespace hardware_interface
{

namespace internal
{

// SFINAE workaround, so that we have reflection inside the template functions
template <typename T>
struct CheckIsResourceManager {
  // method called if C is a ResourceManager
  template <typename C>
  static void callCM(typename std::vector<C*>& managers, C* result, typename C::resource_manager_type*)
  {
    // We have to typecast back to base class
    std::vector<typename C::resource_manager_type*> managers_in(managers.begin(), managers.end());
    C::concatManagers(managers_in, result);
  }

  // method called if C is not a ResourceManager
  template <typename C>
  static void callCM(typename std::vector<C*>& /*managers*/, C* /*result*/, ...) {}

  // calls ResourceManager::concatManagers if C is a ResourceManager
  static void callConcatManagers(typename std::vector<T*>& managers, T* result)
  { callCM<T>(managers, result, nullptr); }


  // method called if C is a ResourceManager
  template <typename C>
  static void callGR(std::vector<std::string> &resources, C* iface, typename C::resource_manager_type*)
  {
    resources = iface->getNames();
  }

  // method called if C is not a ResourceManager
  template <typename C>
  static void callGR(std::vector<std::string> &/*resources*/, T* /*iface*/, ...) { }

  // calls ResourceManager::concatManagers if C is a ResourceManager
  static void callGetResources(std::vector<std::string> &resources, T* iface)
  { return callGR<T>(resources, iface, nullptr); }

  template <typename C>
  static T* newCI(std::vector<ResourceManagerBase*> &guards, typename C::resource_manager_type*)
  {
    T* iface_combo = new T;
    // save the new interface pointer to allow for its correct destruction
    guards.push_back(static_cast<ResourceManagerBase*>(iface_combo));
    return iface_combo;
  }

  // method called if C is not a ResourceManager
  template <typename C>
  static T* newCI(std::vector<ResourceManagerBase*> &/*guards*/, ...) {
    // it is not a ResourceManager
    ROS_ERROR("You cannot register multiple interfaces of the same type which are "
              "not of type ResourceManager. There is no established protocol "
              "for combining them.");
    return nullptr;
  }

  static T* newCombinedInterface(std::vector<ResourceManagerBase*> &guards)
  {
    return newCI<T>(guards, nullptr);
  }

};

} // namespace internal

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
    internal::CheckIsResourceManager<T>::callGetResources(resources_[iface_name], iface);
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
        return nullptr;
      }
      iface_list.push_back(iface);
    }

    // look for interfaces registered in the registered hardware
    for (const auto& interface_manager : interface_managers_) {
      T* iface = interface_manager->get<T>();
      if (iface)
        iface_list.push_back(iface);
    }

    if(iface_list.size() == 0)
      return nullptr;

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
      iface_combo = internal::CheckIsResourceManager<T>::newCombinedInterface(interface_destruction_list_);
      if(iface_combo) {
        // concat all of the resource managers together
        internal::CheckIsResourceManager<T>::callConcatManagers(iface_list, iface_combo);
        // save the combined interface for if this is called again
        interfaces_combo_[type_name] = iface_combo;
        num_ifaces_registered_[type_name] = iface_list.size();
      } else {
        // it is not a ResourceManager
        ROS_ERROR("You cannot register multiple interfaces of the same type which are "
                  "not of type ResourceManager. There is no established protocol "
                  "for combining them.");
        iface_combo = nullptr;
      }
    }
    return iface_combo;
  }

  /** \return Vector of interface names registered to this instance. */
  std::vector<std::string> getNames() const
  {
    std::vector<std::string> out;
    out.reserve(interfaces_.size());
    for (const auto& interface : interfaces_)
    {
      out.push_back(interface.first);
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
  std::vector<ResourceManagerBase*> interface_destruction_list_;
  /// This will allow us to check the resources based on the demangled type name of the interface
  ResourceMap resources_;
};

} // namespace
