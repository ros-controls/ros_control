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
  static void callCM(typename std::vector<C*>& managers, C* result, typename C::ResourceManagerType*)
  {
    // We have to typecast back to base class
    std::vector<typename C::ResourceManagerType*> managers_in(managers.begin(), managers.end());
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
  static void callGR(std::vector<std::string> &resources, C* iface, typename C::ResourceManagerType*)
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
  static T* newCI(std::vector<ResourceManagerBase*> &guards, typename C::ResourceManagerType*)
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

/**
 * \brief Manager for hardware interface registrations.
 *
 * This class enables the registration of interfaces based on their class type,
 * handling all the required demangling and storage. The registration ensures
 * the presence of at most one interface instance per type. Accessors for
 * interface listing are provided. Additionally, combinations of interfaces as
 * required in \c CombinedRobotHW are handled transparently.
 */
class InterfaceManager
{
public:
  /**
   * \brief Register an interface.
   *
   * This associates the name of the type of interface to be registered with
   * the given pointer.
   *
   * \note The registration of an interface will replace previously registered
   * instances of its type
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

  /**
   * \brief Register another interface manager.
   *
   * This manager will be integrated transparently into all further access
   * methods.
   *
   * \param iface_man A pointer to the interface manager to store
   */
  void registerInterfaceManager(InterfaceManager* iface_man)
  {
    interface_managers_.push_back(iface_man);
  }

  /**
   * \brief Get an interface.
   *
   * If this class and its registered sub-managers only have one registered
   * instance of the requested interface type, this will be returned. If
   * multiple instances of the interface type were registered and the type is a
   * \c ResourceManager, this call will try to combine them into a single
   * handle. In all other cases, \c nullptr will be returned.
   *
   * \note As this class can only store one instance of each interface type,
   * the only way multiple instances of the same interface type can be
   * registered  is through the registration of sub-managers.
   *
   * \note If there are multiple registered interfaces of the requested type
   * and they are not \c ResourceManager, this method will not be able to
   * combine them and will return \c nullptr.
   *
   * \tparam T The interface type
   * \return A handle for the stored interfaces of type \c T or \c nullptr
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
        // Multiple interfaces of the same type which are not ResourceManager cannot be combined, so return
        // nullptr
        iface_combo = nullptr;
      }
    }
    return iface_combo;
  }

  /**
   * \brief Lists the demangled names of all registered interfaces.
   *
   * This includes the interfaces directly registered to this instance and
   * the interfaces registered to registered managers. As multiple interfaces
   * of the same type will get merged on access, duplicates are omitted.
   *
   * \return Vector of demangled interface names of registered interfaces.
   **/
  std::vector<std::string> getNames() const
  {
    std::vector<std::string> out;
    out.reserve(interfaces_.size());
    for (const auto& interface : interfaces_)
    {
      out.push_back(interface.first);
    }

    for (const auto& interface_manager : interface_managers_)
    {
      // Make sure interfaces appear only once, as they may have been combined
      for (const auto& interface_name : interface_manager->getNames())
      {
        if (std::find(out.begin(), out.end(), interface_name) == out.end())
        {
          out.push_back(interface_name);
        }
      }
    }

    return out;
  }

  /**
   * \brief Get the resource names registered to an interface, specified by type.
   *
   * This will return the list of all registered resources for
   * \c ResourceManager interfaces and an empty list for all others.
   *
   * \param iface_type The demangled type name of an interface
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

    for (const auto& interface_manager : interface_managers_)
    {
      std::vector<std::string> resources = interface_manager->getInterfaceResources(iface_type);
      out.insert(out.end(), resources.begin(), resources.end());
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
