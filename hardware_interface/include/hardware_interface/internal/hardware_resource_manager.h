///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
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

/// \author Adolfo Rodriguez Tsouroukdissian

#pragma once


#include <string>

#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/internal/resource_manager.h>

namespace hardware_interface
{

struct ClaimResources;
struct DontClaimResources;

/**
 * \brief Base class for handling hardware resources.
 *
 * Hardware resources are encapsulated inside handle instances, and this class allows to register and get them by name.
 * It is also possible to specify through the \b ClaimPolicy template parameter whether getting a handle claims
 * the corresponding resource or not, like in the following example
 * \code
 * // If unspecified, the resource manager will not claim resources
 * {
 *   HardwareResourceManager<JointStateHandle> m;
 *   // Populate m
 *   m.getHandle("handle_name"); // DOES NOT claim the "handle_name" resource
 * }
 *
 * // Explicitly set ClaimPolicy to DontClaimResources
 * {
 *   HardwareResourceManager<JointStateHandle, DontClaimResources> m;
 *   // Populate m
 *   m.getHandle("handle_name"); // DOES NOT claim the "handle_name" resource
 * }
 *
 * // Explicitly set ClaimPolicy to ClaimResources
 * {
 *   HardwareResourceManager<JointHandle, ClaimResources> m;
 *   // Populate m
 *   m.getHandle("handle_name"); // DOES claim the "handle_name" resource
 * }
 *
 * \endcode
 * \tparam ResourceHandle Resource handle type. The only requisite on the type is that it implements a
 * <tt>std::string getName()</tt> method.
 * \tparam ClaimPolicy Specifies the resource claiming policy for resource handling
 */

template <class ResourceHandle, class ClaimPolicy = DontClaimResources>
class HardwareResourceManager : public HardwareInterface, public ResourceManager<ResourceHandle>
{
public:
  typedef ResourceHandle ResourceHandleType;

  /** \name Non Real-Time Safe Functions
   *\{*/

  /**
   * \brief Get a resource handle by name.
   *
   * \note If the \b ClaimPolicy template parameter is set to \b ClaimResources, calling this method will internally
   * claim the resource.
   * If set to \b DontClaimResources, calling this method will not claim the resource.
   * \param name Resource name.
   * \return Resource associated to \e name. If the resource name is not found, an exception is thrown.
   */
  ResourceHandle getHandle(const std::string& name)
  {
    try
    {
      ResourceHandle out = this->ResourceManager<ResourceHandle>::getHandle(name);

      // If ClaimPolicy type is ClaimResources, the below method claims resources, for DontClaimResources it's a no-op
      ClaimPolicy::claim(this, name);

      return out;
    }
    catch(const std::logic_error& e)
    {
      throw HardwareInterfaceException(e.what());
    }
  }

  /*\}*/
};

/** \cond HIDDEN_SYMBOLS */
struct ClaimResources
{
  static void claim(HardwareInterface* hw, const std::string& name) {hw->claim(name);}
};

struct DontClaimResources
{
  static void claim(HardwareInterface* /*hw*/, const std::string& /*name*/) {}
};
/** \endcond */

}
