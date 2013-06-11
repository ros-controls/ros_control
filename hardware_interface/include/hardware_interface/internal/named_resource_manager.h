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

#ifndef HARDWARE_INTERFACE_INTERNAL_NAMED_RESOURCE_MANAGER_H
#define HARDWARE_INTERFACE_INTERNAL_NAMED_RESOURCE_MANAGER_H

#include <stdexcept>
#include <string>
#include <map>
#include <vector>
#include <utility>  // for std::make_pair

namespace hardware_interface
{

namespace internal
{

/**
 * \brief Class providing convenience methods around a map associative container.
 * \tparam Resource type.
 */
template <class T>
class NamedResourceManager
{
public:
  /** \return Vector containing the names of all managed resources. */
  std::vector<std::string> getNames() const
  {
    std::vector<std::string> out;
    out.reserve(resource_map_.size());
    for(typename ResourceMap::const_iterator it = resource_map_.begin(); it != resource_map_.end(); ++it)
    {
      out.push_back(it->first);
    }
    return out;
  }

  /**
   * \brief Insert a new element in the stored container.
   * If the resource name already exists, the existing resource value will be replaced with \e val.
   * \param name Resource name.
   * \param val Resource value.
   */
  void insert(const std::string& name, const T& val)
  {
    typename ResourceMap::iterator it = resource_map_.find(name);
    if (it == resource_map_.end())
    {
      resource_map_.insert(std::make_pair(name, val));
    }
    else
    {
      it->second = val;
    }
  }

  /**
   * \param name Resource name.
   * \return Resource value associated to \e name. If the resource name is not found, an exception is thrown.
   */
  T get(const std::string& name) const
  {
    typename ResourceMap::const_iterator it = resource_map_.find(name);

    if (it == resource_map_.end())
    {
      throw std::invalid_argument("Could not find resource.");
    }
    return it->second;
  }

private:
  typedef std::map<std::string, T> ResourceMap;
  ResourceMap resource_map_;
};

}

}

#endif // HARDWARE_INTERFACE_INTERNAL_NAMED_RESOURCE_MANAGER_H
