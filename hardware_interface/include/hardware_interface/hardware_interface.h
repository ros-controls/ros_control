///////////////////////////////////////////////////////////////////////////////
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

/*
 * Author: Wim Meeussen
 */


#ifndef HARDWARE_INTERFACE_HARDWARE_INTERFACE_H
#define HARDWARE_INTERFACE_HARDWARE_INTERFACE_H

#include <exception>
#include <string>
#include <vector>
#include <typeinfo>
#include <set>

namespace hardware_interface{

struct Resource
{
  std::string type;
  std::string name;

  Resource(std::string type, std::string name) : type(type), name(name) { }

  bool operator==(const Resource& rhs) const
  {
    return (type == rhs.type) && (name == rhs.name);
  }

  bool operator<(const Resource& rhs) const
  {
    if(type == rhs.type)
      return name < rhs.name;
    return type < rhs.type;
  }
};


class HardwareInterface
{
public:
  std::vector<std::string> getRegisteredTypes() {return types_;}
  void registerType(std::string type) { types_.push_back(type); }
  virtual ~HardwareInterface() { };

  // Resource management
  virtual void claim(Resource resource) { claims_.insert(resource); }
  void clearClaims()                    { claims_.clear(); }
  std::set<Resource> getClaims() const  { return claims_; }

private:
  std::vector<std::string> types_;     //!< Names of implemented derived hardware interfaces
  std::set<Resource> claims_;          //!< Claimed resources in the current claim cycle
};






class HardwareInterfaceException: public std::exception
{
public:
  HardwareInterfaceException(const std::string& message)
    : msg(message) {};

  virtual ~HardwareInterfaceException() throw() {};

  virtual const char* what() const throw()
  {
    return msg.c_str();
  }


private:
  std::string msg;
};
}


#endif
