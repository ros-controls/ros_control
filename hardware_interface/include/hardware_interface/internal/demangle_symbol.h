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


#include <cstdlib>
#include <string>

#if (__GNUC__ && __cplusplus && __GNUC__ >= 3)
#include <cxxabi.h>
#endif

namespace hardware_interface
{
namespace internal
{

/**
 * \brief Demangle symbol, if symbol demangling is available.
 * \param name Symbol to demangle.
 * \return Demangled symbol if demangling is available and successful, mangled symbol otherwise.
 */
inline std::string demangleSymbol(const char* name)
{
  #if (__GNUC__ && __cplusplus && __GNUC__ >= 3)
    int         status;
    char* res = abi::__cxa_demangle(name, nullptr, nullptr, &status);
    if (res)
    {
      const std::string demangled_name(res);
      std::free(res);
      return demangled_name;
    }
    // Demangling failed, fallback to mangled name
    return std::string(name);
  #else
    return std::string(name);
  #endif
}

/**
 * \brief Convenience method for demangling type names.
 * \sa demangleSymbol
 */
template <class T>
inline std::string demangledTypeName()
{
  return demangleSymbol(typeid(T).name());
}

/**
* \brief Convenience method for demangling type names.
* \sa demangleSymbol
*/
template <class T>
inline std::string demangledTypeName(const T& val)
{
  return demangleSymbol(typeid(val).name());
}

}

}
