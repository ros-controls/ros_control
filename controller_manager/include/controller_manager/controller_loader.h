///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012, hiDOF, Inc
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

#ifndef CONRTOLLER_MANAGER_CONTROLLER_LOADER_H
#define CONRTOLLER_MANAGER_CONTROLLER_LOADER_H

#include <pluginlib/class_loader.hpp>
#include <controller_manager/controller_loader_interface.h>
#include <boost/shared_ptr.hpp>

namespace controller_manager
{

/** \brief Pluginlib-Based Controller Loader
 *
 * This default controller loader uses pluginlib to load and then instantiate
 * controller libraries.
 *
 * \tparam T The base class of the controller types to be loaded
 *
 */

template <class T>
class ControllerLoader : public ControllerLoaderInterface
{
public:
  ControllerLoader(const std::string& package, const std::string& base_class) :
    ControllerLoaderInterface(base_class),
    package_(package),
    base_class_(base_class)
  {
    reload();
  }

  controller_interface::ControllerBaseSharedPtr createInstance(const std::string& lookup_name)
  {
    return controller_loader_->createInstance(lookup_name);
  }

  std::vector<std::string> getDeclaredClasses()
  {
    return controller_loader_->getDeclaredClasses();
  }

  void reload()
  {
    controller_loader_.reset(new pluginlib::ClassLoader<T>(package_, base_class_) );
  }

private:
  std::string package_;
  std::string base_class_;
  boost::shared_ptr<pluginlib::ClassLoader<T> > controller_loader_;
};

}

#endif
