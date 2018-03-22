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

//! \author Vijay Pradeep

#ifndef CONTROLLER_MANAGER_TESTS_MY_DUMMY_CONTROLLER_H
#define CONTROLLER_MANAGER_TESTS_MY_DUMMY_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.hpp>

namespace controller_manager_tests
{

class MyDummyInterface : public hardware_interface::HardwareInterface
{
public:
  MyDummyInterface()
  {

  }
};

class MyDummyController : public controller_interface::Controller<MyDummyInterface>
{
public:
  MyDummyController() { }

  using controller_interface::Controller<MyDummyInterface>::init;
  bool init(MyDummyInterface* /*hw*/, ros::NodeHandle& /*n*/) { return true; }
  void starting(const ros::Time& /*time*/) { }
  void update(const ros::Time& /*time*/, const ros::Duration& /*period*/) { }
  void stopping(const ros::Time& /*time*/) { }
};

}

#endif
