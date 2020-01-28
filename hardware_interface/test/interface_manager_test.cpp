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

#include <gtest/gtest.h>
#include <hardware_interface/internal/interface_manager.h>

using namespace hardware_interface;

struct FooInterface
{
  FooInterface(int foo) : foo(foo) {}
  int foo;
};

struct BarInterface
{
  char* bar;
};

struct BazInterface
{
  double baz;
};

TEST(InterfaceManagerTest, InterfaceRegistration)
{
  // Register interfaces
  FooInterface foo_iface(0);
  BarInterface bar_iface;

  InterfaceManager iface_mgr;
  iface_mgr.registerInterface(&foo_iface);
  iface_mgr.registerInterface(&bar_iface);

  // Get interfaces
  EXPECT_TRUE(&foo_iface == iface_mgr.get<FooInterface>());
  EXPECT_TRUE(&bar_iface == iface_mgr.get<BarInterface>());
  EXPECT_FALSE(iface_mgr.get<BazInterface>());
}

TEST(InterfaceManagerTest, InterfaceRewriting)
{
  // Two instances of the same interface
  FooInterface foo_iface_1(1);
  FooInterface foo_iface_2(2);

  // Register first interface and validate it
  InterfaceManager iface_mgr;

  iface_mgr.registerInterface(&foo_iface_1);

  FooInterface* foo_iface_ptr = iface_mgr.get<FooInterface>();
  EXPECT_EQ(1, foo_iface_ptr->foo);

  // Register second interface and verify that it has taken the place of the previously inserted one
  iface_mgr.registerInterface(&foo_iface_2);
  foo_iface_ptr = iface_mgr.get<FooInterface>();
  EXPECT_EQ(2, foo_iface_ptr->foo);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
