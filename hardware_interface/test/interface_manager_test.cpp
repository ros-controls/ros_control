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

class TestCombinationHandle
{
public:
  TestCombinationHandle(const std::string& name) : name_(name) {}

  std::string getName() const
  {
    return name_;
  }

private:
  std::string name_;
};

class TestCombinationInterface : public ResourceManager<TestCombinationHandle> {};

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

TEST(InterfaceManagerTest, InterfaceCombination)
{
  // Create two InterfaceManagers, each containing a simple interface and a ResourceManager interface,
  // and an additional simple interface for the second one
  InterfaceManager root_mgr;
  InterfaceManager leaf_mgr;

  FooInterface foo_root_iface(1);
  root_mgr.registerInterface(&foo_root_iface);
  FooInterface foo_leaf_iface(2);
  leaf_mgr.registerInterface(&foo_leaf_iface);

  BazInterface baz_leaf_iface;
  baz_leaf_iface.baz = 4.2;
  leaf_mgr.registerInterface(&baz_leaf_iface);

  TestCombinationInterface combi_root_iface;
  TestCombinationHandle combi_root_handle("combi_root_handle");
  combi_root_iface.registerHandle(combi_root_handle);
  root_mgr.registerInterface(&combi_root_iface);

  TestCombinationInterface combi_leaf_iface;
  TestCombinationHandle combi_leaf_handle("combi_leaf_handle");
  combi_leaf_iface.registerHandle(combi_leaf_handle);
  leaf_mgr.registerInterface(&combi_leaf_iface);

  // Now register leaf_mgr to root_mgr
  root_mgr.registerInterfaceManager(&leaf_mgr);

  // Querying FooInterface should not work anymore, as this would require combining the interfaces
  FooInterface* foo_iface_ptr = root_mgr.get<FooInterface>();
  EXPECT_EQ(nullptr, foo_iface_ptr);

  // BazInterface should still work, as there is only one interface of this type
  BazInterface* baz_iface_ptr = root_mgr.get<BazInterface>();
  EXPECT_NE(nullptr, baz_iface_ptr);
  if (baz_iface_ptr != nullptr) // Don't crash on error
  {
    EXPECT_EQ(4.2, baz_iface_ptr->baz);
  }

  // Check presence of handles in combined interface
  TestCombinationInterface* combi_iface_ptr = root_mgr.get<TestCombinationInterface>();
  EXPECT_NE(nullptr, combi_iface_ptr);
  if (combi_iface_ptr != nullptr) // Don't crash on error
  {
    std::vector<std::string> combi_handle_names = combi_iface_ptr->getNames();
    EXPECT_EQ(2, combi_handle_names.size());
    EXPECT_NE(combi_handle_names.end(), std::find(combi_handle_names.begin(), combi_handle_names.end(), "combi_root_handle"))
      << "Did not find handle 'combi_root_handle' in combined interface";
    EXPECT_NE(combi_handle_names.end(), std::find(combi_handle_names.begin(), combi_handle_names.end(), "combi_leaf_handle"))
      << "Did not find handle 'combi_leaf_handle' in combined interface";
  }

  // Check presence of interfaces in combined interface
  std::vector<std::string> iface_names = root_mgr.getNames();
  EXPECT_EQ(3, iface_names.size());
  EXPECT_NE(iface_names.end(), std::find(iface_names.begin(), iface_names.end(), "FooInterface"))
    << "Did not find interface 'FooInterface' in combined interface";
  EXPECT_NE(iface_names.end(), std::find(iface_names.begin(), iface_names.end(), "BazInterface"))
    << "Did not find interface 'BazInterface' in combined interface";
  EXPECT_NE(iface_names.end(), std::find(iface_names.begin(), iface_names.end(), "TestCombinationInterface"))
    << "Did not find interface 'TestCombinationInterface' in combined interface";

  // Check presence of resources in combined interface
  std::vector<std::string> iface_res = root_mgr.getInterfaceResources("FooInterface");
  EXPECT_EQ(0, iface_res.size());
  iface_res = root_mgr.getInterfaceResources("BazInterface");
  EXPECT_EQ(0, iface_res.size());
  iface_res = root_mgr.getInterfaceResources("TestCombinationInterface");
  EXPECT_EQ(2, iface_res.size());
  EXPECT_NE(iface_res.end(), std::find(iface_res.begin(), iface_res.end(), "combi_root_handle"))
    << "Did not find interface resource 'combi_root_handle' for interface 'TestCombinationInterface' in combined interface";
  EXPECT_NE(iface_res.end(), std::find(iface_res.begin(), iface_res.end(), "combi_leaf_handle"))
    << "Did not find interface resource 'combi_leaf_handle' for interface 'TestCombinationInterface' in combined interface";
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
