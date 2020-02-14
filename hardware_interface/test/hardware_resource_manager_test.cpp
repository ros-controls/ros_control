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

#include <algorithm>
#include <set>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <hardware_interface/internal/hardware_resource_manager.h>

using std::find;
using std::set;
using std::string;
using std::vector;

using namespace hardware_interface;

class HandleType
{
public:
  HandleType(const string& name, int val) : name_(name), val_(val) {}
  string getName() const {return name_;}
  int getValue() {return val_;}
private:
  string name_;
  int val_;
};

class HardwareResourceManagerTest : public ::testing::Test
{
protected:
  HandleType h1 = {"resource1", 1};
  HandleType h2 = {"resource2", 2};
};

TEST_F(HardwareResourceManagerTest, ExcerciseApi)
{
  // Populate resource manager
  HardwareResourceManager<HandleType> mgr;
  ASSERT_TRUE(mgr.getNames().empty());

  mgr.registerHandle(h1);
  mgr.registerHandle(h2);

  // Joint names
  vector<string> names = mgr.getNames();
  EXPECT_EQ(2, names.size());
  EXPECT_TRUE(find(names.begin(), names.end(), h1.getName()) != names.end());
  EXPECT_TRUE(find(names.begin(), names.end(), h2.getName()) != names.end());

  // Get handles
  EXPECT_NO_THROW(mgr.getHandle(h1.getName()));
  EXPECT_NO_THROW(mgr.getHandle(h2.getName()));

  HandleType h1_tmp = mgr.getHandle(h1.getName());
  EXPECT_EQ(h1.getName(), h1_tmp.getName());
  EXPECT_EQ(h1.getValue(), h1_tmp.getValue());

  HandleType h2_tmp = mgr.getHandle(h2.getName());
  EXPECT_EQ(h2.getName(), h2_tmp.getName());
  EXPECT_EQ(h2.getValue(), h2_tmp.getValue());

  // Trying to get a handle to a non-existent resource should throw
  EXPECT_THROW(mgr.getHandle("no_resource"), HardwareInterfaceException);
}

TEST_F(HardwareResourceManagerTest, HandleRewriting)
{
  // Populate resource manager
  HardwareResourceManager<HandleType> mgr;
  mgr.registerHandle(h1);

  // Check handle values
  {
    EXPECT_NO_THROW(mgr.getHandle(h1.getName()));
    HandleType h1_tmp = mgr.getHandle(h1.getName());
    EXPECT_EQ(h1.getValue(), h1_tmp.getValue());
  }

  // Re-register resource: same name, different value
  HandleType h3(h1.getName(), h2.getValue());
  mgr.registerHandle(h3);

  // Check that there is indeed only one registered resource
  vector<string> names = mgr.getNames();
  EXPECT_EQ(1, names.size());
  EXPECT_TRUE(find(names.begin(), names.end(), h1.getName()) != names.end());

  // Get handle and check that it now contains a different value
  {
    EXPECT_NO_THROW(mgr.getHandle(h1.getName()));
    HandleType h3_tmp = mgr.getHandle(h1.getName());
    EXPECT_EQ(h2.getValue(), h3_tmp.getValue());
    EXPECT_NE(h1.getValue(), h3_tmp.getValue());
  }
}

TEST_F(HardwareResourceManagerTest, ResourceClaims)
{
  // Default: Manager that does not claim resources
  {
    HardwareResourceManager<HandleType> mgr;
    mgr.registerHandle(h1);
    mgr.registerHandle(h2);

    EXPECT_TRUE(mgr.getClaims().empty());

    // Getting a handle is the resource-claiming operation (which in this case should not claim anything)
    mgr.getHandle(h1.getName());
    mgr.getHandle(h2.getName());
    mgr.getHandle(h2.getName()); // Getting twice the same resource

    EXPECT_TRUE(mgr.getClaims().empty());
  }

  // Explicit setting: Manager that does not claim resources
  {
    HardwareResourceManager<HandleType, DontClaimResources> mgr;
    mgr.registerHandle(h1);
    mgr.registerHandle(h2);
    EXPECT_TRUE(mgr.getClaims().empty());

    // Getting a handle is the resource-claiming operation (which in this case should not claim anything)
    mgr.getHandle(h1.getName());
    mgr.getHandle(h2.getName());
    mgr.getHandle(h2.getName()); // Getting twice the same resource

    EXPECT_TRUE(mgr.getClaims().empty());
  }

  // Explicit setting: Manager that does claim resources
  {
    HardwareResourceManager<HandleType, ClaimResources> mgr;
    mgr.registerHandle(h1);
    mgr.registerHandle(h2);
    EXPECT_TRUE(mgr.getClaims().empty());

    // Getting a handle is the resource-claiming operation (which in this case should indeed claim resources)
    mgr.getHandle(h1.getName());
    mgr.getHandle(h2.getName());
    mgr.getHandle(h2.getName()); // Getting twice the same resource, claims list should still be of size 2

    set<string> claims = mgr.getClaims();
    EXPECT_EQ(2, claims.size());
    EXPECT_TRUE(find(claims.begin(), claims.end(), h1.getName()) != claims.end());
    EXPECT_TRUE(find(claims.begin(), claims.end(), h2.getName()) != claims.end());
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
