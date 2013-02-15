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

#include <transmission_interface/simple_transmission.h>

using namespace transmission_interface;

// Floating-point value comparison threshold
const double EPS = 1e-6;

TEST(PreconditionsTest, ExceptionThrowing)
{
  // Invalid instance creation: Transmission cannot have zero reduction
  EXPECT_THROW(SimpleTransmission(0.0),       TransmissionException);
  EXPECT_THROW(SimpleTransmission(0.0,  1.0), TransmissionException);
  EXPECT_THROW(SimpleTransmission(0.0, -1.0), TransmissionException);

  // Valid instance creation
  EXPECT_NO_THROW(SimpleTransmission( 1.0));
  EXPECT_NO_THROW(SimpleTransmission( 1.0,  1.0));
  EXPECT_NO_THROW(SimpleTransmission(-1.0,  1.0));
  EXPECT_NO_THROW(SimpleTransmission( 1.0, -1.0));
  EXPECT_NO_THROW(SimpleTransmission(-1.0, -1.0));
}

#ifndef NDEBUG // NOTE: This test validates assertion triggering, hence only gets compiled in debug mode
TEST(PreconditionsTest, AssertionTriggering)
{
  // Create input/output transmission data
  double a_val1 = 0.0, a_val2 = 0.0;
  double j_val1 = 0.0, j_val2 = 0.0;

  std::vector<double*> a_data_bad;
  a_data_bad.push_back(&a_val1);
  a_data_bad.push_back(&a_val2);

  std::vector<double*> j_data_bad;
  j_data_bad.push_back(&j_val1);
  j_data_bad.push_back(&j_val2);

  std::vector<double*> a_data_good(1, &a_val1);
  std::vector<double*> j_data_good(1, &j_val1);

  // Transmission instance
  SimpleTransmission trans = SimpleTransmission(1.0);

  // Wrong parameter sizes should trigger an assertion
  EXPECT_DEATH(trans.actuatorToJointEffort(a_data_bad,  j_data_bad),  ".*");
  EXPECT_DEATH(trans.actuatorToJointEffort(a_data_good, j_data_bad),  ".*");
  EXPECT_DEATH(trans.actuatorToJointEffort(a_data_bad,  j_data_good), ".*");

  EXPECT_DEATH(trans.actuatorToJointVelocity(a_data_bad,  j_data_bad),  ".*");
  EXPECT_DEATH(trans.actuatorToJointVelocity(a_data_good, j_data_bad),  ".*");
  EXPECT_DEATH(trans.actuatorToJointVelocity(a_data_bad,  j_data_good), ".*");

  EXPECT_DEATH(trans.actuatorToJointPosition(a_data_bad,  j_data_bad),  ".*");
  EXPECT_DEATH(trans.actuatorToJointPosition(a_data_good, j_data_bad),  ".*");
  EXPECT_DEATH(trans.actuatorToJointPosition(a_data_bad,  j_data_good), ".*");

  EXPECT_DEATH(trans.jointToActuatorEffort(j_data_bad,  a_data_bad),  ".*");
  EXPECT_DEATH(trans.jointToActuatorEffort(j_data_good, a_data_bad),  ".*");
  EXPECT_DEATH(trans.jointToActuatorEffort(j_data_bad,  a_data_good), ".*");

  EXPECT_DEATH(trans.jointToActuatorVelocity(j_data_bad,  a_data_bad),  ".*");
  EXPECT_DEATH(trans.jointToActuatorVelocity(j_data_good, a_data_bad),  ".*");
  EXPECT_DEATH(trans.jointToActuatorVelocity(j_data_bad,  a_data_good), ".*");

  EXPECT_DEATH(trans.jointToActuatorPosition(j_data_bad,  a_data_bad),  ".*");
  EXPECT_DEATH(trans.jointToActuatorPosition(j_data_good, a_data_bad),  ".*");
  EXPECT_DEATH(trans.jointToActuatorPosition(j_data_bad,  a_data_good), ".*");
}
#endif // NDEBUG


class TransmissionSetup
{
public:
  TransmissionSetup()
    : a_val(),
      j_val(),
      a_data(std::vector<double*>(1, &a_val)),
      j_data(std::vector<double*>(1, &j_val)) {}

protected:
  // Input/output transmission data
  double a_val;
  double j_val;
  std::vector<double*> a_data;
  std::vector<double*> j_data;
};

// NOTE: I tried to make the above a proper gtest fixture and below inherit from
// public ::testing::WithParamInterface<SimpleTransmission>, but gcc complains with
// "error: expected template-name before ‘<’ token", hence this slightly less optimal class hierarchy

/// \brief Exercises the actuator->joint->actuator roundtrip, which should yield the identity map.
class BlackBoxTest : public TransmissionSetup,
                     public ::testing::TestWithParam<SimpleTransmission>

{
protected:
  /// \param trans Transmission instance.
  /// \param ref_val Reference value (effort, velocity or position) that will be transformed with the respective forward
  /// and inverse transmission transformations.
  void testIdentityMap(SimpleTransmission& trans, const double ref_val)
  {
    // Effort interface
    *a_data[0] = ref_val;
    trans.actuatorToJointEffort(a_data, j_data);
    trans.jointToActuatorEffort(j_data, a_data);
    EXPECT_NEAR(ref_val, *a_data[0], EPS);

    // Velocity interface
    *a_data[0] = ref_val;
    trans.actuatorToJointVelocity(a_data, j_data);
    trans.jointToActuatorVelocity(j_data, a_data);
    EXPECT_NEAR(ref_val, *a_data[0], EPS);

    // Position interface
    *a_data[0] = ref_val;
    trans.actuatorToJointPosition(a_data, j_data);
    trans.jointToActuatorPosition(j_data, a_data);
    EXPECT_NEAR(ref_val, *a_data[0], EPS);
  }
};

TEST_P(BlackBoxTest, IdentityMap)
{
  // Transmission instance
  SimpleTransmission trans = GetParam();

  // Test transmission for positive, zero, and negative inputs
  testIdentityMap(trans,  1.0);
  testIdentityMap(trans,  0.0);
  testIdentityMap(trans, -1.0);
}

INSTANTIATE_TEST_CASE_P(IdentityMap,
                        BlackBoxTest,
                        ::testing::Values(SimpleTransmission( 10.0),
                                          SimpleTransmission(-10.0),
                                          SimpleTransmission( 10.0,  1.0),
                                          SimpleTransmission( 10.0, -1.0),
                                          SimpleTransmission(-10.0,  1.0),
                                          SimpleTransmission(-10.0, -1.0)));

class WhiteBoxTest : public TransmissionSetup,
                     public ::testing::Test {};

TEST_F(WhiteBoxTest, MoveJoint)
{
  // NOTE: We only test the actuator->joint map, as the joint->actuator map is indirectly validated in the test that
  // checks that actuator->joint->actuator == identity.

  SimpleTransmission trans(10.0, 1.0);
  *a_data[0] = 1.0;

  // Effort interface
  trans.actuatorToJointEffort(a_data, j_data);
  EXPECT_NEAR(10.0, *j_data[0], EPS);

  // Velocity interface
  trans.actuatorToJointVelocity(a_data, j_data);
  EXPECT_NEAR(0.1, *j_data[0], EPS);

  // Position interface
  trans.actuatorToJointPosition(a_data, j_data);
  EXPECT_NEAR(1.1, *j_data[0], EPS);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
