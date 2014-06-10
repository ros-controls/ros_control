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
//   * Neither the name of PAL Robotics S.L. nor the names of its
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

#include <vector>

#include <gtest/gtest.h>

#include <transmission_interface/simple_transmission.h>

using std::vector;
using namespace transmission_interface;

// Floating-point value comparison threshold
const double EPS = 1e-6;

TEST(PreconditionsTest, ExceptionThrowing)
{
  // Invalid instance creation: Transmission cannot have zero reduction
  EXPECT_THROW(SimpleTransmission(0.0),       TransmissionInterfaceException);
  EXPECT_THROW(SimpleTransmission(0.0,  1.0), TransmissionInterfaceException);
  EXPECT_THROW(SimpleTransmission(0.0, -1.0), TransmissionInterfaceException);

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
  double a_val = 0.0;
  double j_val = 0.0;

  ActuatorData a_good_data;
  a_good_data.position = vector<double*>(1, &a_val);
  a_good_data.velocity = vector<double*>(1, &a_val);
  a_good_data.effort   = vector<double*>(1, &a_val);

  JointData j_good_data;
  j_good_data.position = vector<double*>(1, &j_val);
  j_good_data.velocity = vector<double*>(1, &j_val);
  j_good_data.effort   = vector<double*>(1, &j_val);

  ActuatorData a_bad_data;
  a_bad_data.position = vector<double*>(1);
  a_bad_data.velocity = vector<double*>(1);
  a_bad_data.effort   = vector<double*>(1);

  JointData j_bad_data;
  j_bad_data.position = vector<double*>(1);
  j_bad_data.velocity = vector<double*>(1);
  j_bad_data.effort   = vector<double*>(1);

  ActuatorData a_bad_size;
  JointData    j_bad_size;

  // Transmission instance
  SimpleTransmission trans = SimpleTransmission(1.0);

  // Data with invalid pointers should trigger an assertion
  EXPECT_DEATH(trans.actuatorToJointEffort(a_bad_data,  j_bad_data),  ".*");
  EXPECT_DEATH(trans.actuatorToJointEffort(a_good_data, j_bad_data),  ".*");
  EXPECT_DEATH(trans.actuatorToJointEffort(a_bad_data,  j_good_data), ".*");

  EXPECT_DEATH(trans.actuatorToJointVelocity(a_bad_data,  j_bad_data),  ".*");
  EXPECT_DEATH(trans.actuatorToJointVelocity(a_good_data, j_bad_data),  ".*");
  EXPECT_DEATH(trans.actuatorToJointVelocity(a_bad_data,  j_good_data), ".*");

  EXPECT_DEATH(trans.actuatorToJointPosition(a_bad_data,  j_bad_data),  ".*");
  EXPECT_DEATH(trans.actuatorToJointPosition(a_good_data, j_bad_data),  ".*");
  EXPECT_DEATH(trans.actuatorToJointPosition(a_bad_data,  j_good_data), ".*");

  EXPECT_DEATH(trans.jointToActuatorEffort(j_bad_data,  a_bad_data),  ".*");
  EXPECT_DEATH(trans.jointToActuatorEffort(j_good_data, a_bad_data),  ".*");
  EXPECT_DEATH(trans.jointToActuatorEffort(j_bad_data,  a_good_data), ".*");

  EXPECT_DEATH(trans.jointToActuatorVelocity(j_bad_data,  a_bad_data),  ".*");
  EXPECT_DEATH(trans.jointToActuatorVelocity(j_good_data, a_bad_data),  ".*");
  EXPECT_DEATH(trans.jointToActuatorVelocity(j_bad_data,  a_good_data), ".*");

  EXPECT_DEATH(trans.jointToActuatorPosition(j_bad_data,  a_bad_data),  ".*");
  EXPECT_DEATH(trans.jointToActuatorPosition(j_good_data, a_bad_data),  ".*");
  EXPECT_DEATH(trans.jointToActuatorPosition(j_bad_data,  a_good_data), ".*");

  // Wrong parameter sizes should trigger an assertion
  EXPECT_DEATH(trans.actuatorToJointEffort(a_bad_size,  j_bad_size),  ".*");
  EXPECT_DEATH(trans.actuatorToJointEffort(a_good_data, j_bad_size),  ".*");
  EXPECT_DEATH(trans.actuatorToJointEffort(a_bad_size,  j_good_data), ".*");

  EXPECT_DEATH(trans.actuatorToJointVelocity(a_bad_size,  j_bad_size),  ".*");
  EXPECT_DEATH(trans.actuatorToJointVelocity(a_good_data, j_bad_size),  ".*");
  EXPECT_DEATH(trans.actuatorToJointVelocity(a_bad_size,  j_good_data), ".*");

  EXPECT_DEATH(trans.actuatorToJointPosition(a_bad_size,  j_bad_size),  ".*");
  EXPECT_DEATH(trans.actuatorToJointPosition(a_good_data, j_bad_size),  ".*");
  EXPECT_DEATH(trans.actuatorToJointPosition(a_bad_size,  j_good_data), ".*");

  EXPECT_DEATH(trans.jointToActuatorEffort(j_bad_size,  a_bad_size),  ".*");
  EXPECT_DEATH(trans.jointToActuatorEffort(j_good_data, a_bad_size),  ".*");
  EXPECT_DEATH(trans.jointToActuatorEffort(j_bad_size,  a_good_data), ".*");

  EXPECT_DEATH(trans.jointToActuatorVelocity(j_bad_size,  a_bad_size),  ".*");
  EXPECT_DEATH(trans.jointToActuatorVelocity(j_good_data, a_bad_size),  ".*");
  EXPECT_DEATH(trans.jointToActuatorVelocity(j_bad_size,  a_good_data), ".*");

  EXPECT_DEATH(trans.jointToActuatorPosition(j_bad_size,  a_bad_size),  ".*");
  EXPECT_DEATH(trans.jointToActuatorPosition(j_good_data, a_bad_size),  ".*");
  EXPECT_DEATH(trans.jointToActuatorPosition(j_bad_size,  a_good_data), ".*");
}
#endif // NDEBUG

TEST(PreconditionsTest, AccessorValidation)
{
  SimpleTransmission trans(2.0, -1.0);

  EXPECT_EQ(1,    trans.numActuators());
  EXPECT_EQ(1,    trans.numJoints());
  EXPECT_EQ( 2.0, trans.getActuatorReduction());
  EXPECT_EQ(-1.0, trans.getJointOffset());
}

class TransmissionSetup
{
public:
  TransmissionSetup()
    : a_val(),
      j_val(),
      a_vec(vector<double*>(1, &a_val)),
      j_vec(vector<double*>(1, &j_val)) {}

protected:
  // Input/output transmission data
  double a_val;
  double j_val;
  vector<double*> a_vec;
  vector<double*> j_vec;
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
    {
      ActuatorData a_data;
      a_data.effort = a_vec;
      *a_data.effort[0] = ref_val;

      JointData j_data;
      j_data.effort = j_vec;

      trans.actuatorToJointEffort(a_data, j_data);
      trans.jointToActuatorEffort(j_data, a_data);
      EXPECT_NEAR(ref_val, *a_data.effort[0], EPS);
    }

    // Velocity interface
    {
      ActuatorData a_data;
      a_data.velocity = a_vec;
      *a_data.velocity[0] = ref_val;

      JointData j_data;
      j_data.velocity = j_vec;

      trans.actuatorToJointVelocity(a_data, j_data);
      trans.jointToActuatorVelocity(j_data, a_data);
      EXPECT_NEAR(ref_val, *a_data.velocity[0], EPS);
    }

    // Position interface
    {
      ActuatorData a_data;
      a_data.position = a_vec;
      *a_data.position[0] = ref_val;

      JointData j_data;
      j_data.position = j_vec;

      trans.actuatorToJointPosition(a_data, j_data);
      trans.jointToActuatorPosition(j_data, a_data);
      EXPECT_NEAR(ref_val, *a_data.position[0], EPS);
    }
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

  *a_vec[0] = 1.0;

  // Effort interface
  {
    ActuatorData a_data;
    a_data.effort = a_vec;

    JointData j_data;
    j_data.effort = j_vec;

    trans.actuatorToJointEffort(a_data, j_data);
    EXPECT_NEAR(10.0, *j_data.effort[0], EPS);
  }

  // Velocity interface
  {
    ActuatorData a_data;
    a_data.velocity = a_vec;

    JointData j_data;
    j_data.velocity = j_vec;

    trans.actuatorToJointVelocity(a_data, j_data);
    EXPECT_NEAR(0.1, *j_data.velocity[0], EPS);
  }

  // Position interface
  {
    ActuatorData a_data;
    a_data.position = a_vec;

    JointData j_data;
    j_data.position = j_vec;

    trans.actuatorToJointPosition(a_data, j_data);
    EXPECT_NEAR(1.1, *j_data.position[0], EPS);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
