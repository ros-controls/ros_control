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

#include <gtest/gtest.h>

#include <transmission_interface/differential_transmission.h>
#include "random_generator_utils.h"

using namespace transmission_interface;
using std::vector;

// Floating-point value comparison threshold
const double EPS = 1e-6;

TEST(PreconditionsTest, ExceptionThrowing)
{
  vector<double> reduction_good(2, 1.0);
  vector<double> reduction_bad1(2, 0.0);
  vector<double> reduction_bad2(2, 0.0); reduction_bad2[0] = 1.0;
  vector<double> reduction_bad3(2, 0.0); reduction_bad3[1] = 1.0;
  vector<double> offset_good(2, 1.0);

  // Invalid instance creation: Transmission cannot have zero reduction
  EXPECT_THROW(DifferentialTransmission(reduction_bad1, reduction_good), TransmissionInterfaceException);
  EXPECT_THROW(DifferentialTransmission(reduction_bad2, reduction_good), TransmissionInterfaceException);
  EXPECT_THROW(DifferentialTransmission(reduction_bad3, reduction_good), TransmissionInterfaceException);

  EXPECT_THROW(DifferentialTransmission(reduction_good, reduction_bad1), TransmissionInterfaceException);
  EXPECT_THROW(DifferentialTransmission(reduction_good, reduction_bad2), TransmissionInterfaceException);
  EXPECT_THROW(DifferentialTransmission(reduction_good, reduction_bad3), TransmissionInterfaceException);

  EXPECT_THROW(DifferentialTransmission(reduction_bad1, reduction_good, offset_good), TransmissionInterfaceException);
  EXPECT_THROW(DifferentialTransmission(reduction_bad2, reduction_good, offset_good), TransmissionInterfaceException);
  EXPECT_THROW(DifferentialTransmission(reduction_bad3, reduction_good, offset_good), TransmissionInterfaceException);

  EXPECT_THROW(DifferentialTransmission(reduction_good, reduction_bad1, offset_good), TransmissionInterfaceException);
  EXPECT_THROW(DifferentialTransmission(reduction_good, reduction_bad2, offset_good), TransmissionInterfaceException);
  EXPECT_THROW(DifferentialTransmission(reduction_good, reduction_bad3, offset_good), TransmissionInterfaceException);

  // Invalid instance creation: Wrong parameter sizes
  vector<double> reduction_bad_size(1, 1.0);
  vector<double>& offset_bad_size = reduction_bad_size;
  EXPECT_THROW(DifferentialTransmission(reduction_bad_size, reduction_good),              TransmissionInterfaceException);
  EXPECT_THROW(DifferentialTransmission(reduction_good, reduction_bad_size),              TransmissionInterfaceException);
  EXPECT_THROW(DifferentialTransmission(reduction_good, reduction_good, offset_bad_size), TransmissionInterfaceException);

  // Valid instance creation
  EXPECT_NO_THROW(DifferentialTransmission(reduction_good, reduction_good));
  EXPECT_NO_THROW(DifferentialTransmission(reduction_good, reduction_good, offset_good));
}

#ifndef NDEBUG // NOTE: This test validates assertion triggering, hence only gets compiled in debug mode
TEST(PreconditionsTest, AssertionTriggering)
{
  // Create input/output transmission data
  double a_val1 = 0.0, a_val2 = 0.0;
  double j_val1 = 0.0, j_val2 = 0.0;

  vector<double*> a_good_vec;
  a_good_vec.push_back(&a_val1);
  a_good_vec.push_back(&a_val2);

  vector<double*> j_good_vec;
  j_good_vec.push_back(&j_val1);
  j_good_vec.push_back(&j_val2);

  ActuatorData a_good_data;
  a_good_data.position = a_good_vec;
  a_good_data.velocity = a_good_vec;
  a_good_data.effort   = a_good_vec;

  JointData j_good_data;
  j_good_data.position = j_good_vec;
  j_good_data.velocity = j_good_vec;
  j_good_data.effort   = j_good_vec;

  ActuatorData a_bad_data;
  a_bad_data.position = vector<double*>(2);
  a_bad_data.velocity = vector<double*>(2);
  a_bad_data.effort   = vector<double*>(2);

  JointData j_bad_data;
  j_bad_data.position = vector<double*>(2);
  j_bad_data.velocity = vector<double*>(2);
  j_bad_data.effort   = vector<double*>(2);

  ActuatorData a_bad_size;
  JointData    j_bad_size;

  // Transmission instance
  DifferentialTransmission trans(vector<double>(2, 1.0),
                                 vector<double>(2, 1.0));

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
  std::vector<double> act_reduction(2);
  act_reduction[0] =  2.0;
  act_reduction[1] = -2.0;

  std::vector<double> jnt_reduction(2);
  jnt_reduction[0] =  4.0;
  jnt_reduction[1] = -4.0;

  std::vector<double> jnt_offset(2);
  jnt_offset[0] =  1.0;
  jnt_offset[1] = -1.0;

  DifferentialTransmission trans(act_reduction,
                                 jnt_reduction,
                                 jnt_offset);

  EXPECT_EQ(2, trans.numActuators());
  EXPECT_EQ(2, trans.numJoints());
  EXPECT_EQ( 2.0, trans.getActuatorReduction()[0]);
  EXPECT_EQ(-2.0, trans.getActuatorReduction()[1]);
  EXPECT_EQ( 4.0, trans.getJointReduction()[0]);
  EXPECT_EQ(-4.0, trans.getJointReduction()[1]);
  EXPECT_EQ( 1.0, trans.getJointOffset()[0]);
  EXPECT_EQ(-1.0, trans.getJointOffset()[1]);
}

class TransmissionSetup : public ::testing::Test
{
public:
  TransmissionSetup()
    : a_val(),
      j_val(),
      a_vec(vector<double*>(2)),
      j_vec(vector<double*>(2))
   {
     a_vec[0] = &a_val[0];
     a_vec[1] = &a_val[1];
     j_vec[0] = &j_val[0];
     j_vec[1] = &j_val[1];
   }

protected:
  // Input/output transmission data
  double a_val[2];
  double j_val[2];
  vector<double*> a_vec;
  vector<double*> j_vec;
};

/// \brief Exercises the actuator->joint->actuator roundtrip, which should yield the identity map.
class BlackBoxTest : public TransmissionSetup
{
protected:
  /// \param trans Transmission instance.
  /// \param ref_val Reference value (effort, velocity or position) that will be transformed with the respective forward
  /// and inverse transmission transformations.
  void testIdentityMap(DifferentialTransmission& trans,
                       const vector<double>& ref_val)
  {
    // Effort interface
    {
      ActuatorData a_data;
      a_data.effort = a_vec;
      *a_data.effort[0] = ref_val[0];
      *a_data.effort[1] = ref_val[1];

      JointData j_data;
      j_data.effort = j_vec;

      trans.actuatorToJointEffort(a_data, j_data);
      trans.jointToActuatorEffort(j_data, a_data);
      EXPECT_NEAR(ref_val[0], *a_data.effort[0], EPS);
      EXPECT_NEAR(ref_val[1], *a_data.effort[1], EPS);
    }

    // Velocity interface
    {
      ActuatorData a_data;
      a_data.velocity = a_vec;
      *a_data.velocity[0] = ref_val[0];
      *a_data.velocity[1] = ref_val[1];

      JointData j_data;
      j_data.velocity = j_vec;

      trans.actuatorToJointVelocity(a_data, j_data);
      trans.jointToActuatorVelocity(j_data, a_data);
      EXPECT_NEAR(ref_val[0], *a_data.velocity[0], EPS);
      EXPECT_NEAR(ref_val[1], *a_data.velocity[1], EPS);
    }

    // Position interface
    {
      ActuatorData a_data;
      a_data.position = a_vec;
      *a_data.position[0] = ref_val[0];
      *a_data.position[1] = ref_val[1];

      JointData j_data;
      j_data.position = j_vec;

      trans.actuatorToJointPosition(a_data, j_data);
      trans.jointToActuatorPosition(j_data, a_data);
      EXPECT_NEAR(ref_val[0], *a_data.position[0], EPS);
      EXPECT_NEAR(ref_val[1], *a_data.position[1], EPS);
    }
  }

  /// Generate a set of transmission instances with random combinations of actuator/joint reduction and joint offset.
  static vector<DifferentialTransmission> createTestInstances(const vector<DifferentialTransmission>::size_type size)
  {
    std::vector<DifferentialTransmission> out;
    out.reserve(size);
    RandomDoubleGenerator rand_gen(-1000.0, 1000.0);                                // NOTE: Magic value

    while (out.size() < size)
    {
      try
      {
        DifferentialTransmission trans(randomVector(2, rand_gen),
                                       randomVector(2, rand_gen),
                                       randomVector(2, rand_gen));
        out.push_back(trans);
      }
      catch(const TransmissionInterfaceException&)
      {
        // NOTE: If by chance a perfect zero is produced by the random number generator, construction will fail
        // We swallow the exception and move on to prevent a test crash.
      }
    }
    return out;
  }
};

TEST_F(BlackBoxTest, IdentityMap)
{
  // Transmission instances
  typedef vector<DifferentialTransmission> TransList;
  TransList trans_list = createTestInstances(100);                                  // NOTE: Magic value

  // Test different transmission configurations...
  for (TransList::iterator it = trans_list.begin(); it != trans_list.end(); ++it)
  {
    // ...and for each transmission, different input values
    RandomDoubleGenerator rand_gen(-1000.0, 1000.0);                                // NOTE: Magic value
    const unsigned int input_value_trials = 100;                                    // NOTE: Magic value
    for (unsigned int i = 0; i < input_value_trials; ++i)
    {
      vector<double> input_value = randomVector(2, rand_gen);
      testIdentityMap(*it, input_value);
    }
  }
}


class WhiteBoxTest : public TransmissionSetup {};

TEST_F(WhiteBoxTest, DontMoveJoints)
{
  vector<double> actuator_reduction(2, 10.0);
  vector<double> joint_reduction(2, 2.0);
  vector<double> joint_offset(2, 1.0);

  DifferentialTransmission trans(actuator_reduction, joint_reduction, joint_offset);

  // Actuator input (used for effort, velocity and position)
  *a_vec[0] = 0.0;
  *a_vec[1] = 0.0;

  // Effort interface
  {
    ActuatorData a_data;
    a_data.effort = a_vec;

    JointData j_data;
    j_data.effort = j_vec;

    trans.actuatorToJointEffort(a_data, j_data);
    EXPECT_NEAR(0.0, *j_data.effort[0], EPS);
    EXPECT_NEAR(0.0, *j_data.effort[1], EPS);
  }

  // Velocity interface
  {
    ActuatorData a_data;
    a_data.velocity = a_vec;

    JointData j_data;
    j_data.velocity = j_vec;

    trans.actuatorToJointVelocity(a_data, j_data);
    EXPECT_NEAR(0.0, *j_data.velocity[0], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[1], EPS);
  }

  // Position interface
  {
    ActuatorData a_data;
    a_data.position = a_vec;

    JointData j_data;
    j_data.position = j_vec;

    trans.actuatorToJointPosition(a_data, j_data);
    EXPECT_NEAR(joint_offset[0], *j_data.position[0], EPS);
    EXPECT_NEAR(joint_offset[1], *j_data.position[1], EPS);
  }
}

TEST_F(WhiteBoxTest, MoveFirstJointOnly)
{
  vector<double> actuator_reduction(2, 10.0);
  vector<double> joint_reduction(2, 2.0);

  DifferentialTransmission trans(actuator_reduction, joint_reduction);

  // Actuator input (used for effort, velocity and position)
  *a_vec[0] = 10.0;
  *a_vec[1] = 10.0;

  // Effort interface
  {
    ActuatorData a_data;
    a_data.effort = a_vec;

    JointData j_data;
    j_data.effort = j_vec;

    trans.actuatorToJointEffort(a_data, j_data);
    EXPECT_NEAR(400.0, *j_data.effort[0], EPS);
    EXPECT_NEAR(0.0, *j_data.effort[1], EPS);
  }

  // Velocity interface
  {
    ActuatorData a_data;
    a_data.velocity = a_vec;

    JointData j_data;
    j_data.velocity = j_vec;

    trans.actuatorToJointVelocity(a_data, j_data);
    EXPECT_NEAR(0.5, *j_data.velocity[0], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[1], EPS);
  }

  // Position interface
  {
    ActuatorData a_data;
    a_data.position = a_vec;

    JointData j_data;
    j_data.position = j_vec;

    trans.actuatorToJointPosition(a_data, j_data);
    EXPECT_NEAR(0.5, *j_data.position[0], EPS);
    EXPECT_NEAR(0.0, *j_data.position[1], EPS);
  }
}

TEST_F(WhiteBoxTest, MoveSecondJointOnly)
{
  vector<double> actuator_reduction(2, 10.0);
  vector<double> joint_reduction(2, 2.0);

  DifferentialTransmission trans(actuator_reduction, joint_reduction);

  // Actuator input (used for effort, velocity and position)
  *a_vec[0] =  10.0;
  *a_vec[1] = -10.0;

  // Effort interface
  {
    ActuatorData a_data;
    a_data.effort = a_vec;

    JointData j_data;
    j_data.effort = j_vec;

    trans.actuatorToJointEffort(a_data, j_data);
    EXPECT_NEAR(0.0, *j_data.effort[0], EPS);
    EXPECT_NEAR(400.0, *j_data.effort[1], EPS);
  }

  // Velocity interface
  {
    ActuatorData a_data;
    a_data.velocity = a_vec;

    JointData j_data;
    j_data.velocity = j_vec;

    trans.actuatorToJointVelocity(a_data, j_data);
    EXPECT_NEAR(0.0, *j_data.velocity[0], EPS);
    EXPECT_NEAR(0.5, *j_data.velocity[1], EPS);
  }

  // Position interface
  {
    ActuatorData a_data;
    a_data.position = a_vec;

    JointData j_data;
    j_data.position = j_vec;

    trans.actuatorToJointPosition(a_data, j_data);
    EXPECT_NEAR(0.0, *j_data.position[0], EPS);
    EXPECT_NEAR(0.5, *j_data.position[1], EPS);
  }
}

TEST_F(WhiteBoxTest, MoveBothJoints)
{
  // NOTE: We only test the actuator->joint map, as the joint->actuator map is indirectly validated in the test that
  // checks that actuator->joint->actuator == identity.

  vector<double> actuator_reduction(2);
  actuator_reduction[0] =  10.0;
  actuator_reduction[1] = -20.0;

  vector<double> joint_reduction(2);
  joint_reduction[0] = -2.0;
  joint_reduction[1] =  4.0;

  vector<double> joint_offset(2);
  joint_offset[0] = -2.0;
  joint_offset[1] =  4.0;

  DifferentialTransmission trans(actuator_reduction, joint_reduction, joint_offset);

  // Actuator input (used for effort, velocity and position)
  *a_vec[0] = 3.0;
  *a_vec[1] = 5.0;

  // Effort interface
  {
    ActuatorData a_data;
    a_data.effort = a_vec;

    JointData j_data;
    j_data.effort = j_vec;

    trans.actuatorToJointEffort(a_data, j_data);
    EXPECT_NEAR(140.0, *j_data.effort[0], EPS);
    EXPECT_NEAR(520.0, *j_data.effort[1], EPS);
  }

  // Velocity interface
  {
    ActuatorData a_data;
    a_data.velocity = a_vec;

    JointData j_data;
    j_data.velocity = j_vec;

    trans.actuatorToJointVelocity(a_data, j_data);
    EXPECT_NEAR(-0.01250, *j_data.velocity[0], EPS);
    EXPECT_NEAR( 0.06875, *j_data.velocity[1], EPS);
  }

  // Position interface
  {
    ActuatorData a_data;
    a_data.position = a_vec;

    JointData j_data;
    j_data.position = j_vec;

    trans.actuatorToJointPosition(a_data, j_data);
    EXPECT_NEAR(-2.01250, *j_data.position[0], EPS);
    EXPECT_NEAR( 4.06875, *j_data.position[1], EPS);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
