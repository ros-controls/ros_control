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

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/transmission_interface.h>

using std::vector;
using std::string;
using namespace transmission_interface;

// Floating-point value comparison threshold
const double EPS = 1e-6;

TEST(PreconditionsTest, ExceptionThrowing)
{
  double val = 0.0;
  vector<double*> good_data(1, &val);
  vector<double*> bad_data(1);
  vector<double*> bad_size(2, &val);

  SimpleTransmission trans(1.0);
  ActuatorToJointPositionInterface trans_iface;

  // Invalid transmission registration: Invalid pointers
  EXPECT_THROW(trans_iface.registerTransmission("dummy_trans", 0, good_data, good_data), TransmissionException);
  EXPECT_THROW(trans_iface.registerTransmission("dummy_trans", &trans, bad_data, good_data), TransmissionException);
  EXPECT_THROW(trans_iface.registerTransmission("dummy_trans", &trans, good_data, bad_data), TransmissionException);

  // Invalid transmission registration: Invalid data vector sizes
  EXPECT_THROW(trans_iface.registerTransmission("dummy_trans", &trans, bad_size, good_data), TransmissionException);
  EXPECT_THROW(trans_iface.registerTransmission("dummy_trans", &trans, good_data, bad_size), TransmissionException);

  // Valid instance creation
  EXPECT_NO_THROW(trans_iface.registerTransmission("dummy_trans", &trans, good_data, good_data));
}

class TransmissionInterfaceSetup : public ::testing::Test
{
public:
  TransmissionInterfaceSetup()
    : a_curr(),
      j_curr(),
      a_cmd(),
      j_cmd(),
      trans1( 10.0, 1.0),
      trans2(-10.0, 1.0) {}

protected:
  // Input/output transmission data
  double a_curr[2];
  double j_curr[2];
  double a_cmd[2];
  double j_cmd[2];

  SimpleTransmission trans1;
  SimpleTransmission trans2;
};

class AccessorTest : public TransmissionInterfaceSetup {};

TEST_F(AccessorTest, AccessorValidation)
{
  ActuatorToJointPositionInterface trans_iface;
  trans_iface.registerTransmission("trans_1", &trans1, vector<double*>(1, &a_curr[0]), vector<double*>(1, &j_curr[0]));
  trans_iface.registerTransmission("trans_2", &trans2, vector<double*>(1, &a_curr[1]), vector<double*>(1, &j_curr[1]));

  // Retrieve transmission names
  vector<string> trans_names = trans_iface.getTransmissionNames();
  ASSERT_EQ(2, trans_names.size());

  EXPECT_EQ("trans_1", trans_names[0]);
  EXPECT_EQ("trans_2", trans_names[1]);

  // Retrieve transmission handles
  ActuatorToJointPositionHandle trans_handle1 = trans_iface.getTransmissionHandle(trans_names[0]);
  EXPECT_EQ(trans_names[0], trans_handle1.getName());

  ActuatorToJointPositionHandle trans_handle2 = trans_iface.getTransmissionHandle(trans_names[1]);
  EXPECT_EQ(trans_names[1], trans_handle2.getName());

  EXPECT_THROW(trans_iface.getTransmissionHandle("unregistered_name"), TransmissionException);
}

class HandleWhiteBoxTest : public TransmissionInterfaceSetup {};

TEST_F(HandleWhiteBoxTest, PositionMap)
{
  // Actuator -> joint
  ActuatorToJointPositionInterface to_jnt_pos;
  to_jnt_pos.registerTransmission("trans_1", &trans1, vector<double*>(1, &a_curr[0]), vector<double*>(1, &j_curr[0]));
  ActuatorToJointPositionHandle to_jnt_pos_handle = to_jnt_pos.getTransmissionHandle("trans_1");

  // Joint -> actuator
  JointToActuatorPositionInterface to_act_pos;
  to_act_pos.registerTransmission("trans_1", &trans1, vector<double*>(1, &a_cmd[0]), vector<double*>(1, &j_cmd[0]));
  JointToActuatorPositionHandle to_act_pos_handle = to_act_pos.getTransmissionHandle("trans_1");

  // Identity map
  a_curr[0] = 1.0;
  to_jnt_pos_handle.propagate();
  EXPECT_NEAR(1.1, j_curr[0], EPS);

  j_cmd[0] = j_curr[0];
  to_act_pos_handle.propagate();
  EXPECT_NEAR(a_curr[0], a_cmd[0], EPS);
}

TEST_F(HandleWhiteBoxTest, VelocityMap)
{
  // Actuator -> joint
  ActuatorToJointVelocityInterface to_jnt_vel;
  to_jnt_vel.registerTransmission("trans_1", &trans1, vector<double*>(1, &a_curr[0]), vector<double*>(1, &j_curr[0]));
  ActuatorToJointVelocityHandle to_jnt_vel_handle = to_jnt_vel.getTransmissionHandle("trans_1");

  // Joint -> actuator
  JointToActuatorVelocityInterface to_act_vel;
  to_act_vel.registerTransmission("trans_1", &trans1, vector<double*>(1, &a_cmd[0]), vector<double*>(1, &j_cmd[0]));
  JointToActuatorVelocityHandle to_act_vel_handle = to_act_vel.getTransmissionHandle("trans_1");

  // Identity map
  a_curr[0] = 1.0;
  to_jnt_vel_handle.propagate();
  EXPECT_NEAR(0.1, j_curr[0], EPS);

  j_cmd[0] = j_curr[0];
  to_act_vel_handle.propagate();
  EXPECT_NEAR(a_curr[0], a_cmd[0], EPS);
}

TEST_F(HandleWhiteBoxTest, EffortMap)
{
  // Actuator -> joint
  ActuatorToJointEffortInterface to_jnt_eff;
  to_jnt_eff.registerTransmission("trans_1", &trans1, vector<double*>(1, &a_curr[0]), vector<double*>(1, &j_curr[0]));
  ActuatorToJointEffortHandle to_jnt_eff_handle = to_jnt_eff.getTransmissionHandle("trans_1");

  // Joint -> actuator
  JointToActuatorEffortInterface to_act_eff;
  to_act_eff.registerTransmission("trans_1", &trans1, vector<double*>(1, &a_cmd[0]), vector<double*>(1, &j_cmd[0]));
  JointToActuatorEffortHandle to_act_eff_handle = to_act_eff.getTransmissionHandle("trans_1");

  // Identity map
  a_curr[0] = 1.0;
  to_jnt_eff_handle.propagate();
  EXPECT_NEAR(10.0, j_curr[0], EPS);

  j_cmd[0] = j_curr[0];
  to_act_eff_handle.propagate();
  EXPECT_NEAR(a_curr[0], a_cmd[0], EPS);
}

class InterfaceWhiteBoxTest : public TransmissionInterfaceSetup {};

TEST_F(InterfaceWhiteBoxTest, PositionMap)
{
  // Actuator -> joint
  ActuatorToJointPositionInterface to_jnt_pos;
  to_jnt_pos.registerTransmission("trans_1", &trans1, vector<double*>(1, &a_curr[0]), vector<double*>(1, &j_curr[0]));
  to_jnt_pos.registerTransmission("trans_2", &trans2, vector<double*>(1, &a_curr[1]), vector<double*>(1, &j_curr[1]));

  // Joint -> actuator
  JointToActuatorPositionInterface to_act_pos;
  to_act_pos.registerTransmission("trans_1", &trans1, vector<double*>(1, &a_cmd[0]), vector<double*>(1, &j_cmd[0]));
  to_act_pos.registerTransmission("trans_2", &trans2, vector<double*>(1, &a_cmd[1]), vector<double*>(1, &j_cmd[1]));

  // Identity map
  a_curr[0] = 1.0;
  a_curr[1] = 1.0;
  to_jnt_pos.propagate();
  EXPECT_NEAR(1.1, j_curr[0], EPS);
  EXPECT_NEAR(0.9, j_curr[1], EPS);

  j_cmd[0] = j_curr[0];
  j_cmd[1] = j_curr[1];
  to_act_pos.propagate();
  EXPECT_NEAR(a_curr[0], a_cmd[0], EPS);
  EXPECT_NEAR(a_curr[1], a_cmd[1], EPS);
}

TEST_F(InterfaceWhiteBoxTest, VelocityMap)
{
  // Actuator -> joint
  ActuatorToJointVelocityInterface to_jnt_vel;
  to_jnt_vel.registerTransmission("trans_1", &trans1, vector<double*>(1, &a_curr[0]), vector<double*>(1, &j_curr[0]));
  to_jnt_vel.registerTransmission("trans_2", &trans2, vector<double*>(1, &a_curr[1]), vector<double*>(1, &j_curr[1]));

  // Joint -> actuator
  JointToActuatorVelocityInterface to_act_vel;
  to_act_vel.registerTransmission("trans_1", &trans1, vector<double*>(1, &a_cmd[0]), vector<double*>(1, &j_cmd[0]));
  to_act_vel.registerTransmission("trans_2", &trans2, vector<double*>(1, &a_cmd[1]), vector<double*>(1, &j_cmd[1]));

  // Identity map
  a_curr[0] = 1.0;
  a_curr[1] = 1.0;
  to_jnt_vel.propagate();
  EXPECT_NEAR( 0.1, j_curr[0], EPS);
  EXPECT_NEAR(-0.1, j_curr[1], EPS);

  j_cmd[0] = j_curr[0];
  j_cmd[1] = j_curr[1];
  to_act_vel.propagate();
  EXPECT_NEAR(a_curr[0], a_cmd[0], EPS);
  EXPECT_NEAR(a_curr[1], a_cmd[1], EPS);
}

TEST_F(InterfaceWhiteBoxTest, EffortMap)
{
  // Actuator -> joint
  ActuatorToJointEffortInterface to_jnt_eff;
  to_jnt_eff.registerTransmission("trans_1", &trans1, vector<double*>(1, &a_curr[0]), vector<double*>(1, &j_curr[0]));
  to_jnt_eff.registerTransmission("trans_2", &trans2, vector<double*>(1, &a_curr[1]), vector<double*>(1, &j_curr[1]));

  // Joint -> actuator
  JointToActuatorEffortInterface to_act_eff;
  to_act_eff.registerTransmission("trans_1", &trans1, vector<double*>(1, &a_cmd[0]), vector<double*>(1, &j_cmd[0]));
  to_act_eff.registerTransmission("trans_2", &trans2, vector<double*>(1, &a_cmd[1]), vector<double*>(1, &j_cmd[1]));

  // Identity map
  a_curr[0] = 1.0;
  a_curr[1] = 1.0;
  to_jnt_eff.propagate();
  EXPECT_NEAR( 10.0, j_curr[0], EPS);
  EXPECT_NEAR(-10.0, j_curr[1], EPS);

  j_cmd[0] = j_curr[0];
  j_cmd[1] = j_curr[1];
  to_act_eff.propagate();
  EXPECT_NEAR(a_curr[0], a_cmd[0], EPS);
  EXPECT_NEAR(a_curr[1], a_cmd[1], EPS);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
