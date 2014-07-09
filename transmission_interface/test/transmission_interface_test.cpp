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

class DummyHandle : public TransmissionHandle
{
public:
  DummyHandle(const std::string&  name,
              Transmission*       transmission,
              const ActuatorData& actuator_data,
              const JointData&    joint_data)
    : TransmissionHandle(name, transmission, actuator_data, joint_data) {}
};

TEST(HandlePreconditionsTest, ValidHandle)
{
  double val = 0.0;
  vector<double*> good_vec(1, &val);
  SimpleTransmission trans(1.0);

  {
    ActuatorData a_data;
    JointData    j_data;
    a_data.position = good_vec;
    EXPECT_NO_THROW(DummyHandle("trans", &trans, a_data, j_data));
  }
  {
    ActuatorData a_data;
    JointData    j_data;
    a_data.velocity = good_vec;
    EXPECT_NO_THROW(DummyHandle("trans", &trans, a_data, j_data));
  }
  {
    ActuatorData a_data;
    JointData    j_data;
    a_data.effort = good_vec;
    EXPECT_NO_THROW(DummyHandle("trans", &trans, a_data, j_data));
  }
  {
    ActuatorData a_data;
    JointData    j_data;
    j_data.position = good_vec;
    EXPECT_NO_THROW(DummyHandle("trans", &trans, a_data, j_data));
  }
  {
    ActuatorData a_data;
    JointData    j_data;
    j_data.velocity = good_vec;
    EXPECT_NO_THROW(DummyHandle("trans", &trans, a_data, j_data));
  }
  {
    ActuatorData a_data;
    JointData    j_data;
    j_data.effort = good_vec;
    EXPECT_NO_THROW(DummyHandle("trans", &trans, a_data, j_data));
  }
  {
    ActuatorData a_data;
    JointData    j_data;
    a_data.position = good_vec;
    a_data.velocity = good_vec;
    a_data.effort   = good_vec;
    j_data.position = good_vec;
    j_data.velocity = good_vec;
    j_data.effort   = good_vec;
    EXPECT_NO_THROW(DummyHandle("trans", &trans, a_data, j_data));
  }
}

TEST(HandlePreconditionsTest, BadSize)
{
  double val = 0.0;
  vector<double*> bad_size_vector(2, &val);
  SimpleTransmission trans(1.0);

  {
    ActuatorData a_data;
    JointData    j_data;
    a_data.position = bad_size_vector;
    EXPECT_THROW(DummyHandle("trans", &trans, a_data, j_data), TransmissionInterfaceException);
  }
  {
    ActuatorData a_data;
    JointData    j_data;
    a_data.velocity = bad_size_vector;
    EXPECT_THROW(DummyHandle("trans", &trans, a_data, j_data), TransmissionInterfaceException);
  }
  {
    ActuatorData a_data;
    JointData    j_data;
    a_data.effort = bad_size_vector;
    EXPECT_THROW(DummyHandle("trans", &trans, a_data, j_data), TransmissionInterfaceException);
  }
  {
    ActuatorData a_data;
    JointData    j_data;
    j_data.position = bad_size_vector;
    EXPECT_THROW(DummyHandle("trans", &trans, a_data, j_data), TransmissionInterfaceException);
  }
  {
    ActuatorData a_data;
    JointData    j_data;
    j_data.velocity = bad_size_vector;
    EXPECT_THROW(DummyHandle("trans", &trans, a_data, j_data), TransmissionInterfaceException);
  }
  {
    ActuatorData a_data;
    JointData    j_data;
    j_data.effort = bad_size_vector;
    EXPECT_THROW(DummyHandle("trans", &trans, a_data, j_data), TransmissionInterfaceException);
  }
}

TEST(HandlePreconditionsTest, EmptyData)
{
  SimpleTransmission trans(1.0);

  EXPECT_THROW(DummyHandle("trans", &trans, ActuatorData(), JointData()), TransmissionInterfaceException);
}

TEST(HandlePreconditionsTest, BadTransmissionPointer)
{
  double val = 0.0;
  vector<double*> good_vec(1, &val);
  ActuatorData a_data;
  JointData    j_data;
  a_data.position = good_vec;
  j_data.position = good_vec;

  EXPECT_THROW(DummyHandle("trans", 0, a_data, j_data), TransmissionInterfaceException);
}

TEST(HandlePreconditionsTest, BadDataPointer)
{
  vector<double*> bad_ptr_vector(1);
  SimpleTransmission trans(1.0);

  {
    ActuatorData a_data;
    JointData    j_data;
    a_data.position = bad_ptr_vector;
    EXPECT_THROW(DummyHandle("trans", &trans, a_data, j_data), TransmissionInterfaceException);
  }
  {
    ActuatorData a_data;
    JointData    j_data;
    a_data.velocity = bad_ptr_vector;
    EXPECT_THROW(DummyHandle("trans", &trans, a_data, j_data), TransmissionInterfaceException);
  }
  {
    ActuatorData a_data;
    JointData    j_data;
    a_data.effort = bad_ptr_vector;
    EXPECT_THROW(DummyHandle("trans", &trans, a_data, j_data), TransmissionInterfaceException);
  }
  {
    ActuatorData a_data;
    JointData    j_data;
    j_data.position = bad_ptr_vector;
    EXPECT_THROW(DummyHandle("trans", &trans, a_data, j_data), TransmissionInterfaceException);
  }
  {
    ActuatorData a_data;
    JointData    j_data;
    j_data.velocity = bad_ptr_vector;
    EXPECT_THROW(DummyHandle("trans", &trans, a_data, j_data), TransmissionInterfaceException);
  }
  {
    ActuatorData a_data;
    JointData    j_data;
    j_data.effort = bad_ptr_vector;
    EXPECT_THROW(DummyHandle("trans", &trans, a_data, j_data), TransmissionInterfaceException);
  }
}

class TransmissionInterfaceSetup : public ::testing::Test
{
public:
  TransmissionInterfaceSetup()
    : a_curr_pos(), a_curr_vel(), a_curr_eff(),
      j_curr_pos(), j_curr_vel(), j_curr_eff(),
      a_cmd_pos(), a_cmd_vel(), a_cmd_eff(),
      j_cmd_pos(), j_cmd_vel(), j_cmd_eff(),
      trans1( 10.0, 1.0),
      trans2(-10.0, 1.0) {}

protected:
  // Input/output transmission data
  double a_curr_pos[2], a_curr_vel[2], a_curr_eff[2];
  double j_curr_pos[2], j_curr_vel[2], j_curr_eff[2];

  double a_cmd_pos[2], a_cmd_vel[2], a_cmd_eff[2];
  double j_cmd_pos[2], j_cmd_vel[2], j_cmd_eff[2];

  SimpleTransmission trans1;
  SimpleTransmission trans2;
};

class HandleWhiteBoxTest : public TransmissionInterfaceSetup {};

TEST_F(HandleWhiteBoxTest, PositionMap)
{
  // Actuator -> joint
  ActuatorData a_curr_data;
  JointData    j_curr_data;
  a_curr_data.position = vector<double*>(1, &a_curr_pos[0]);
  j_curr_data.position = vector<double*>(1, &j_curr_pos[0]);

  ActuatorToJointPositionInterface to_jnt_pos;
  to_jnt_pos.registerHandle(ActuatorToJointPositionHandle("trans_1", &trans1, a_curr_data, j_curr_data));
  ActuatorToJointPositionHandle to_jnt_pos_handle = to_jnt_pos.getHandle("trans_1");

  // Joint -> actuator
  ActuatorData a_cmd_data;
  JointData    j_cmd_data;
  a_cmd_data.position = vector<double*>(1, &a_cmd_pos[0]);
  j_cmd_data.position = vector<double*>(1, &j_cmd_pos[0]);

  JointToActuatorPositionInterface to_act_pos;
  to_act_pos.registerHandle(JointToActuatorPositionHandle("trans_1", &trans1, a_cmd_data, j_cmd_data));
  JointToActuatorPositionHandle to_act_pos_handle = to_act_pos.getHandle("trans_1");

  // Identity map
  a_curr_pos[0] = 1.0;
  to_jnt_pos_handle.propagate();
  EXPECT_NEAR(1.1, j_curr_pos[0], EPS);

  j_cmd_pos[0] = j_curr_pos[0];
  to_act_pos_handle.propagate();
  EXPECT_NEAR(a_curr_pos[0], a_cmd_pos[0], EPS);
}

TEST_F(HandleWhiteBoxTest, VelocityMap)
{
  // Actuator -> joint
  ActuatorData a_curr_data;
  JointData    j_curr_data;
  a_curr_data.velocity = vector<double*>(1, &a_curr_vel[0]);
  j_curr_data.velocity = vector<double*>(1, &j_curr_vel[0]);

  ActuatorToJointVelocityInterface to_jnt_vel;
  to_jnt_vel.registerHandle(ActuatorToJointVelocityHandle("trans_1", &trans1, a_curr_data, j_curr_data));
  ActuatorToJointVelocityHandle to_jnt_vel_handle = to_jnt_vel.getHandle("trans_1");

  // Joint -> actuator
  ActuatorData a_cmd_data;
  JointData    j_cmd_data;
  a_cmd_data.velocity = vector<double*>(1, &a_cmd_vel[0]);
  j_cmd_data.velocity = vector<double*>(1, &j_cmd_vel[0]);

  JointToActuatorVelocityInterface to_act_vel;
  to_act_vel.registerHandle(JointToActuatorVelocityHandle("trans_1", &trans1, a_cmd_data, j_cmd_data));
  JointToActuatorVelocityHandle to_act_vel_handle = to_act_vel.getHandle("trans_1");

  // Identity map
  a_curr_vel[0] = 1.0;
  to_jnt_vel_handle.propagate();
  EXPECT_NEAR(0.1, j_curr_vel[0], EPS);

  j_cmd_vel[0] = j_curr_vel[0];
  to_act_vel_handle.propagate();
  EXPECT_NEAR(a_curr_vel[0], a_cmd_vel[0], EPS);
}

TEST_F(HandleWhiteBoxTest, EffortMap)
{
  // Actuator -> joint
  ActuatorData a_curr_data;
  JointData    j_curr_data;
  a_curr_data.effort = vector<double*>(1, &a_curr_eff[0]);
  j_curr_data.effort = vector<double*>(1, &j_curr_eff[0]);

  ActuatorToJointEffortInterface to_jnt_eff;
  to_jnt_eff.registerHandle(ActuatorToJointEffortHandle("trans_1", &trans1, a_curr_data, j_curr_data));
  ActuatorToJointEffortHandle to_jnt_eff_handle = to_jnt_eff.getHandle("trans_1");

  // Joint -> actuator
  ActuatorData a_cmd_data;
  JointData    j_cmd_data;
  a_cmd_data.effort = vector<double*>(1, &a_cmd_eff[0]);
  j_cmd_data.effort = vector<double*>(1, &j_cmd_eff[0]);

  JointToActuatorEffortInterface to_act_eff;
  to_act_eff.registerHandle(JointToActuatorEffortHandle("trans_1", &trans1, a_cmd_data, j_cmd_data));
  JointToActuatorEffortHandle to_act_eff_handle = to_act_eff.getHandle("trans_1");

  // Identity map
  a_curr_eff[0] = 1.0;
  to_jnt_eff_handle.propagate();
  EXPECT_NEAR(10.0, j_curr_eff[0], EPS);

  j_cmd_eff[0] = j_curr_eff[0];
  to_act_eff_handle.propagate();
  EXPECT_NEAR(a_curr_eff[0], a_cmd_eff[0], EPS);
}

TEST_F(HandleWhiteBoxTest, StateMap)
{
  // Actuator -> joint
  ActuatorData a_curr_data;
  a_curr_data.position = vector<double*>(1, &a_curr_pos[0]);
  a_curr_data.velocity = vector<double*>(1, &a_curr_vel[0]);
  a_curr_data.effort   = vector<double*>(1, &a_curr_eff[0]);

  JointData j_curr_data;
  j_curr_data.position = vector<double*>(1, &j_curr_pos[0]);
  j_curr_data.velocity = vector<double*>(1, &j_curr_vel[0]);
  j_curr_data.effort   = vector<double*>(1, &j_curr_eff[0]);

  ActuatorToJointStateInterface to_jnt_state;
  to_jnt_state.registerHandle(ActuatorToJointStateHandle("trans_1", &trans1, a_curr_data, j_curr_data));
  ActuatorToJointStateHandle to_jnt_state_handle = to_jnt_state.getHandle("trans_1");

  // Joint -> actuator
  ActuatorData a_cmd_data;
  a_cmd_data.position = vector<double*>(1, &a_cmd_pos[0]);
  a_cmd_data.velocity = vector<double*>(1, &a_cmd_vel[0]);
  a_cmd_data.effort   = vector<double*>(1, &a_cmd_eff[0]);

  JointData j_cmd_data;
  j_cmd_data.position = vector<double*>(1, &j_cmd_pos[0]);
  j_cmd_data.velocity = vector<double*>(1, &j_cmd_vel[0]);
  j_cmd_data.effort   = vector<double*>(1, &j_cmd_eff[0]);

  JointToActuatorStateInterface to_act_state;
  to_act_state.registerHandle(JointToActuatorStateHandle("trans_1", &trans1, a_cmd_data, j_cmd_data));
  JointToActuatorStateHandle to_act_state_handle = to_act_state.getHandle("trans_1");

  // Identity map
  a_curr_pos[0] = 1.0;
  a_curr_vel[0] = 1.0;
  a_curr_eff[0] = 1.0;
  to_jnt_state_handle.propagate();
  EXPECT_NEAR( 1.1, j_curr_pos[0], EPS);
  EXPECT_NEAR( 0.1, j_curr_vel[0], EPS);
  EXPECT_NEAR(10.0, j_curr_eff[0], EPS);

  j_cmd_pos[0] = j_curr_pos[0];
  j_cmd_vel[0] = j_curr_vel[0];
  j_cmd_eff[0] = j_curr_eff[0];
  to_act_state_handle.propagate();
  EXPECT_NEAR(a_curr_pos[0], a_cmd_pos[0], EPS);
  EXPECT_NEAR(a_curr_vel[0], a_cmd_vel[0], EPS);
  EXPECT_NEAR(a_curr_eff[0], a_cmd_eff[0], EPS);
}

class InterfaceWhiteBoxTest : public TransmissionInterfaceSetup {};

TEST_F(InterfaceWhiteBoxTest, PositionMap)
{
  // Actuator -> joint
  ActuatorData a_curr_data[2];
  a_curr_data[0].position = vector<double*>(1, &a_curr_pos[0]);
  a_curr_data[1].position = vector<double*>(1, &a_curr_pos[1]);

  JointData j_curr_data[2];
  j_curr_data[0].position = vector<double*>(1, &j_curr_pos[0]);
  j_curr_data[1].position = vector<double*>(1, &j_curr_pos[1]);

  ActuatorToJointPositionInterface to_jnt_pos;
  to_jnt_pos.registerHandle(ActuatorToJointPositionHandle("trans_1", &trans1, a_curr_data[0], j_curr_data[0]));
  to_jnt_pos.registerHandle(ActuatorToJointPositionHandle("trans_2", &trans2, a_curr_data[1], j_curr_data[1]));

  // Joint -> actuator
  ActuatorData a_cmd_data[2];
  a_cmd_data[0].position = vector<double*>(1, &a_cmd_pos[0]);
  a_cmd_data[1].position = vector<double*>(1, &a_cmd_pos[1]);

  JointData j_cmd_data[2];
  j_cmd_data[0].position = vector<double*>(1, &j_cmd_pos[0]);
  j_cmd_data[1].position = vector<double*>(1, &j_cmd_pos[1]);

  JointToActuatorPositionInterface to_act_pos;
  to_act_pos.registerHandle(JointToActuatorPositionHandle("trans_1", &trans1, a_cmd_data[0], j_cmd_data[0]));
  to_act_pos.registerHandle(JointToActuatorPositionHandle("trans_2", &trans2, a_cmd_data[1], j_cmd_data[1]));

  // Identity map
  a_curr_pos[0] = 1.0;
  a_curr_pos[1] = 1.0;
  to_jnt_pos.propagate();
  EXPECT_NEAR(1.1, j_curr_pos[0], EPS);
  EXPECT_NEAR(0.9, j_curr_pos[1], EPS);

  j_cmd_pos[0] = j_curr_pos[0];
  j_cmd_pos[1] = j_curr_pos[1];
  to_act_pos.propagate();
  EXPECT_NEAR(a_curr_pos[0], a_cmd_pos[0], EPS);
  EXPECT_NEAR(a_curr_pos[1], a_cmd_pos[1], EPS);
}

TEST_F(InterfaceWhiteBoxTest, VelocityMap)
{
  // Actuator -> joint
  ActuatorData a_curr_data[2];
  a_curr_data[0].velocity = vector<double*>(1, &a_curr_vel[0]);
  a_curr_data[1].velocity = vector<double*>(1, &a_curr_vel[1]);

  JointData j_curr_data[2];
  j_curr_data[0].velocity = vector<double*>(1, &j_curr_vel[0]);
  j_curr_data[1].velocity = vector<double*>(1, &j_curr_vel[1]);

  ActuatorToJointVelocityInterface to_jnt_vel;
  to_jnt_vel.registerHandle(ActuatorToJointVelocityHandle("trans_1", &trans1, a_curr_data[0], j_curr_data[0]));
  to_jnt_vel.registerHandle(ActuatorToJointVelocityHandle("trans_2", &trans2, a_curr_data[1], j_curr_data[1]));

  // Joint -> actuator
  ActuatorData a_cmd_data[2];
  a_cmd_data[0].velocity = vector<double*>(1, &a_cmd_vel[0]);
  a_cmd_data[1].velocity = vector<double*>(1, &a_cmd_vel[1]);

  JointData j_cmd_data[2];
  j_cmd_data[0].velocity = vector<double*>(1, &j_cmd_vel[0]);
  j_cmd_data[1].velocity = vector<double*>(1, &j_cmd_vel[1]);

  JointToActuatorVelocityInterface to_act_vel;
  to_act_vel.registerHandle(JointToActuatorVelocityHandle("trans_1", &trans1, a_cmd_data[0], j_cmd_data[0]));
  to_act_vel.registerHandle(JointToActuatorVelocityHandle("trans_2", &trans2, a_cmd_data[1], j_cmd_data[1]));

  // Identity map
  a_curr_vel[0] = 1.0;
  a_curr_vel[1] = 1.0;
  to_jnt_vel.propagate();
  EXPECT_NEAR( 0.1, j_curr_vel[0], EPS);
  EXPECT_NEAR(-0.1, j_curr_vel[1], EPS);

  j_cmd_vel[0] = j_curr_vel[0];
  j_cmd_vel[1] = j_curr_vel[1];
  to_act_vel.propagate();
  EXPECT_NEAR(a_curr_vel[0], a_cmd_vel[0], EPS);
  EXPECT_NEAR(a_curr_vel[1], a_cmd_vel[1], EPS);
}

TEST_F(InterfaceWhiteBoxTest, EffortMap)
{
  // Actuator -> joint
  ActuatorData a_curr_data[2];
  a_curr_data[0].effort = vector<double*>(1, &a_curr_eff[0]);
  a_curr_data[1].effort = vector<double*>(1, &a_curr_eff[1]);

  JointData j_curr_data[2];
  j_curr_data[0].effort = vector<double*>(1, &j_curr_eff[0]);
  j_curr_data[1].effort = vector<double*>(1, &j_curr_eff[1]);

  ActuatorToJointEffortInterface to_jnt_eff;
  to_jnt_eff.registerHandle(ActuatorToJointEffortHandle("trans_1", &trans1, a_curr_data[0], j_curr_data[0]));
  to_jnt_eff.registerHandle(ActuatorToJointEffortHandle("trans_2", &trans2, a_curr_data[1], j_curr_data[1]));

  // Joint -> actuator
  ActuatorData a_cmd_data[2];
  a_cmd_data[0].effort = vector<double*>(1, &a_cmd_eff[0]);
  a_cmd_data[1].effort = vector<double*>(1, &a_cmd_eff[1]);

  JointData j_cmd_data[2];
  j_cmd_data[0].effort = vector<double*>(1, &j_cmd_eff[0]);
  j_cmd_data[1].effort = vector<double*>(1, &j_cmd_eff[1]);

  JointToActuatorEffortInterface to_act_eff;
  to_act_eff.registerHandle(JointToActuatorEffortHandle("trans_1", &trans1, a_cmd_data[0], j_cmd_data[0]));
  to_act_eff.registerHandle(JointToActuatorEffortHandle("trans_2", &trans2, a_cmd_data[1], j_cmd_data[1]));

  // Identity map
  a_curr_eff[0] = 1.0;
  a_curr_eff[1] = 1.0;
  to_jnt_eff.propagate();
  EXPECT_NEAR( 10.0, j_curr_eff[0], EPS);
  EXPECT_NEAR(-10.0, j_curr_eff[1], EPS);

  j_cmd_eff[0] = j_curr_eff[0];
  j_cmd_eff[1] = j_curr_eff[1];
  to_act_eff.propagate();
  EXPECT_NEAR(a_curr_eff[0], a_cmd_eff[0], EPS);
  EXPECT_NEAR(a_curr_eff[1], a_cmd_eff[1], EPS);
}

TEST_F(InterfaceWhiteBoxTest, StateMap)
{
  // Actuator -> joint
  ActuatorData a_curr_data[2];
  a_curr_data[0].position = vector<double*>(1, &a_curr_pos[0]);
  a_curr_data[0].velocity = vector<double*>(1, &a_curr_vel[0]);
  a_curr_data[0].effort   = vector<double*>(1, &a_curr_eff[0]);

  a_curr_data[1].position = vector<double*>(1, &a_curr_pos[1]);
  a_curr_data[1].velocity = vector<double*>(1, &a_curr_vel[1]);
  a_curr_data[1].effort   = vector<double*>(1, &a_curr_eff[1]);

  JointData j_curr_data[2];
  j_curr_data[0].position = vector<double*>(1, &j_curr_pos[0]);
  j_curr_data[0].velocity = vector<double*>(1, &j_curr_vel[0]);
  j_curr_data[0].effort   = vector<double*>(1, &j_curr_eff[0]);

  j_curr_data[1].position = vector<double*>(1, &j_curr_pos[1]);
  j_curr_data[1].velocity = vector<double*>(1, &j_curr_vel[1]);
  j_curr_data[1].effort   = vector<double*>(1, &j_curr_eff[1]);

  ActuatorToJointStateInterface to_jnt_state;
  to_jnt_state.registerHandle(ActuatorToJointStateHandle("trans_1", &trans1, a_curr_data[0], j_curr_data[0]));
  to_jnt_state.registerHandle(ActuatorToJointStateHandle("trans_2", &trans2, a_curr_data[1], j_curr_data[1]));

  // Joint -> actuator
  ActuatorData a_cmd_data[2];
  a_cmd_data[0].position = vector<double*>(1, &a_cmd_pos[0]);
  a_cmd_data[0].velocity = vector<double*>(1, &a_cmd_vel[0]);
  a_cmd_data[0].effort   = vector<double*>(1, &a_cmd_eff[0]);

  a_cmd_data[1].position = vector<double*>(1, &a_cmd_pos[1]);
  a_cmd_data[1].velocity = vector<double*>(1, &a_cmd_vel[1]);
  a_cmd_data[1].effort   = vector<double*>(1, &a_cmd_eff[1]);

  JointData j_cmd_data[2];
  j_cmd_data[0].position = vector<double*>(1, &j_cmd_pos[0]);
  j_cmd_data[0].velocity = vector<double*>(1, &j_cmd_vel[0]);
  j_cmd_data[0].effort   = vector<double*>(1, &j_cmd_eff[0]);

  j_cmd_data[1].position = vector<double*>(1, &j_cmd_pos[1]);
  j_cmd_data[1].velocity = vector<double*>(1, &j_cmd_vel[1]);
  j_cmd_data[1].effort   = vector<double*>(1, &j_cmd_eff[1]);

  JointToActuatorStateInterface to_act_state;
  to_act_state.registerHandle(JointToActuatorStateHandle("trans_1", &trans1, a_cmd_data[0], j_cmd_data[0]));
  to_act_state.registerHandle(JointToActuatorStateHandle("trans_2", &trans2, a_cmd_data[1], j_cmd_data[1]));

  // Identity map
  a_curr_pos[0] = 1.0;
  a_curr_vel[0] = 1.0;
  a_curr_eff[0] = 1.0;
  a_curr_pos[1] = 1.0;
  a_curr_vel[1] = 1.0;
  a_curr_eff[1] = 1.0;

  to_jnt_state.propagate();
  EXPECT_NEAR(  1.1, j_curr_pos[0], EPS);
  EXPECT_NEAR(  0.1, j_curr_vel[0], EPS);
  EXPECT_NEAR( 10.0, j_curr_eff[0], EPS);
  EXPECT_NEAR(  0.9, j_curr_pos[1], EPS);
  EXPECT_NEAR( -0.1, j_curr_vel[1], EPS);
  EXPECT_NEAR(-10.0, j_curr_eff[1], EPS);

  j_cmd_pos[0] = j_curr_pos[0];
  j_cmd_vel[0] = j_curr_vel[0];
  j_cmd_eff[0] = j_curr_eff[0];
  j_cmd_pos[1] = j_curr_pos[1];
  j_cmd_vel[1] = j_curr_vel[1];
  j_cmd_eff[1] = j_curr_eff[1];

  to_act_state.propagate();
  EXPECT_NEAR(a_curr_pos[0], a_cmd_pos[0], EPS);
  EXPECT_NEAR(a_curr_vel[0], a_cmd_vel[0], EPS);
  EXPECT_NEAR(a_curr_eff[0], a_cmd_eff[0], EPS);
  EXPECT_NEAR(a_curr_pos[1], a_cmd_pos[1], EPS);
  EXPECT_NEAR(a_curr_vel[1], a_cmd_vel[1], EPS);
  EXPECT_NEAR(a_curr_eff[1], a_cmd_eff[1], EPS);
}


class AccessorTest : public TransmissionInterfaceSetup {};

TEST_F(AccessorTest, AccessorValidation)
{
  ActuatorData a_curr_data[2];
  JointData    j_curr_data[2];
  a_curr_data[0].position = vector<double*>(1, &a_curr_pos[0]);
  j_curr_data[0].position = vector<double*>(1, &j_curr_pos[0]);
  a_curr_data[1].position = vector<double*>(1, &a_curr_pos[1]);
  j_curr_data[1].position = vector<double*>(1, &j_curr_pos[1]);

  ActuatorToJointPositionInterface trans_iface;
  trans_iface.registerHandle(ActuatorToJointPositionHandle("trans_1", &trans1, a_curr_data[0], j_curr_data[0]));
  trans_iface.registerHandle(ActuatorToJointPositionHandle("trans_2", &trans2, a_curr_data[1], j_curr_data[1]));

  // Retrieve transmission names
  vector<string> trans_names = trans_iface.getNames();
  ASSERT_EQ(2, trans_names.size());

  EXPECT_EQ("trans_1", trans_names[0]);
  EXPECT_EQ("trans_2", trans_names[1]);

  // Retrieve transmission handles
  ActuatorToJointPositionHandle trans_handle1 = trans_iface.getHandle(trans_names[0]);
  EXPECT_EQ(trans_names[0], trans_handle1.getName());

  ActuatorToJointPositionHandle trans_handle2 = trans_iface.getHandle(trans_names[1]);
  EXPECT_EQ(trans_names[1], trans_handle2.getName());

  EXPECT_THROW(trans_iface.getHandle("unregistered_name"), TransmissionInterfaceException);

  // Print error message
  // Requires manual output inspection, but exception message should contain the interface name (not its base class)
  try {trans_iface.getHandle("unregistered_name");}
  catch(const TransmissionInterfaceException& e) {ROS_ERROR_STREAM(e.what());}
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
