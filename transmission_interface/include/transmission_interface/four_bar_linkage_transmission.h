
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

#pragma once


#include <cassert>
#include <string>
#include <vector>

#include <transmission_interface/transmission.h>
#include <transmission_interface/transmission_interface_exception.h>

namespace transmission_interface
{

  /**
 * \brief Implementation of a four-bar-linkage transmission.
 *
 * This transmission relates <b>two actuators</b> and <b>two joints</b> through a mechanism in which the state of the
 * first joint only depends on the first actuator, while the second joint depends on both actuators, as
 * illustrated below.
 * Although the class name makes specific reference to the four-bar-linkage, there are other mechanical layouts
 * that yield the same behavior, such as the remote actuation example also depicted below.
 * \image html four_bar_linkage_transmission.png
 *
 * <CENTER>
 * <table>
 * <tr><th></th><th><CENTER>Effort</CENTER></th><th><CENTER>Velocity</CENTER></th><th><CENTER>Position</CENTER></th></tr>
 * <tr><td>
 * <b> Actuator to joint </b>
 * </td>
 * <td>
 * \f{eqnarray*}{
 * \tau_{j_1} & = & n_{j_1} n_{a_1} \tau_{a_1} \\
 * \tau_{j_2} & = & n_{j_2} (n_{a_2} \tau_{a_2} - n_{j_1} n_{a_1} \tau_{a_1})
 * \f}
 * </td>
 * <td>
 * \f{eqnarray*}{
 * \dot{x}_{j_1} & = & \frac{ \dot{x}_{a_1} }{ n_{j_1} n_{a_1} } \\
 * \dot{x}_{j_2} & = & \frac{ \dot{x}_{a_2} / n_{a_2} - \dot{x}_{a_1} / (n_{j_1} n_{a_1}) }{ n_{j_2} }
 * \f}
 * </td>
 * <td>
 * \f{eqnarray*}{
 * x_{j_1} & = & \frac{ x_{a_1} }{ n_{j_1} n_{a_1} } + x_{off_1} \\
 * x_{j_2} & = & \frac{ x_{a_2} / n_{a_2} - x_{a_1} / (n_{j_1} n_{a_1}) }{ n_{j_2} } + x_{off_2}
 * \f}
 * </td>
 * </tr>
 * <tr><td>
 * <b> Joint to actuator </b>
 * </td>
 * <td>
 * \f{eqnarray*}{
 * \tau_{a_1} & = & \tau_{j_1} / (n_{j_1} n_{a_1}) \\
 * \tau_{a_2} & = & \frac{ \tau_{j_1} + \tau_{j_2} / n_{j_2} }{ n_{a_2} }
 * \f}
 * </td>
 * <td>
 * \f{eqnarray*}{
 * \dot{x}_{a_1} & = & n_{j_1} n_{a_1} \dot{x}_{j_1} \\
 * \dot{x}_{a_2} & = & n_{a_2} (\dot{x}_{j_1} + n_{j_2} \dot{x}_{j_2})
 * \f}
 * </td>
 * <td>
 * \f{eqnarray*}{
 * x_{a_1} & = & n_{j_1} n_{a_1} (x_{j_1} - x_{off_1}) \\
 * x_{a_2} & = & n_{a_2} \left[(x_{j_1} - x_{off_1}) + n_{j_2} (x_{j_2}  - x_{off_2})\right]
 * \f}
 * </td></tr></table>
 * </CENTER>
 *
 * where:
 * - \f$ x \f$, \f$ \dot{x} \f$ and \f$ \tau \f$ are position, velocity and effort variables, respectively.
 * - Subindices \f$ _a \f$ and \f$ _j \f$ are used to represent actuator-space and joint-space variables, respectively.
 * - \f$ x_{off}\f$ represents the offset between motor and joint zeros, expressed in joint position coordinates.
 *   (cf. SimpleTransmission class documentation for a more detailed description of this variable).
 * - \f$ n \f$ represents a transmission ratio (reducers/amplifiers are depicted as timing belts in the figure).
 *   A transmission ratio can take any real value \e except zero. In particular:
 *     - If its absolute value is greater than one, it's a velocity reducer / effort amplifier, while if its absolute
 *       value lies in \f$ (0, 1) \f$ it's a velocity amplifier / effort reducer.
 *     - Negative values represent a direction flip, ie. input and output move in opposite directions.
 *     - <b>Important:</b> Use transmission ratio signs to match this class' convention of positive actuator/joint
 *       directions with a given mechanical design, as they will in general not match.
 *
 * \ingroup transmission_types
 */
class FourBarLinkageTransmission : public Transmission
{
public:
  /**
   * \param actuator_reduction Reduction ratio of actuators.
   * \param joint_reduction    Reduction ratio of joints.
   * \param joint_offset       Joint position offset used in the position mappings.
   * \pre Nonzero actuator reduction values.
   */
  FourBarLinkageTransmission(const std::vector<double>& actuator_reduction,
                             const std::vector<double>& joint_reduction,
                             const std::vector<double>& joint_offset = std::vector<double>(2, 0.0));

  /**
   * \brief Transform \e effort variables from actuator to joint space.
   * \param[in]  act_data Actuator-space variables.
   * \param[out] jnt_data Joint-space variables.
   * \pre Actuator and joint effort vectors must have size 2 and point to valid data.
   *  To call this method it is not required that all other data vectors contain valid data, and can even remain empty.
   */
  void actuatorToJointEffort(const ActuatorData& act_data,
                                   JointData&    jnt_data) override;

  /**
   * \brief Transform \e velocity variables from actuator to joint space.
   * \param[in]  act_data Actuator-space variables.
   * \param[out] jnt_data Joint-space variables.
   * \pre Actuator and joint velocity vectors must have size 2 and point to valid data.
   *  To call this method it is not required that all other data vectors contain valid data, and can even remain empty.
   */
  void actuatorToJointVelocity(const ActuatorData& act_data,
                                     JointData&    jnt_data) override;

  /**
   * \brief Transform \e position variables from actuator to joint space.
   * \param[in]  act_data Actuator-space variables.
   * \param[out] jnt_data Joint-space variables.
   * \pre Actuator and joint position vectors must have size 2 and point to valid data.
   *  To call this method it is not required that all other data vectors contain valid data, and can even remain empty.
   */
  void actuatorToJointPosition(const ActuatorData& act_data,
                                     JointData&    jnt_data) override;

  /**
   * \brief Transform \e absolute encoder values from actuator to joint space.
   * \param[in]  act_data Actuator-space variables.
   * \param[out] jnt_data Joint-space variables.
   * \pre Actuator, joint position and absolute encoder position vectors must have the same size.
   *  To call this method it is not required that all other data vectors contain valid data, and can even remain empty.
   */
  void actuatorToJointAbsolutePosition(const ActuatorData& act_data,
                                             JointData&    jnt_data) override;

  /**
   * \brief Transform \e torque sensor values from actuator to joint space.
   * \param[in]  act_data Actuator-space variables.
   * \param[out] jnt_data Joint-space variables.
   * \pre Actuator, joint position and torque sensor vectors must have the same size.
   */
  void actuatorToJointTorqueSensor(const ActuatorData& act_data,
                                         JointData&    jnt_data) override;

  /**
   * \brief Transform \e effort variables from joint to actuator space.
   * \param[in]  jnt_data Joint-space variables.
   * \param[out] act_data Actuator-space variables.
   * \pre Actuator and joint effort vectors must have size 2 and point to valid data.
   *  To call this method it is not required that all other data vectors contain valid data, and can even remain empty.
   */
  void jointToActuatorEffort(const JointData&    jnt_data,
                                   ActuatorData& act_data) override;

  /**
   * \brief Transform \e velocity variables from joint to actuator space.
   * \param[in]  jnt_data Joint-space variables.
   * \param[out] act_data Actuator-space variables.
   * \pre Actuator and joint velocity vectors must have size 2 and point to valid data.
   *  To call this method it is not required that all other data vectors contain valid data, and can even remain empty.
   */
  void jointToActuatorVelocity(const JointData&    jnt_data,
                                     ActuatorData& act_data) override;

  /**
   * \brief Transform \e position variables from joint to actuator space.
   * \param[in]  jnt_data Joint-space variables.
   * \param[out] act_data Actuator-space variables.
   * \pre Actuator and joint position vectors must have size 2 and point to valid data.
   *  To call this method it is not required that all other data vectors contain valid data, and can even remain empty.
   */
  void jointToActuatorPosition(const JointData&    jnt_data,
                                     ActuatorData& act_data) override;

  std::size_t numActuators() const override {return 2;}
  std::size_t numJoints()    const override {return 2;}
  bool hasActuatorToJointAbsolutePosition()  const override {return true;}
  bool hasActuatorToJointTorqueSensor()      const override {return true;}

  const std::vector<double>& getActuatorReduction() const {return act_reduction_;}
  const std::vector<double>& getJointReduction()    const {return jnt_reduction_;}
  const std::vector<double>& getJointOffset()       const {return jnt_offset_;}

protected:
  std::vector<double>  act_reduction_;
  std::vector<double>  jnt_reduction_;
  std::vector<double>  jnt_offset_;
};

inline FourBarLinkageTransmission::FourBarLinkageTransmission(const std::vector<double>& actuator_reduction,
                                                              const std::vector<double>& joint_reduction,
                                                              const std::vector<double>& joint_offset)
  : act_reduction_(actuator_reduction),
    jnt_reduction_(joint_reduction),
    jnt_offset_(joint_offset)
{
  if (numActuators() != act_reduction_.size() ||
      numJoints()    != jnt_reduction_.size() ||
      numJoints()    != jnt_offset_.size())
  {
    throw TransmissionInterfaceException("Reduction and offset vectors of a four-bar linkage transmission must have size 2.");
  }
  if (0.0 == act_reduction_[0] ||
      0.0 == act_reduction_[1] ||
      0.0 == jnt_reduction_[0] ||
      0.0 == jnt_reduction_[1])
  {
    throw TransmissionInterfaceException("Transmission reduction ratios cannot be zero.");
  }
}

inline void FourBarLinkageTransmission::actuatorToJointEffort(const ActuatorData& act_data,
                                                                    JointData&    jnt_data)
{
  assert(numActuators() == act_data.effort.size() && numJoints() == jnt_data.effort.size());
  assert(act_data.effort[0] && act_data.effort[1] && jnt_data.effort[0] && jnt_data.effort[1]);

  std::vector<double>& ar = act_reduction_;
  std::vector<double>& jr = jnt_reduction_;

  *jnt_data.effort[0] = jr[0] * (*act_data.effort[0] * ar[0]);
  *jnt_data.effort[1] = jr[1] * (*act_data.effort[1] * ar[1] - *act_data.effort[0] * ar[0] * jr[0]);
}

inline void FourBarLinkageTransmission::actuatorToJointVelocity(const ActuatorData& act_data,
                                                                      JointData&    jnt_data)
{
  assert(numActuators() == act_data.velocity.size() && numJoints() == jnt_data.velocity.size());
  assert(act_data.velocity[0] && act_data.velocity[1] && jnt_data.velocity[0] && jnt_data.velocity[1]);

  std::vector<double>& ar = act_reduction_;
  std::vector<double>& jr = jnt_reduction_;

  *jnt_data.velocity[0] = *act_data.velocity[0] / (jr[0] * ar[0]);
  *jnt_data.velocity[1] = (*act_data.velocity[1] / ar[1] - *act_data.velocity[0] / (jr[0] * ar[0])) / jr[1];
}

inline void FourBarLinkageTransmission::actuatorToJointPosition(const ActuatorData& act_data,
                                                                      JointData&    jnt_data)
{
  assert(numActuators() == act_data.position.size() && numJoints() == jnt_data.position.size());
  assert(act_data.position[0] && act_data.position[1] && jnt_data.position[0] && jnt_data.position[1]);

  std::vector<double>& ar = act_reduction_;
  std::vector<double>& jr = jnt_reduction_;

  *jnt_data.position[0] = *act_data.position[0] /(jr[0] * ar[0]) + jnt_offset_[0];
  *jnt_data.position[1] = (*act_data.position[1] / ar[1] - *act_data.position[0] / (jr[0] * ar[0])) / jr[1]
                          + jnt_offset_[1];
}

void FourBarLinkageTransmission::actuatorToJointAbsolutePosition(const ActuatorData& act_data,
                                                                       JointData&    jnt_data)
{

  assert(numActuators() == act_data.absolute_position.size() && numJoints() == jnt_data.absolute_position.size());
  assert(act_data.absolute_position[0] && act_data.absolute_position[1] && jnt_data.absolute_position[0] && jnt_data.absolute_position[1]);

  const std::vector<double>& ar = act_reduction_;
  const std::vector<double>& jr = jnt_reduction_;

  *jnt_data.absolute_position[0] = *act_data.absolute_position[0] /(jr[0] * ar[0]) + jnt_offset_[0];
  *jnt_data.absolute_position[1] = (*act_data.absolute_position[1] / ar[1] - *act_data.absolute_position[0] / (jr[0] * ar[0])) /
                                    jr[1] + jnt_offset_[1];
}

void FourBarLinkageTransmission::actuatorToJointTorqueSensor(const ActuatorData& act_data,
                                                                   JointData&    jnt_data)
{

  assert(numActuators() == act_data.torque_sensor.size() && numJoints() == jnt_data.torque_sensor.size());
  assert(act_data.torque_sensor[0] && act_data.torque_sensor[1] && jnt_data.torque_sensor[0] && jnt_data.torque_sensor[1]);

  const std::vector<double>& ar = act_reduction_;
  const std::vector<double>& jr = jnt_reduction_;

  *jnt_data.torque_sensor[0] = jr[0] * (*act_data.torque_sensor[0] * ar[0]);
  *jnt_data.torque_sensor[1] = jr[1] * (*act_data.torque_sensor[1] * ar[1] - *act_data.torque_sensor[0] * ar[0] * jr[0]);
}


inline void FourBarLinkageTransmission::jointToActuatorEffort(const JointData&    jnt_data,
                                                                    ActuatorData& act_data)
{
  assert(numActuators() == act_data.effort.size() && numJoints() == jnt_data.effort.size());
  assert(act_data.effort[0] && act_data.effort[1] && jnt_data.effort[0] && jnt_data.effort[1]);

  std::vector<double>& ar = act_reduction_;
  std::vector<double>& jr = jnt_reduction_;

  *act_data.effort[0] = *jnt_data.effort[0] / (ar[0] * jr[0]);
  *act_data.effort[1] = (*jnt_data.effort[0] + *jnt_data.effort[1] / jr[1]) / ar[1];
}

inline void FourBarLinkageTransmission::jointToActuatorVelocity(const JointData&    jnt_data,
                                                                      ActuatorData& act_data)
{
  assert(numActuators() == act_data.velocity.size() && numJoints() == jnt_data.velocity.size());
  assert(act_data.velocity[0] && act_data.velocity[1] && jnt_data.velocity[0] && jnt_data.velocity[1]);

  std::vector<double>& ar = act_reduction_;
  std::vector<double>& jr = jnt_reduction_;

  *act_data.velocity[0] = *jnt_data.velocity[0] * jr[0] * ar[0];
  *act_data.velocity[1] = (*jnt_data.velocity[0] + *jnt_data.velocity[1] * jr[1]) * ar[1];
}

inline void FourBarLinkageTransmission::jointToActuatorPosition(const JointData&    jnt_data,
                                                                      ActuatorData& act_data)
{
  assert(numActuators() == act_data.position.size() && numJoints() == jnt_data.position.size());
  assert(act_data.position[0] && act_data.position[1] && jnt_data.position[0] && jnt_data.position[1]);

  std::vector<double>& ar = act_reduction_;
  std::vector<double>& jr = jnt_reduction_;

  double jnt_pos_off[2] = {*jnt_data.position[0] - jnt_offset_[0], *jnt_data.position[1] - jnt_offset_[1]};

  *act_data.position[0] = jnt_pos_off[0] * jr[0] * ar[0];
  *act_data.position[1] = (jnt_pos_off[0] + jnt_pos_off[1] * jr[1]) * ar[1];
}

} // transmission_interface
