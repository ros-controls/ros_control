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


// ros_control
#include <transmission_interface/transmission_info.h>
#include <transmission_interface/transmission_interface_loader.h>
#include <transmission_interface/joint_state_interface_provider.h>

namespace transmission_interface
{

class VelocityJointInterfaceProvider : public JointStateInterfaceProvider
{
public:
  bool updateJointInterfaces(const TransmissionInfo&      transmission_info,
                             hardware_interface::RobotHW* robot_hw,
                             JointInterfaces&             joint_interfaces,
                             RawJointDataMap&             raw_joint_data_map) override;

protected:

  bool getJointCommandData(const TransmissionInfo& transmission_info,
                           const RawJointDataMap&  raw_joint_data_map,
                           JointData&              jnt_cmd_data) override;

  bool getActuatorCommandData(const TransmissionInfo&      transmission_info,
                              hardware_interface::RobotHW* robot_hw,
                              ActuatorData&                act_cmd_data) override;

  bool registerTransmission(TransmissionLoaderData& loader_data,
                            TransmissionHandleData& handle_data) override;
};

} // namespace
