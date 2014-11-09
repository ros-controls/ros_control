///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012, hiDOF INC.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of hiDOF Inc nor the names of its
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

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <controller_manager_tests/my_robot_hw.h>

using namespace controller_manager_tests;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "MultiDummyApp");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  MyRobotHW hw_vel_base("vel_");
  MyRobotHW hw_eff_base("eff_");
  DerivedMyRobotHW hw_vel_derived("vel_");
  DerivedMyRobotHW hw_eff_derived("eff_");
  hardware_interface::RobotHW hw_combined;
  hw_combined.registerInterfaceManager(&hw_vel_base);
  hw_combined.registerInterfaceManager(&hw_eff_base);
  hw_combined.registerInterfaceManager(&hw_vel_derived);
  hw_combined.registerInterfaceManager(&hw_eff_derived);

  ros::NodeHandle nh;
  controller_manager::ControllerManager cm(&hw_combined, nh);

  ros::Duration period(1.0);
  while (ros::ok())
  {
    ROS_INFO("loop");
    hw_vel_base.read();
    hw_eff_base.read();
    hw_vel_derived.read();
    hw_eff_derived.read();
    cm.update(ros::Time::now(), period);
    hw_vel_base.write();
    hw_eff_base.write();
    hw_vel_derived.write();
    hw_eff_derived.write();
    period.sleep();
  }
}

