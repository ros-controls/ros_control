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

/*
 * Author: Wim Meeussen
 */


#ifndef CONTROLLER_INTERFACE_CONTROLLER_BASE_H
#define CONTROLLER_INTERFACE_CONTROLLER_BASE_H

#include <ros/node_handle.h>
#include <hardware_interface/robot_hw.h>


namespace controller_interface
{

class ControllerBase
{
public:
  ControllerBase(): state_(CONSTRUCTED){}
  virtual ~ControllerBase(){}

  /// The starting method is called just before the first update from within the realtime thread.
  virtual void starting(const ros::Time& time) {};

  /// The update method is called periodically by the realtime thread when the controller is running
  virtual void update(const ros::Time& time) = 0;

  /// The stopping method is called by the realtime thread just after the last update call
  virtual void stopping(const ros::Time& time) {};

  /// Check if the controller is running
  bool isRunning()
  {
    return (state_ == RUNNING);
  }



  virtual bool initRequest(hardware_interface::RobotHW* hw, ros::NodeHandle &n)=0;

  void updateRequest(const ros::Time& time)
  {
    if (state_ == RUNNING)
      update(time);
  }

  bool startRequest(const ros::Time& time)
  {
    // start succeeds even if the controller was already started
    if (state_ == RUNNING || state_ == INITIALIZED){
      starting(time);
      state_ = RUNNING;
      return true;
    }
    else
      return false;
  }


  bool stopRequest(const ros::Time& time)
  {
    // stop succeeds even if the controller was already stopped
    if (state_ == RUNNING || state_ == INITIALIZED){
      stopping(time);
      state_ = INITIALIZED;
      return true;
    }
    else
      return false;
  }

  enum {CONSTRUCTED, INITIALIZED, RUNNING} state_;



private:
  ControllerBase(const ControllerBase &c);
  ControllerBase& operator =(const ControllerBase &c);

};

}


#endif
