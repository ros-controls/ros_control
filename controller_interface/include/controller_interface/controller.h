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
//   * Neither the name of Stanford University nor the names of its
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


#include <controller_interface/controller_base.h>

namespace controller_interface
{


Template <class T>
class Controller<T>
{
public:
  Controller<T>(): state_(CONSTRUCTED){}
  virtual ~Controller<T>(){}

  /**
   * @brief The init function is called to initialize the controller from a non-realtime thread
   *
   * @param robot The hardware interface to the robot
   *
   * @param n A NodeHandle in the namespace from which the controller
   * should read its configuration, and where it should set up its ROS
   * interface.
   *
   * @return True if initialization was successful and the controller
   * is ready to be started.
   */
  virtual bool init(T* hw, ros::NodeHandle &n) = 0;



protected:
  virtual bool initRequest(hardware_interface::HardwareInterface* hw, ros::NodeHandle &n)
  {
    if (state_ != CONSTRUCTED)
      return false;
    else
    {
      T* hw_t = dynamic_cast<T>(hw);

      // initialize
      if (!hw_t || !init_untyped(hw, n))
        return false;

      // success
      state_ = INITIALIZED;
      return true;
    }
  }


private:
  Controller<T>(const Controller<T> &c);
  Controller<T>& operator =(const Controller<T> &c);

};

}
