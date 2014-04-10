/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2014, Igor Kalevatykh, Bauman Moscow State Technical University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the Bauman Moscow State Technical University,
 *      nor the names of its contributors may be used to endorse or promote
 *      products derived from this software without specific prior written
 *      permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/// \author Igor Kalevatykh <kalevatykhia@gmail.com>

#ifndef COMPOSITE_HARDWARE_INTERFACE__DEVICE_HW_H
#define COMPOSITE_HARDWARE_INTERFACE__DEVICE_HW_H

#include <composite_hardware_interface/shared_interface_manager.h>
#include <composite_hardware_interface/shared_resource_manager.h>
#include <composite_hardware_interface/shared_urdf_model.h>
#include <boost/shared_ptr.hpp>

namespace composite_hardware_interface
{

/** \brief Device Hardware Interface
 *
 * It is meant to be used as a base class for abstracting custom robot
 * device hardware.
 *
 */
class DeviceHW
{
public:
  /** \brief Configure device.
   *
   * \param ifaces     Interfaces shared thru all robot devices
   * \param resources  Named resources shared thru all robot devices
   * \param urdf_model Urdf robot model shared thru all robot devices
   * \param dev_name   Device name from config
   * \param dev_node   Node with namespace base_ns / \ref dev_name
   *
   * \returns True on success
   */
  virtual bool configure(SharedInterfaceManager &ifaces,
                         SharedResourceManager &resources,
                         SharedUrdfModel &urdf_model,
                         const std::string &dev_name,
                         const ros::NodeHandle &dev_node) = 0;

  /** \brief Start device hardware.
   *
   *  \returns True on success
   */
  virtual bool start() { return true; }

  /** \name Real-Time Safe Functions
   *\{*/
  /** \brief Read data from the device.
   *
   * \param time The current time
   * \param period The change in time since the last call to \ref read
   *
   * \returns True on success
   */
  virtual bool read(const ros::Time time, const ros::Duration period) { return true; }

  /** \brief Write data to the device.
   *
   * \param time The current time
   * \param period The change in time since the last call to \ref write
   *
   * \returns True on success
   */
  virtual bool write(const ros::Time time, const ros::Duration period) { return true; }
  /*\}*/

  /** \brief Stop device hardware. */
  virtual void stop() { }

  /** \brief Virtual destructor. */
  virtual ~DeviceHW() { }
};

typedef boost::shared_ptr<DeviceHW> DeviceHwPtr;

}

#endif

