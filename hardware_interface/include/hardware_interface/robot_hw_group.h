///////////////////////////////////////////////////////////////////////////////
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
 * Author: Kelsey Hawkins
 */

#ifndef HARDWARE_INTERFACE_ROBOT_HW_GROUP_H
#define HARDWARE_INTERFACE_ROBOT_HW_GROUP_H

#include <vector>
#include <hardware_interface/robot_hw.h>
#include <ros/console.h>

namespace hardware_interface
{

/** \brief Robot Hardware Collection Manager
 *
 * This class maintains a group of RobotHW, along with its
 * own registered interfaces, and exposes an interface
 * identical to RobotHW.
 *
 */
class RobotHWGroup : public RobotHW
{
public:
  RobotHWGroup()
  {

  }

  void registerHardware(RobotHW* hw)
  {
    hardware_.push_back(hw);
  }

  /**
   * \brief Get generic pointers to interfaces with type_name.
   *
   * Returns a list of all interfaces registered internally or
   * registered in one of the registered hardwares.
   * \param type_name The name of the interface types stored.
   * \return List of generic pointers to the interfaces found.
   */
  virtual std::vector<void*> findInterfaceData(std::string type_name)
  {
    std::vector<void*> iface_data;

    // look for interfaces registered here
    std::vector<void*> iface_data_cur;
    iface_data_cur = InterfaceManager::findInterfaceData(type_name);
    if(iface_data_cur.size() > 0)
      iface_data.insert(iface_data.end(), iface_data_cur.begin(), iface_data_cur.end());

    // look for interfaces registered in the registered hardware
    for(HardwareVector::iterator it = hardware_.begin(); it != hardware_.end(); ++it) {
      iface_data_cur = (*it)->findInterfaceData(type_name);
      if(iface_data_cur.size() > 0) 
        iface_data.insert(iface_data.end(), iface_data_cur.begin(), iface_data_cur.end());
    }
    return iface_data;
  }

protected:
  typedef std::vector<RobotHW*> HardwareVector;
  HardwareVector hardware_;

};

}

#endif

