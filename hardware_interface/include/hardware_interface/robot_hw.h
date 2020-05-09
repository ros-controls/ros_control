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

#pragma once


#include <list>
#include <map>
#include <memory>
#include <typeinfo>
#include <hardware_interface/internal/demangle_symbol.h>
#include <hardware_interface/internal/interface_manager.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/controller_info.h>
#include <ros/console.h>
#include <ros/node_handle.h>

namespace hardware_interface
{

/** \brief Robot Hardware Interface and Resource Manager
 *
 * This class provides a standardized interface to a set of robot hardware
 * interfaces to the controller manager. It performs resource conflict checking
 * for a given set of controllers and maintains a map of hardware interfaces.
 * It is meant to be used as a base class for abstracting custom robot
 * hardware.
 *
 * The hardware interface map (\ref interfaces_) is a 1-to-1 map between
 * the names of interface types derived from \ref HardwareInterface and
 * instances of those interface types.
 *
 */
class RobotHW : public InterfaceManager
{
public:
  virtual ~RobotHW() = default;

  /** \brief The init function is called to initialize the RobotHW from a
   * non-realtime thread.
   *
   * \param root_nh A NodeHandle in the root of the caller namespace.
   *
   * \param robot_hw_nh A NodeHandle in the namespace from which the RobotHW
   * should read its configuration.
   *
   * \returns True if initialization was successful
   */
  virtual bool init(ros::NodeHandle& /*root_nh*/, ros::NodeHandle &/*robot_hw_nh*/) {return true;}

  /** \name Resource Management
   *\{*/

  /** Check (in non-realtime) if the given set of controllers is allowed
   * to run simultaneously.
   *
   * This default implementation simply checks if any two controllers use the
   * same resource.
   */
  virtual bool checkForConflict(const std::list<ControllerInfo>& info) const
  {
    // Map from resource name to all controllers claiming it
    std::map<std::string, std::list<ControllerInfo>> resource_map;

    // Populate a map of all controllers claiming individual resources.
    // We do this by iterating over every claimed resource of every hardware interface used by every controller
    for (const auto& controller_info : info)
    {
      for (const auto& claimed_resource : controller_info.claimed_resources)
      {
        for (const auto& iface_resource : claimed_resource.resources)
        {
          resource_map[iface_resource].push_back(controller_info);
        }
      }
    }

    // Enforce resource exclusivity policy: No resource can be claimed by more than one controller
    bool in_conflict = false;
    for (const auto& resource_name_and_claiming_controllers : resource_map)
    {
      if (resource_name_and_claiming_controllers.second.size() > 1)
      {
        std::string controller_list;
        for (const auto& controller : resource_name_and_claiming_controllers.second)
          controller_list += controller.name + ", ";
        ROS_WARN("Resource conflict on [%s].  Controllers = [%s]", resource_name_and_claiming_controllers.first.c_str(), controller_list.c_str());
        in_conflict = true;
      }
    }

    return in_conflict;
  }
  /**\}*/

  /** \name Hardware Interface Switching
   *\{*/

  /**
   * Check (in non-realtime) if given controllers could be started and stopped from the current state of the RobotHW
   * with regard to necessary hardware interface switches and prepare the switching. Start and stop list are disjoint.
   * This handles the check and preparation, the actual switch is commited in doSwitch()
   */
  virtual bool prepareSwitch(const std::list<ControllerInfo>& /*start_list*/,
                             const std::list<ControllerInfo>& /*stop_list*/) { return true; }

  /**
   * Perform (in realtime) all necessary hardware interface switches in order to start and stop the given controllers.
   * Start and stop list are disjoint. The feasability was checked in prepareSwitch() beforehand.
   */
  virtual void doSwitch(const std::list<ControllerInfo>& /*start_list*/,
                        const std::list<ControllerInfo>& /*stop_list*/) {}

  enum SwitchState
  {
    DONE,
    ONGOING,
    ERROR
  };

  /** \brief Return (in realtime) the state of the last doSwitch(). */
  virtual SwitchState switchResult() const
  {
    return DONE;
  }

  /** \brief Return (in realtime) the state of the last doSwitch() for a given controller. */
  virtual SwitchState switchResult(const ControllerInfo& /*controller*/) const
  {
    return DONE;
  }
  /**\}*/

  /** \name Control Loop
   *\{*/

  /** \brief Read data from the robot hardware.
   *
   * The read method is part of the control loop cycle (\ref read, update, \ref write) 
   * and is used to populate the robot state from the robot's hardware resources
   * (joints, sensors, actuators). This method should be called before 
   * controller_manager::ControllerManager::update() and \ref write.
   * 
   * \note The name \ref read refers to reading state from the hardware.
   * This complements \ref write, which refers to writing commands to the hardware.
   *
   * Querying WallTime inside \ref read is not realtime safe. The parameters
   * \ref time and \ref period make it possible to inject time from a realtime source.
   *
   * \param time The current time
   * \param period The time passed since the last call to \ref read
   */
  virtual void read(const ros::Time& /*time*/, const ros::Duration& /*period*/) {}

  /** \brief Write commands to the robot hardware.
   * 
   * The write method is part of the control loop cycle (\ref read, update, \ref write) 
   * and is used to send out commands to the robot's hardware 
   * resources (joints, actuators). This method should be called after 
   * \ref read and controller_manager::ControllerManager::update.
   * 
   * \note The name \ref write refers to writing commands to the hardware.
   * This complements \ref read, which refers to reading state from the hardware.
   *
   * Querying WallTime inside \ref write is not realtime safe. The parameters
   * \ref time and \ref period make it possible to inject time from a realtime source.
   *
   * \param time The current time
   * \param period The time passed since the last call to \ref write
   */
  virtual void write(const ros::Time& /*time*/, const ros::Duration& /*period*/) {}

  /**\}*/
};

typedef std::shared_ptr<RobotHW> RobotHWSharedPtr;

}
