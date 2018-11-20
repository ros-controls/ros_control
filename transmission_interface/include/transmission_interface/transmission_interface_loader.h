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

#ifndef TRANSMISSION_INTERFACE_TRANSMISSION_INTERFACE_LOADER_H
#define TRANSMISSION_INTERFACE_TRANSMISSION_INTERFACE_LOADER_H

// C++ standard
#include <algorithm>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

// Boost
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

// ROS
#include <ros/console.h>

// pluginlib
#include <pluginlib/class_loader.hpp>

// ros_control
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/internal/demangle_symbol.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

#include <transmission_interface/robot_transmissions.h>
#include <transmission_interface/transmission.h>
#include <transmission_interface/transmission_interface.h>
#include <transmission_interface/transmission_info.h>
#include <transmission_interface/transmission_loader.h>
#include <transmission_interface/transmission_parser.h>

namespace transmission_interface
{

namespace internal
{
// NOTE: Adapted to C++03 from http://www.cplusplus.com/reference/algorithm/is_permutation
// TODO: Use Boost's or C++11's implementation, once they become widespread
template<class ForwardIt1, class ForwardIt2>
bool is_permutation(ForwardIt1 first, ForwardIt1 last,
                    ForwardIt2 d_first)
{
  // skip common prefix
  std::pair<ForwardIt1, ForwardIt2> pair = std::mismatch(first, last, d_first);
  first = pair.first; d_first = pair.second;

  // iterate over the rest, counting how many times each element
  // from [first, last) appears in [d_first, d_last)
  if (first != last) {
    ForwardIt2 d_last = d_first;
    std::advance(d_last, std::distance(first, last));
    for (ForwardIt1 i = first; i != last; ++i) {
      if (i != std::find(first, i, *i)) continue; // already counted this *i

      int m = std::count(d_first, d_last, *i);
      if (m==0 || std::count(i, last, *i) != m) {
          return false;
      }
    }
  }
  return true;
}

} // namespace


/**
 * \brief Raw data for a set of joints.
 *
 * For simplicity, every joint has read/write position, velocity and effort variables, but not all of them will
 * necessarily be used.
 */
// TODO: Don't assume interfaces?
struct RawJointData
{
  RawJointData()
    : position(std::numeric_limits<double>::quiet_NaN()),
      velocity(std::numeric_limits<double>::quiet_NaN()),
      effort(std::numeric_limits<double>::quiet_NaN()),
      position_cmd(std::numeric_limits<double>::quiet_NaN()),
      velocity_cmd(std::numeric_limits<double>::quiet_NaN()),
      effort_cmd(std::numeric_limits<double>::quiet_NaN())
  {}

  double position;
  double velocity;
  double effort;
  double position_cmd;
  double velocity_cmd;
  double effort_cmd;
};

typedef std::map<std::string, RawJointData> RawJointDataMap;

/**
 * \brief Joint interfaces of a robot. Only used interfaces need to be populated.
 */
struct JointInterfaces
{
  hardware_interface::JointStateInterface    joint_state_interface;
  hardware_interface::PositionJointInterface position_joint_interface;
  hardware_interface::VelocityJointInterface velocity_joint_interface;
  hardware_interface::EffortJointInterface   effort_joint_interface;
};

struct ForwardTransmissionInterfaces
{
  ActuatorToJointStateInterface    act_to_jnt_state;
  JointToActuatorPositionInterface jnt_to_act_pos_cmd;
  JointToActuatorVelocityInterface jnt_to_act_vel_cmd;
  JointToActuatorEffortInterface   jnt_to_act_eff_cmd;
};

struct InverseTransmissionInterfaces
{
  JointToActuatorStateInterface    jnt_to_act_state;
  ActuatorToJointPositionInterface act_to_jnt_pos_cmd;
  ActuatorToJointVelocityInterface act_to_jnt_vel_cmd;
  ActuatorToJointEffortInterface   act_to_jnt_eff_cmd;
};

struct TransmissionLoaderData
{
  typedef boost::shared_ptr<Transmission> TransmissionPtr; // DEPRECATED and unused!

  TransmissionLoaderData()
    : robot_hw(0),
      robot_transmissions(0)
  {}

  hardware_interface::RobotHW*  robot_hw;            ///< Lifecycle is externally controlled (ie. hardware abstraction)
  RobotTransmissions*           robot_transmissions; ///< Lifecycle is externally controlled (ie. hardware abstraction)
  JointInterfaces               joint_interfaces;
  RawJointDataMap               raw_joint_data_map;
  ForwardTransmissionInterfaces transmission_interfaces;
  InverseTransmissionInterfaces inverse_transmission_interfaces;
  std::vector<TransmissionSharedPtr>  transmission_data;
};

class RequisiteProvider // TODO: There must be a more descriptive name for this class!
{
public:

  virtual ~RequisiteProvider() {}

  /**
   * \brief Update a robot's joint interfaces with joint information contained in a transmission.
   *
   * \param[in] transmission_info Structure containing information of which joints to add. Only new, non-previously
   * registered joints will be added.
   *
   * \param[out] joint_ifaces Joint interfaces where new joints will be added. It may already contain data, which
   * will not be overwritten; only new joints will be added.
   *
   * \param raw_joint_data_map[out] Structure where the raw data of new joints will reside.  It may already contain data,
   * which will not be overwritten; only new data will be added.
   *
   * \return true if successful.
   */
  virtual bool updateJointInterfaces(const TransmissionInfo&      transmission_info,
                                     hardware_interface::RobotHW* robot_hw,
                                     JointInterfaces&             joint_interfaces,
                                     RawJointDataMap&             raw_joint_data_map) = 0;

  bool loadTransmissionMaps(const TransmissionInfo& transmission_info,
                            TransmissionLoaderData& loader_data,
                            TransmissionSharedPtr   transmission);
protected:
  struct TransmissionHandleData
  {
    std::string           name;
    ActuatorData          act_state_data;
    ActuatorData          act_cmd_data;
    JointData             jnt_state_data;
    JointData             jnt_cmd_data;
    TransmissionSharedPtr transmission;
  };

  virtual bool getJointStateData(const TransmissionInfo& transmission_info,
                                 const RawJointDataMap&  raw_joint_data_map,
                                 JointData&              jnt_state_data) = 0;

  virtual bool getJointCommandData(const TransmissionInfo& transmission_info,
                                   const RawJointDataMap&  raw_joint_data_map,
                                   JointData&              jnt_cmd_data) = 0;

  virtual bool getActuatorStateData(const TransmissionInfo&      transmission_info,
                                    hardware_interface::RobotHW* robot_hw,
                                    ActuatorData&                act_state_data) = 0;

  virtual bool getActuatorCommandData(const TransmissionInfo&      transmission_info,
                                      hardware_interface::RobotHW* robot_hw,
                                      ActuatorData&                act_cmd_data) = 0;

  virtual bool registerTransmission(TransmissionLoaderData& loader_data,
                                    TransmissionHandleData& handle_data) = 0;

  template <class Interface>
  static bool hasResource(const std::string& name, const Interface& iface)
  {
    using hardware_interface::internal::demangledTypeName;

    // Do nothing if resource already exists on the interface
    const std::vector<std::string>& existing_resources = iface.getNames();
    if (existing_resources.end() != std::find(existing_resources.begin(), existing_resources.end(), name))
    {
      ROS_DEBUG_STREAM_NAMED("parser", "Resource '" << name << "' already exists on interface '" <<
                             demangledTypeName<Interface>());
      return true;
    }
    else
    {
      ROS_DEBUG_STREAM_NAMED("parser", "Resource '" << name << "' does not exist on interface '" <<
                             demangledTypeName<Interface>());
      return false;
    }
  }

  template <class HardwareInterface, class Handle>
  bool getActuatorHandles(const std::vector<ActuatorInfo>& actuators_info,
                          hardware_interface::RobotHW*     robot_hw,
                          std::vector<Handle>&             handles)
  {
    using hardware_interface::RobotHW;
    using hardware_interface::HardwareInterfaceException;
    using hardware_interface::internal::demangledTypeName;

    HardwareInterface* hw_iface = robot_hw->get<HardwareInterface>();

    // Check required hardware interface
    if (!hw_iface)
    {
      ROS_ERROR_STREAM_NAMED("parser", "Robot does not have the required hardware interface '" <<
                             demangledTypeName<HardwareInterface>() << "'.");
      return false;
    }

    // Get handles to all required resource
    BOOST_FOREACH(const ActuatorInfo& info, actuators_info)
    {
      try
      {
        handles.push_back(hw_iface->getHandle(info.name_));
      }
      catch(const HardwareInterfaceException& ex)
      {
        ROS_ERROR_STREAM_NAMED("parser", "Actuator '" << info.name_ <<
                               "' does not expose the required hardware interface '" <<
                               demangledTypeName<HardwareInterface>() << "'.");
        return false;
      }
    }
    return true;
  }
};

/**
 * \brief Class for loading transmissions from a URDF description into ros_control interfaces.
 *
 * This is the entry point for automatic transmission parsing. An instance of this class is initialized with
 * pointers to hardware and transmission abstraction interfaces, and calling the \c load() methods populates these
 * interfaces.
 *
 * The input information needed for automatically loading transmissions are:
 *  - A transmission specification (provided in the \c load() methods, e.g., a URDF description).
 *  - A \c RobotHW instance (provided in the constructor) already populated with the robot's actuator interfaces.
 *
 * The output after successful transmission loading is:
 * - A \c RobotHW instance (provided in the constructor) to which joint interfaces have been added.
 * - A \c RobotTransmissions instance (provided in the constructor) to which transmission interfaces have been
 *   added.
 *
 * <b>Important note:</b> This class is \e stateful, and stores internally the raw data of the generated joint
 * interfaces, so keep instances alive as long as these interfaces are used.
 *
 * \code
 * class FooRobot : public hardware_interface::RobotHW
 * {
 * public:
 *   void read(ros::Time time, ros::Duration period);
 *   void write(ros::Time time, ros::Duration period);
 *
 *   bool init()
 *   {
 *     // Populate robot_hw with actuator interfaces (e.g., EffortActuatorInterface)
 *     // This is hardware-specific, and not detailed here
 *     // ...
 *
 *     // Initialize transmission loader
 *     try
 *     {
 *       transmission_loader_.reset(new TransmissionInterfaceLoader(this, &robot_transmissions));
 *     }
 *     catch(const std::invalid_argument& ex)
 *     {
 *       ROS_ERROR_STREAM("Failed to create transmission interface loader. " << ex.what());
 *       return false;
 *     }
 *     catch(const pluginlib::LibraryLoadException& ex)
 *     {
 *       ROS_ERROR_STREAM("Failed to create transmission interface loader. " << ex.what());
 *       return false;
 *     }
 *     catch(...)
 *     {
 *       ROS_ERROR_STREAM("Failed to create transmission interface loader. ");
 *       return false;
 *     }
 *
 *     std::string robot_description;
 *     // ...load URDF from parameter server or file
 *
 *     // Perform actual transmission loading
 *     if (!transmission_loader_->load(robot_description)) {return false;}
 *
 *    // We can now query for any of the created interfaces, for example:
 *    // robot_transmissions_.get<ActuatorToJointStateInterface>();
 *    // this->get<JointStateInterface>();
 *
 *    return true;
 *  }
 *
 * private:
 *   RobotTransmissions robot_transmissions_;
 *   boost::scoped_ptr<TransmissionInterfaceLoader> transmission_loader_;
 *
 * };
 * \endcode
 */
class TransmissionInterfaceLoader
{
public:
  /**
   * \param robot_hw Robot hardware abstraction already populated with actuator interfaces.
   * \param robot_transmissions Robot transmissions abstraction.
   */
  TransmissionInterfaceLoader(hardware_interface::RobotHW* robot_hw,
                              RobotTransmissions*          robot_transmissions);

  /**
   * \brief Load all transmissions in a URDF.
   *
   * This method adds new joint and transmission interfaces to the \c RobotHW and \c RobotTransmissions instances
   * passed in the constructor, respectively.
   * \param urdf Robot description containing a transmission specification.
   * \return True if successful.
   */
  bool load(const std::string& urdf);

  /**
   * \brief Load all transmissions contained in a vector of \c TransmissionInfo instances.
   *
   * This method adds new joint and transmission interfaces to the \c RobotHW and \c RobotTransmissions instances
   * passed in the constructor, respectively.
   * \param transmission_info_vec Vector of \c TransmissionInfo instances.
   * \return True if successful.
   */
  bool load(const std::vector<TransmissionInfo>& transmission_info_vec);

  /**
   * \brief Load a single transmission.
   *
   * This method adds new joint and transmission interfaces to the \c RobotHW and \c RobotTransmissions instances
   * passed in the constructor, respectively.
   * \param transmission_info Specification of a single transmission.
   * \return True if successful.
   */
  bool load(const TransmissionInfo& transmission_info);

  TransmissionLoaderData* getData() {return &loader_data_;}

private:
  typedef pluginlib::ClassLoader<TransmissionLoader>      TransmissionClassLoader;
  typedef boost::shared_ptr<TransmissionClassLoader>      TransmissionClassLoaderPtr;
  typedef pluginlib::ClassLoader<RequisiteProvider>       RequisiteProviderClassLoader;
  typedef boost::shared_ptr<RequisiteProviderClassLoader> RequisiteProviderClassLoaderPtr;

  typedef boost::shared_ptr<RequisiteProvider>            RequisiteProviderPtr;

  TransmissionClassLoaderPtr transmission_class_loader_;
  RequisiteProviderClassLoaderPtr req_provider_loader_;

private:
  hardware_interface::RobotHW* robot_hw_ptr_;
  RobotTransmissions*          robot_transmissions_ptr_;

  TransmissionLoaderData loader_data_;
};

} // namespace

#endif // header guard
