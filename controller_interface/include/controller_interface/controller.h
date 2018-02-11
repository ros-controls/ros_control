///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012, hiDOF INC.
// Copyright (C) 2015, PAL Robotics S.L.
// Copyright (C) 2018, Clearpath Robotics.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the names of the copyright holders or the names of their
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

/**
 * \author Wim Meeussen
 * \author Adolfo Rodríguez Tsouroukdissian
 * \author Mike Purvis
 * */

#ifndef CONTROLLER_INTERFACE_CONTROLLER_H
#define CONTROLLER_INTERFACE_CONTROLLER_H

#include <boost/config.hpp>
#include <controller_interface/controller_base.h>
#include <controller_interface/internal/robothw_interfaces.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/hardware_interface.h>
#include <ros/node_handle.h>

namespace controller_interface
{

/**
 * \brief %Controller is able to claim resources from multiple hardware interfaces.
 *
 * This controller implementation allows to claim resources from one or
 * more different hardware interfaces. The types of these hardware interfaces
 * are specified as template parameters.
 *
 * An example multi-interface controller could claim, for instance, resources
 * from a position-controlled arm and velocity-controlled wheels. Another
 * example would be a controller claiming both position and effort interfaces
 * for the same robot resources, but this would require a robot with a custom
 * (non-exclusive) resource handling policy.
 *
 * By default, all specified hardware interfaces are required, and their
 * existence will be enforced by \ref initRequest. It is possible to make hardware
 * interfaces optional by means of the \c allow_optional_interfaces
 * \ref Controller::Controller "constructor" parameter.
 * This allows to write controllers where some interfaces are mandatory, and
 * others, if present, improve controller performance, but whose absence does not
 * prevent the controller from running.
 *
 * The following is an example of a controller claiming resources from velocity-
 * and effort-controlled joints.
 *
 * \code
 * #include <controller_interface/controller.h>
 * #include <hardware_interface/joint_command_interface.h>
 *
 * using namespace hardware_interface;
 * class VelEffController : public
 *       controller_interface::Controller<VelocityJointInterface,
 *                                        EffortJointInterface>
 * {
 * public:
 *   VelEffController() {}
 *
 *   bool init(VelocityJointInterface* v, EffortJointInterface* e, ros::NodeHandle &n)
 *     // v and e are guaranteed to be valid. Fetch resources from them,
 *     // perform rest of initialization
 *
 *     return true;
 *   }
 *   void starting(const ros::Time& time);
 *   void update(const ros::Time& time, const ros::Duration& period);
 *   void stopping(const ros::Time& time);
 * };
 * \endcode
 *
 * The following fragment is a modified version of the above example, where
 * controller interfaces are not required. It is left to the controller
 * implementer to verify interface validity. Only the initialization code is
 * shown.
 *
 * \code
 * class VelEffController : public
 *       controller_interface::Controller<VelocityJointInterface,
                                          EffortJointInterface>
 * {
 * public:
 *   // Note true flag passed to parent class, allowing requested hardware
 *   // interfaces to be optional
 *   VelEffController()
 *    : controller_interface::Controller<VelocityJointInterface,
                                         EffortJointInterface> (true)
 *   {}
 *
 *   bool init(VelocityJointInterface* v, EffortJointInterface* e, ros::NodeHandle &n)
 *   {
 *     // v is a required interface
 *     if (!v)
 *     {
 *       return false;
 *     }
 *
 *     // e is an optional interface. If present, additional features are enabled.
 *     // Controller can still function if interface or some of its resources are
 *     // absent
 *     if (e)
 *     {
 *       // ...
 *     }
 *
 *     // Fetch resources from interfaces, perform rest of initialization
 *     // ...
 *
 *     return true;
 *   }
 *   ...
 * };
 * \endcode
 *
 * \tparam T... Hardware interface types.
 * This parameter is \e required.
 */
template <class... Interfaces>
class Controller: public virtual ControllerBase
{
public:
  /**
   * \param allow_optional_interfaces If set to true, \ref initRequest will
   * not fail if one or more of the requested interfaces is not present.
   * If set to false (the default), all requested interfaces are required.
   */
  Controller(bool allow_optional_interfaces = false)
    : allow_optional_interfaces_(allow_optional_interfaces)
  {state_ = CONSTRUCTED;}

  virtual ~Controller() {}

  /** \name Non Real-Time Safe Functions
   *\{*/

  /**
   * \brief Custom controller initialization logic.
   *
   * In this method resources from different interfaces are claimed, and other
   * non real-time initialization is performed, such as setup of ROS interfaces
   * and resource pre-allocation.
   *
   * \param interfaces Pointers to the hardware interfaces used by this
   * controller, as specified in the template parameter pack.
   * If \ref Controller::Controller was called with \c allow_optional_interfaces
   * set to \c false (the default), all pointers will be valid and non-NULL.
   * If \c allow_optional_interfaces was set to \c true, some or all of the
   * interface pointers may be NULL, and must be individually checked.
   * Please refer to the code examples in the \ref Controller class description.
   *
   * \param controller_nh A NodeHandle in the namespace from which the controller
   * should read its configuration, and where it should set up its ROS
   * interface.
   *
   * \returns True if initialization was successful and the controller
   * is ready to be started.
   */
  virtual bool init(Interfaces*... /*interfaces*/,
                    ros::NodeHandle& /*controller_nh*/)
  {
    return true;
  }

  /**
   * \brief Custom controller initialization logic.
   *
   * In this method resources from different interfaces are claimed, and other
   * non real-time initialization is performed, such as setup of ROS interfaces
   * and resource pre-allocation.
   *
   * \param interfaces Pointers to the hardware interfaces used by this
   * controller, as specified in the template parameter pack.
   * If \ref Controller::Controller was called with \c allow_optional_interfaces
   * set to \c false (the default), all pointers will be valid and non-NULL.
   * If \c allow_optional_interfaces was set to \c true, some or all of the
   * interface pointers may be NULL, and must be individually checked.
   * Please refer to the code examples in the \ref Controller class description.
   *
   * \param root_nh A NodeHandle in the root of the controller manager namespace.
   * This is where the ROS interfaces are setup (publishers, subscribers, services).
   *
   * \param controller_nh A NodeHandle in the namespace of the controller.
   * This is where the controller-specific configuration resides.
   *
   * \returns True if initialization was successful and the controller
   * is ready to be started.
   */
  virtual bool init(Interfaces*... /*interfaces*/,
                    ros::NodeHandle& /*root_nh*/,
                    ros::NodeHandle& /*controller_nh*/)
  {
    return true;
  }


protected:
  /**
   * \brief Initialize the controller from a RobotHW pointer.
   *
   * This calls the user-supplied \ref init method with the hardware interfaces
   * from the \c robot_hw pointer, some or all of which may be NULL, depending
   * on the value of \c allow_optional_interfaces passed to the constructor.
   *
   * \param robot_hw The robot hardware abstraction.
   *
   * \param root_nh A NodeHandle in the root of the controller manager namespace.
   * This is where the ROS interfaces are setup (publishers, subscribers, services).
   *
   * \param controller_nh A NodeHandle in the namespace of the controller.
   * This is where the controller-specific configuration resides.
   *
   * \param[out] claimed_resources The resources claimed by this controller.
   * They can belong to multiple hardware interfaces.
   *
   * \returns True if initialization was successful and the controller
   * is ready to be started.
   */
  virtual bool initRequest(hardware_interface::RobotHW* robot_hw,
                           ros::NodeHandle&             root_nh,
                           ros::NodeHandle&             controller_nh,
                           ClaimedResources&            claimed_resources)
  {
    // Check if construction finished cleanly.
    if (state_ != CONSTRUCTED){
      ROS_ERROR("Cannot initialize this controller because it failed to be constructed.");
      return false;
    }

    // Check for required hardware interfaces.
    if (!allow_optional_interfaces_ && !internal::hasInterfaces<Interfaces...>(robot_hw))
    {
      // Error message has already been sent by the checking function.
      return false;
    }

    // Custom controller initialization.
    if (!init(robot_hw->get<Interfaces>()..., controller_nh) ||
        !init(robot_hw->get<Interfaces>()..., root_nh, controller_nh))
    {
      ROS_ERROR("Failed to initialize the controller.");
      return false;
    }

    // Populate claimed resources.
    claimed_resources.clear();
    internal::populateClaimedResources<Interfaces...>(robot_hw, claimed_resources);
    internal::clearClaims<Interfaces...>(robot_hw);
    // NOTE: Above, claims are cleared since we only want to know what they are and report them back
    // as an output parameter. Actual resource claiming by the controller is done when the controller
    // is start()ed

    // Initialization has succeeded.
    state_ = INITIALIZED;
    return true;
  }

  /**
   * This is provided for compatibility with Controller from when it only took a single hardware
   * interface. It can't be used when Controller has more than on interface type.
   */
  ROS_DEPRECATED std::string getHardwareInterfaceType() const
  {
    return hardware_interface::internal::demangledTypeName<Interfaces...>();
  }

  /*\}*/

private:
  /** Flag to indicate if hardware interfaces are considered optional (i.e. non-required). */
  bool allow_optional_interfaces_;

  /**
   * Construction by copying prohibited. Controllers are not copyable.
   */
  Controller(const Controller& c);

  /**
   * Construction by assignment prohibited. Controllers are not copyable.
   */
  Controller& operator =(const Controller& c);

#ifndef BOOST_NO_VARIADIC_TEMPLATES
  // Everything other than this line actually compiles just fine under gcc without -std=c++11,
  // barring warnings about varidic templates being unsupported. This guard can be removed in Melodic
  // when the compiler is c++14 by default.
  static_assert(sizeof...(Interfaces) >= 1, "Controller must have at least one hardware interface.");
#endif  // BOOST_NO_VARIADIC_TEMPLATES
};

} // namespace

#endif
