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

#ifndef CONTROLLER_INTERFACE_MULTI_CONTROLLER_H
#define CONTROLLER_INTERFACE_MULTI_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/internal/demangle_symbol.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/hardware_interface.h>
#include <ros/ros.h>


namespace controller_interface
{

/** \brief %Controller with a specific hardware interface
 *
 * \tparam T1 The hardware interface type used by this controller. This enforces
 * semantic compatibility between the controller and the hardware it's meant to
 * control.
 * \tparam T2 The hardware interface type used by this controller. This enforces
 * semantic compatibility between the controller and the hardware it's meant to
 * control.
 */
template <class T1, class T2>
class Controller2: public ControllerBase
{
public:
  Controller2()  {state_ = CONSTRUCTED;}
  virtual ~Controller2<T1, T2>(){}

  /** \brief The init function is called to initialize the controller from a
   * non-realtime thread with a pointer to the hardware interface, itself,
   * instead of a pointer to a RobotHW.
   *
   * \param hw1 The specific hardware interface used by this controller.
   *
   * \param hw2 The specific hardware interface used by this controller.
   *
   * \param controller_nh A NodeHandle in the namespace from which the controller
   * should read its configuration, and where it should set up its ROS
   * interface.
   *
   * \returns True if initialization was successful and the controller
   * is ready to be started.
   */
  virtual bool init(T1* hw1, T2* hw2, ros::NodeHandle &controller_nh) {return true;};

  /** \brief The init function is called to initialize the controller from a
   * non-realtime thread with a pointer to the hardware interface, itself,
   * instead of a pointer to a RobotHW.
   *
   * \param hw1 The specific hardware interface used by this controller.
   *
   * \param hw2 The specific hardware interface used by this controller.
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
  virtual bool init(T1* hw1, T2* hw2, ros::NodeHandle& root_nh, ros::NodeHandle &controller_nh) {return true;};


protected:
  /** \brief Initialize the controller from a RobotHW pointer
   *
   * This calls \ref init with the hardware interface for this controller if it
   * can extract the correct interface from \c robot_hw.
   *
   */
  virtual bool initRequest(hardware_interface::RobotHW* robot_hw,
                           ros::NodeHandle& root_nh, ros::NodeHandle &controller_nh,
                           std::set<std::string> &claimed_resources)
  {
    // check if construction finished cleanly
    if (state_ != CONSTRUCTED){
      ROS_ERROR("Cannot initialize this controller because it failed to be constructed");
      return false;
    }

    // get a pointer to the hardware interface
    T1* hw1 = robot_hw->get<T1>();
    T2* hw2 = robot_hw->get<T2>();
    if (!hw1 || !hw2)
    {
      ROS_ERROR("This controller requires a hardware interface of type '%s'."
                " Make sure this is registered in the hardware_interface::RobotHW class.",
                getHardwareInterfaceType().c_str());
      return false;
    }

    // return which resources are claimed by this controller
    hw1->clearClaims();
    hw2->clearClaims();
    if (!init(hw1, hw2, controller_nh) || !init(hw1, hw2, root_nh, controller_nh))
    {
      ROS_ERROR("Failed to initialize the controller");
      return false;
    }
    claimed_resources = hw1->getClaims();
    std::set<std::string> claimed_resources2 = hw2->getClaims();
    claimed_resources.insert(claimed_resources2.begin(), claimed_resources2.end());
    hw1->clearClaims();
    hw2->clearClaims();

    // success
    state_ = INITIALIZED;
    return true;
  }

  virtual std::string getHardwareInterfaceType() const
  {
    return "<" + hardware_interface::internal::demangledTypeName<T1>() + "," +
           hardware_interface::internal::demangledTypeName<T2>() + ">";
  }

private:
  Controller2<T1, T2>(const Controller2<T1, T2> &c);
  Controller2<T1, T2>& operator =(const Controller2<T1, T2> &c);

};

/** \brief %Controller with a specific hardware interface
 *
 * \tparam T1 The hardware interface type used by this controller. This enforces
 * semantic compatibility between the controller and the hardware it's meant to
 * control.
 * \tparam T2 The hardware interface type used by this controller. This enforces
 * semantic compatibility between the controller and the hardware it's meant to
 * control.
 * \tparam T3 The hardware interface type used by this controller. This enforces
 * semantic compatibility between the controller and the hardware it's meant to
 * control.
 */
template <class T1, class T2, class T3>
class Controller3: public ControllerBase
{
public:
  Controller3()  {state_ = CONSTRUCTED;}
  virtual ~Controller3<T1, T2, T3>(){}

  /** \brief The init function is called to initialize the controller from a
   * non-realtime thread with a pointer to the hardware interface, itself,
   * instead of a pointer to a RobotHW.
   *
   * \param hw1 The specific hardware interface used by this controller.
   *
   * \param hw2 The specific hardware interface used by this controller.
   *
   * \param hw3 The specific hardware interface used by this controller.
   *
   * \param controller_nh A NodeHandle in the namespace from which the controller
   * should read its configuration, and where it should set up its ROS
   * interface.
   *
   * \returns True if initialization was successful and the controller
   * is ready to be started.
   */
  virtual bool init(T1* hw1, T2* hw2, T3* hw3, ros::NodeHandle &controller_nh) {return true;};

  /** \brief The init function is called to initialize the controller from a
   * non-realtime thread with a pointer to the hardware interface, itself,
   * instead of a pointer to a RobotHW.
   *
   * \param hw1 The specific hardware interface used by this controller.
   *
   * \param hw2 The specific hardware interface used by this controller.
   *
   * \param hw3 The specific hardware interface used by this controller.
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
  virtual bool init(T1* hw1, T2* hw2, T3* hw3, ros::NodeHandle& root_nh, ros::NodeHandle &controller_nh) {return true;};


protected:
  /** \brief Initialize the controller from a RobotHW pointer
   *
   * This calls \ref init with the hardware interface for this controller if it
   * can extract the correct interface from \c robot_hw.
   *
   */
  virtual bool initRequest(hardware_interface::RobotHW* robot_hw,
                           ros::NodeHandle& root_nh, ros::NodeHandle &controller_nh,
                           std::set<std::string> &claimed_resources)
  {
    // check if construction finished cleanly
    if (state_ != CONSTRUCTED){
      ROS_ERROR("Cannot initialize this controller because it failed to be constructed");
      return false;
    }

    // get a pointer to the hardware interface
    T1* hw1 = robot_hw->get<T1>();
    T2* hw2 = robot_hw->get<T2>();
    T3* hw3 = robot_hw->get<T3>();
    if (!hw1 || !hw2 || !hw3)
    {
      ROS_ERROR("This controller requires a hardware interface of type '%s'."
                " Make sure this is registered in the hardware_interface::RobotHW class.",
                getHardwareInterfaceType().c_str());
      return false;
    }

    // return which resources are claimed by this controller
    hw1->clearClaims();
    hw2->clearClaims();
    hw3->clearClaims();
    if (!init(hw1, hw2, hw3, controller_nh) || !init(hw1, hw2, hw3, root_nh, controller_nh))
    {
      ROS_ERROR("Failed to initialize the controller");
      return false;
    }
    claimed_resources = hw1->getClaims();
    std::set<std::string> claimed_resources2 = hw2->getClaims();
    std::set<std::string> claimed_resources3 = hw3->getClaims();
    claimed_resources.insert(claimed_resources2.begin(), claimed_resources2.end());
    claimed_resources.insert(claimed_resources3.begin(), claimed_resources3.end());
    hw1->clearClaims();
    hw2->clearClaims();
    hw3->clearClaims();

    // success
    state_ = INITIALIZED;
    return true;
  }

  virtual std::string getHardwareInterfaceType() const
  {
    return "<" + hardware_interface::internal::demangledTypeName<T1>() + "," +
           hardware_interface::internal::demangledTypeName<T2>() + "," +
           hardware_interface::internal::demangledTypeName<T3>() + ">";
  }

private:
  Controller3<T1, T2, T3>(const Controller3<T1, T2, T3> &c);
  Controller3<T1, T2, T3>& operator =(const Controller3<T1, T2, T3> &c);

};

/** \brief %Controller with a specific hardware interface
 *
 * \tparam T1 The hardware interface type used by this controller. This enforces
 * semantic compatibility between the controller and the hardware it's meant to
 * control.
 * \tparam T2 The hardware interface type used by this controller. This enforces
 * semantic compatibility between the controller and the hardware it's meant to
 * control.
 * \tparam T3 The hardware interface type used by this controller. This enforces
 * semantic compatibility between the controller and the hardware it's meant to
 * control.
 * \tparam T4 The hardware interface type used by this controller. This enforces
 * semantic compatibility between the controller and the hardware it's meant to
 * control.
 */
template <class T1, class T2, class T3, class T4>
class Controller4: public ControllerBase
{
public:
  Controller4()  {state_ = CONSTRUCTED;}
  virtual ~Controller4<T1, T2, T3, T4>(){}

  /** \brief The init function is called to initialize the controller from a
   * non-realtime thread with a pointer to the hardware interface, itself,
   * instead of a pointer to a RobotHW.
   *
   * \param hw1 The specific hardware interface used by this controller.
   *
   * \param hw2 The specific hardware interface used by this controller.
   *
   * \param hw3 The specific hardware interface used by this controller.
   *
   * \param hw4 The specific hardware interface used by this controller.
   *
   * \param controller_nh A NodeHandle in the namespace from which the controller
   * should read its configuration, and where it should set up its ROS
   * interface.
   *
   * \returns True if initialization was successful and the controller
   * is ready to be started.
   */
  virtual bool init(T1* hw1, T2* hw2, T3* hw3, T4* hw4, ros::NodeHandle &controller_nh) {return true;};

  /** \brief The init function is called to initialize the controller from a
   * non-realtime thread with a pointer to the hardware interface, itself,
   * instead of a pointer to a RobotHW.
   *
   * \param hw1 The specific hardware interface used by this controller.
   *
   * \param hw2 The specific hardware interface used by this controller.
   *
   * \param hw3 The specific hardware interface used by this controller.
   *
   * \param hw4 The specific hardware interface used by this controller.
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
  virtual bool init(T1* hw1, T2* hw2, T3* hw3, T4* hw4, ros::NodeHandle& root_nh, ros::NodeHandle &controller_nh) {return true;};


protected:
  /** \brief Initialize the controller from a RobotHW pointer
   *
   * This calls \ref init with the hardware interface for this controller if it
   * can extract the correct interface from \c robot_hw.
   *
   */
  virtual bool initRequest(hardware_interface::RobotHW* robot_hw,
                           ros::NodeHandle& root_nh, ros::NodeHandle &controller_nh,
                           std::set<std::string> &claimed_resources)
  {
    // check if construction finished cleanly
    if (state_ != CONSTRUCTED){
      ROS_ERROR("Cannot initialize this controller because it failed to be constructed");
      return false;
    }

    // get a pointer to the hardware interface
    T1* hw1 = robot_hw->get<T1>();
    T2* hw2 = robot_hw->get<T2>();
    T3* hw3 = robot_hw->get<T3>();
    T4* hw4 = robot_hw->get<T4>();
    if (!hw1 || !hw2 || !hw3 || !hw4)
    {
      ROS_ERROR("This controller requires a hardware interface of type '%s'."
                " Make sure this is registered in the hardware_interface::RobotHW class.",
                getHardwareInterfaceType().c_str());
      return false;
    }

    // return which resources are claimed by this controller
    hw1->clearClaims();
    hw2->clearClaims();
    hw3->clearClaims();
    hw4->clearClaims();
    if (!init(hw1, hw2, hw3, hw4, controller_nh) || !init(hw1, hw2, hw3, hw4, root_nh, controller_nh))
    {
      ROS_ERROR("Failed to initialize the controller");
      return false;
    }
    claimed_resources = hw1->getClaims();
    std::set<std::string> claimed_resources2 = hw2->getClaims();
    std::set<std::string> claimed_resources3 = hw3->getClaims();
    std::set<std::string> claimed_resources4 = hw4->getClaims();
    claimed_resources.insert(claimed_resources2.begin(), claimed_resources2.end());
    claimed_resources.insert(claimed_resources3.begin(), claimed_resources3.end());
    claimed_resources.insert(claimed_resources4.begin(), claimed_resources4.end());
    hw1->clearClaims();
    hw2->clearClaims();
    hw3->clearClaims();
    hw4->clearClaims();

    // success
    state_ = INITIALIZED;
    return true;
  }

  virtual std::string getHardwareInterfaceType() const
  {
    return "<" + hardware_interface::internal::demangledTypeName<T1>() + "," +
           hardware_interface::internal::demangledTypeName<T2>() + "," +
           hardware_interface::internal::demangledTypeName<T3>() + "," +
           hardware_interface::internal::demangledTypeName<T4>() + ">";
  }

private:
  Controller4<T1, T2, T3, T4>(const Controller4<T1, T2, T3, T4> &c);
  Controller4<T1, T2, T3, T4>& operator =(const Controller4<T1, T2, T3, T4> &c);

};

}

#endif
