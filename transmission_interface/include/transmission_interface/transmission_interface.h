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

#ifndef TRANSMISSION_INTERFACE_TRANSMISSION_INTERFACE_H
#define TRANSMISSION_INTERFACE_TRANSMISSION_INTERFACE_H

#include <map>
#include <string>
#include <vector>

#include <hardware_interface/internal/resource_manager.h>
#include <transmission_interface/transmission.h>
#include <transmission_interface/transmission_interface_exception.h>

namespace transmission_interface
{

/**
 * \brief Handle for propagating a single map (position, velocity, or effort) on a single transmission
 * (eg. actuator to joint effort for a simple reducer).
 */
class TransmissionHandle
{
public:
  /** \return Transmission name. */
  std::string getName() const {return name_;}

protected:
  std::string   name_;
  Transmission* transmission_;
  ActuatorData  actuator_data_;
  JointData     joint_data_;

  /**
   * \param name %Transmission name.
   * \param transmission Pointer to transmission instance.
   * \param actuator_data Actuator-space variables.
   * \param joint_data Joint-space variables.
   * \note The lifecycle of the pointed-to instances passed as parameters is not handled by this class.
   * \pre Valid transmission pointer. Actuator and joint variable vectors required by this handle must contain valid
   * data and their size should be consistent with the number of transmission actuators and joints.
   * Data vectors not used by this handle can remain empty.
   */
  TransmissionHandle(const std::string&  name,
                     Transmission*       transmission,
                     const ActuatorData& actuator_data,
                     const JointData&    joint_data)
    : name_(name),
      transmission_(transmission),
      actuator_data_(actuator_data),
      joint_data_(joint_data)
  {
    // Precondition: Valid transmission
    if (!transmission_)
    {
      throw TransmissionInterfaceException("Unspecified transmission.");
    }

    // Catch trivial error: All data vectors are empty (handle can't do anything without data)
    if (actuator_data.position.empty() && actuator_data.velocity.empty() && actuator_data.effort.empty() &&
        joint_data.position.empty() && joint_data.velocity.empty() && joint_data.effort.empty())
    {
       throw TransmissionInterfaceException("All data vectors are empty. Transmission instance can't do anything!.");
    }

    // Precondition: All non-empty data vectors must have sizes consistent with the transmission
    if (!actuator_data.position.empty() && actuator_data.position.size() != transmission_->numActuators())
    {
      throw TransmissionInterfaceException("Actuator position data size does not match transmission.");
    }
    if (!actuator_data.velocity.empty() && actuator_data.velocity.size() != transmission_->numActuators())
    {
      throw TransmissionInterfaceException("Actuator velocity data size does not match transmission.");
    }
    if (!actuator_data.effort.empty() && actuator_data.effort.size() != transmission_->numActuators())
    {
      throw TransmissionInterfaceException("Actuator effort data size does not match transmission.");
    }

    if (!joint_data.position.empty() && joint_data.position.size() != transmission_->numJoints())
    {
      throw TransmissionInterfaceException("Joint position data size does not match transmission.");
    }
    if (!joint_data.velocity.empty() && joint_data.velocity.size() != transmission_->numJoints())
    {
      throw TransmissionInterfaceException("Joint velocity data size does not match transmission.");
    }
    if (!joint_data.effort.empty() && joint_data.effort.size() != transmission_->numJoints())
    {
      throw TransmissionInterfaceException("Joint effort data size does not match transmission.");
    }

    // Precondition: Valid pointers to raw data
    if (!hasValidPointers(actuator_data.position))
    {
      throw TransmissionInterfaceException("Actuator position data contains null pointers.");
    }
    if (!hasValidPointers(actuator_data.velocity))
    {
      throw TransmissionInterfaceException("Actuator velocity data contains null pointers.");
    }
    if (!hasValidPointers(actuator_data.effort))
    {
      throw TransmissionInterfaceException("Actuator effort data contains null pointers.");
    }

    if (!hasValidPointers(joint_data.position))
    {
      throw TransmissionInterfaceException("Joint position data contains null pointers.");
    }
    if (!hasValidPointers(joint_data.velocity))
    {
      throw TransmissionInterfaceException("Joint velocity data contains null pointers.");
    }
    if (!hasValidPointers(joint_data.effort))
    {
      throw TransmissionInterfaceException("Joint effort data contains null pointers.");
    }
  }

private:
  static bool hasValidPointers(const std::vector<double*>& data)
  {
    for (std::vector<double*>::const_iterator it = data.begin(); it != data.end(); ++it)
    {
      if (!(*it)) {return false;}
    }
    return true;
  }
};

/**
 *\brief Handle for propagating actuator state (position, velocity and effort) to joint state for a given transmission.
 */
class ActuatorToJointStateHandle : public TransmissionHandle
{
public:
  /** \sa TransmissionHandle::TransmissionHandle */
  ActuatorToJointStateHandle(const std::string&  name,
                             Transmission*       transmission,
                             const ActuatorData& actuator_data,
                             const JointData&    joint_data)
    : TransmissionHandle(name, transmission, actuator_data, joint_data) {}

  /** \name Real-Time Safe Functions
   *\{*/
  /** \brief Propagate actuator state to joint state for the stored transmission. */
  void propagate()
  {
    transmission_->actuatorToJointPosition(actuator_data_, joint_data_);
    transmission_->actuatorToJointVelocity(actuator_data_, joint_data_);
    transmission_->actuatorToJointEffort(  actuator_data_, joint_data_);
  }
  /*\}*/
};


/** \brief Handle for propagating actuator positions to joint positions for a given transmission. */
class ActuatorToJointPositionHandle : public TransmissionHandle
{
public:
  /** \sa TransmissionHandle::TransmissionHandle */
  ActuatorToJointPositionHandle(const std::string&  name,
                                Transmission*       transmission,
                                const ActuatorData& actuator_data,
                                const JointData&    joint_data)
    : TransmissionHandle(name, transmission, actuator_data, joint_data) {}

  /** \name Real-Time Safe Functions
   *\{*/
  /** \brief Propagate actuator positions to joint positions for the stored transmission. */
  void propagate() {transmission_->actuatorToJointPosition(actuator_data_, joint_data_);}
  /*\}*/
};


/** \brief Handle for propagating actuator velocities to joint velocities for a given transmission. */
class ActuatorToJointVelocityHandle : public TransmissionHandle
{
public:
  /** \sa TransmissionHandle::TransmissionHandle */
  ActuatorToJointVelocityHandle(const std::string&  name,
                                Transmission*       transmission,
                                const ActuatorData& actuator_data,
                                const JointData&    joint_data)
    : TransmissionHandle(name, transmission, actuator_data, joint_data) {}

  /** \name Real-Time Safe Functions
   *\{*/
  /** \brief Propagate actuator velocities to joint velocities for the stored transmission. */
  void propagate() {transmission_->actuatorToJointVelocity(actuator_data_, joint_data_);}
  /*\}*/
};


/** \brief Handle for propagating actuator efforts to joint efforts for a given transmission. */
class ActuatorToJointEffortHandle : public TransmissionHandle
{
public:
  /** \sa TransmissionHandle::TransmissionHandle */
  ActuatorToJointEffortHandle(const std::string&  name,
                              Transmission*       transmission,
                              const ActuatorData& actuator_data,
                              const JointData&    joint_data)
    : TransmissionHandle(name, transmission, actuator_data, joint_data) {}

  /** \name Real-Time Safe Functions
   *\{*/
  /** \brief Propagate actuator efforts to joint efforts for the stored transmission. */
  void propagate() {transmission_->actuatorToJointEffort(actuator_data_, joint_data_);}
  /*\}*/
};


/**
 *\brief Handle for propagating joint state (position, velocity and effort) to actuator state for a given transmission.
 */
class JointToActuatorStateHandle : public TransmissionHandle
{
public:
  /** \sa TransmissionHandle::TransmissionHandle */
  JointToActuatorStateHandle(const std::string&  name,
                             Transmission*       transmission,
                             const ActuatorData& actuator_data,
                             const JointData&    joint_data)
    : TransmissionHandle(name, transmission, actuator_data, joint_data) {}

  /** \name Real-Time Safe Functions
   *\{*/
  /** \brief Propagate joint state to actuator state for the stored transmission. */
  void propagate()
  {
    transmission_->jointToActuatorPosition(joint_data_, actuator_data_);
    transmission_->jointToActuatorVelocity(joint_data_, actuator_data_);
    transmission_->jointToActuatorEffort(  joint_data_, actuator_data_);
  }
  /*\}*/
};


/** \brief Handle for propagating joint positions to actuator positions for a given transmission. */
class JointToActuatorPositionHandle : public TransmissionHandle
{
public:
  /** \sa TransmissionHandle::TransmissionHandle */
  JointToActuatorPositionHandle(const std::string&  name,
                                Transmission*       transmission,
                                const ActuatorData& actuator_data,
                                const JointData&    joint_data)
    : TransmissionHandle(name, transmission, actuator_data, joint_data) {}

  /** \name Real-Time Safe Functions
   *\{*/
  /** \brief Propagate joint positions to actuator positions for the stored transmission. */
  void propagate() {transmission_->jointToActuatorPosition(joint_data_, actuator_data_);}
  /*\}*/
};


/** \brief Handle for propagating joint velocities to actuator velocities for a given transmission. */
class JointToActuatorVelocityHandle : public TransmissionHandle
{
public:
  /** \sa TransmissionHandle::TransmissionHandle */
  JointToActuatorVelocityHandle(const std::string&  name,
                                Transmission*       transmission,
                                const ActuatorData& actuator_data,
                                const JointData&    joint_data)
    : TransmissionHandle(name, transmission, actuator_data, joint_data) {}

  /** \name Real-Time Safe Functions
   *\{*/
  /** \brief Propagate joint velocities to actuator velocities for the stored transmission. */
  void propagate() {transmission_->jointToActuatorVelocity(joint_data_, actuator_data_);}
  /*\}*/
};


/** \brief Handle for propagating joint efforts to actuator efforts for a given transmission. */
class JointToActuatorEffortHandle : public TransmissionHandle
{
public:
  /** \sa TransmissionHandle::TransmissionHandle */
  JointToActuatorEffortHandle(const std::string&  name,
                              Transmission*       transmission,
                              const ActuatorData& actuator_data,
                              const JointData&    joint_data)
    : TransmissionHandle(name, transmission, actuator_data, joint_data) {}

  /** \name Real-Time Safe Functions
   *\{*/
  /** \brief Propagate joint efforts to actuator efforts for the stored transmission. */
  void propagate() {transmission_->jointToActuatorEffort(joint_data_, actuator_data_);}
  /*\}*/
};

/**
 * \brief Interface for propagating a given map on a set of transmissions.
 *
 * This map can consist of a single variable (eg. actuator to joint efforts, as implemented by
 * \ref ActuatorToJointEffortHandle) or multiple variables (eg. joint to actuator position, velocity and effort, as
 * implemented by \ref JointToActuatorStateHandle).
 *
 * The set of transmissions handled by this interface can be heterogeneous, (eg. an arm with a four-bar-linkage in the
 * shoulder, a differential in the wrist, and simple reducers elsewhere).
 *
 * \tparam HandleType %Transmission handle type. Must implement the following methods:
 *  \code
 *   void propagate();
 *   std::string getName() const;
 *  \endcode
 */

template <class HandleType>
class TransmissionInterface : public hardware_interface::ResourceManager<HandleType>
{
public:

  HandleType getHandle(const std::string& name)
  {
    // Rethrow exception with a meaningful type
    try
    {
      return this->hardware_interface::ResourceManager<HandleType>::getHandle(name);
    }
    catch(const std::logic_error& e)
    {
      throw TransmissionInterfaceException(e.what());
    }
  }

  /** \name Real-Time Safe Functions
   *\{*/
  /** \brief Propagate the transmission maps of all managed handles. */
  void propagate()
  {
    typedef typename hardware_interface::ResourceManager<HandleType>::ResourceMap::iterator ItratorType;
    for (ItratorType it = this->resource_map_.begin(); it != this->resource_map_.end(); ++it)
    {
      it->second.propagate();
    }
  }
  /*\}*/
};

// Convenience typedefs

/** Interface for propagating actuator state (position, velocity and effort) to joint state on a set of transmissions. */
class ActuatorToJointStateInterface : public TransmissionInterface<ActuatorToJointStateHandle> {};

/** Interface for propagating actuator positions to joint positions on a set of transmissions. */
class ActuatorToJointPositionInterface : public TransmissionInterface<ActuatorToJointPositionHandle> {};

/** Interface for propagating actuator velocities to joint velocities on a set of transmissions. */
class ActuatorToJointVelocityInterface : public TransmissionInterface<ActuatorToJointVelocityHandle> {};

/** Interface for propagating actuator efforts to joint efforts on a set of transmissions. */
class ActuatorToJointEffortInterface : public TransmissionInterface<ActuatorToJointEffortHandle> {};

/** Interface for propagating joint state (position, velocity and effort) to actuator state on a set of transmissions. */
class JointToActuatorStateInterface : public TransmissionInterface<JointToActuatorStateHandle> {};

/** Interface for propagating joint positions to actuator positions on a set of transmissions. */
class JointToActuatorPositionInterface : public TransmissionInterface<JointToActuatorPositionHandle> {};

/** Interface for propagating joint velocities to actuator velocities on a set of transmissions. */
class JointToActuatorVelocityInterface : public TransmissionInterface<JointToActuatorVelocityHandle> {};

/** Interface for propagating joint efforts to actuator efforts on a set of transmissions. */
class JointToActuatorEffortInterface : public TransmissionInterface<JointToActuatorEffortHandle> {};

} // transmission_interface

#endif // TRANSMISSION_INTERFACE_TRANSMISSION_INTERFACE_H
