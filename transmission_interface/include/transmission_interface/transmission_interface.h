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

/// \author Adolfo Rodriguez Tsouroukdissian

#ifndef TRANSMISSION_INTERFACE_TRANSMISSION_INTERFACE_H
#define TRANSMISSION_INTERFACE_TRANSMISSION_INTERFACE_H

#include <map>
#include <string>
#include <vector>

#include <transmission_interface/transmission.h>
#include <transmission_interface/transmission_exception.h>

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

  /** \brief Propagate a single map of the stored transmission. */
  virtual void propagate() = 0; // TODO: Remove and let derived classes provide the method without inheritance

protected:
  std::string name_;
  Transmission* transmission_;
  std::vector<double*> actuator_data_;
  std::vector<double*> joint_data_;

  /**
   * \param name %Transmission name.
   * \param transmission Pointer to transmission instance.
   * \param actuator_data Vector of pointers to actuator data.
   * \param joint_data Vector of pointers to joint data.
   * \note The lifecycle of the pointed-to instances passed as parameters is not handled by this class.
   * \pre Valid transmission and actuator/joint data pointers, matching transmission and actuator/joint data sizes.
   */
  TransmissionHandle(const std::string&          name,
                     Transmission*               transmission,
                     const std::vector<double*>& actuator_data,
                     const std::vector<double*>& joint_data)
    : name_(name),
      transmission_(transmission),
      actuator_data_(actuator_data),
      joint_data_(joint_data)
  {
    // Precondition: Valid transmission
    if (!transmission_)
    {
      throw TransmissionException("Unspecified transmission.");
    }

    // Precondition: Matching sizes
    if (actuator_data.size() != transmission_->numActuators())
    {
      throw TransmissionException("Actuator data size does not match transmission.");
    }
    if (joint_data.size() != transmission_->numJoints())
    {
      throw TransmissionException("Joint data size does not match transmission.");
    }

    // Precondition: Valid pointers
    if (!hasValidPointers(actuator_data))
    {
      throw TransmissionException("Actuator data contains null pointers.");
    }
    if (!hasValidPointers(joint_data))
    {
      throw TransmissionException("Joint data contains null pointers.");
    }
  }

  virtual ~TransmissionHandle() {}

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


/** \brief Handle for propagating actuator positions to joint positions for a given transmission. */
class ActuatorToJointPositionHandle : public TransmissionHandle
{
public:
  /** \sa TransmissionHandle::TransmissionHandle */
  ActuatorToJointPositionHandle(const std::string&          name,
                                Transmission*               transmission,
                                const std::vector<double*>& actuator_data,
                                const std::vector<double*>& joint_data)
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
  ActuatorToJointVelocityHandle(const std::string&          name,
                                Transmission*               transmission,
                                const std::vector<double*>& actuator_data,
                                const std::vector<double*>& joint_data)
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
  ActuatorToJointEffortHandle(const std::string&          name,
                              Transmission*               transmission,
                              const std::vector<double*>& actuator_data,
                              const std::vector<double*>& joint_data)
    : TransmissionHandle(name, transmission, actuator_data, joint_data) {}

  /** \name Real-Time Safe Functions
   *\{*/
  /** \brief Propagate actuator efforts to joint efforts for the stored transmission. */
  void propagate() {transmission_->actuatorToJointEffort(actuator_data_, joint_data_);}
  /*\}*/
};


/** \brief Handle for propagating joint positions to actuator positions for a given transmission. */
class JointToActuatorPositionHandle : public TransmissionHandle
{
public:
  /** \sa TransmissionHandle::TransmissionHandle */
  JointToActuatorPositionHandle(const std::string&          name,
                                Transmission*               transmission,
                                const std::vector<double*>& actuator_data,
                                const std::vector<double*>& joint_data)
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
  JointToActuatorVelocityHandle(const std::string&          name,
                                Transmission*               transmission,
                                const std::vector<double*>& actuator_data,
                                const std::vector<double*>& joint_data)
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
  JointToActuatorEffortHandle(const std::string&          name,
                                Transmission*               transmission,
                                const std::vector<double*>& actuator_data,
                                const std::vector<double*>& joint_data)
    : TransmissionHandle(name, transmission, actuator_data, joint_data) {}

  /** \name Real-Time Safe Functions
   *\{*/
  /** \brief Propagate joint efforts to actuator efforts for the stored transmission. */
  void propagate() {transmission_->jointToActuatorEffort(joint_data_, actuator_data_);}
  /*\}*/
};

/**
 * \brief Interface for propagating a single map (position, velocity, effort) on a set of transmissions
 *  (eg. actuator to joint efforts for a complete robot containing a heterogeneous set of transmissions).
 *
 * \tparam HandleType %Transmission handle type. Must implement the following methods:
 *  \code
 *   void propagate();
 *   std::string getName() const;
 *  \endcode
 */
template <class HandleType>
class TransmissionInterface
{
public:
  /** \name Non Real-Time Safe Functions
   *\{*/
  /** \brief Register a new transmission to this interface. */
  void registerTransmission(const std::string&          name,
                            Transmission*               transmission,
                            const std::vector<double*>& actuator_data,
                            const std::vector<double*>& joint_data)
  {
    HandleType handle(name, transmission, actuator_data, joint_data);
    typename HandleMap::iterator it = handle_map_.find(name);
    if (it == handle_map_.end())
    {
      handle_map_.insert(std::make_pair(name, handle));
    }
    else
    {
      it->second = handle;
    }
  }

  /** \return Vector of transmission names registered to this interface. */
  std::vector<std::string> getTransmissionNames() const
  {
    std::vector<std::string> out;
    out.reserve(handle_map_.size());
    for (typename HandleMap::const_iterator it = handle_map_.begin(); it != handle_map_.end(); ++it)
    {
      out.push_back(it->first);
    }
    return out;
  }

  /**
   * \param name The name of the transmission.
   * \return Transmission handle.
   */
  HandleType getTransmissionHandle(const std::string& name) const
  {
    typename HandleMap::const_iterator it = handle_map_.find(name);
    if (it == handle_map_.end())
    {
      throw TransmissionException("Could not find transmission [" + name + "].");
    }
    return it->second;
  }
  /*\}*/

  /** \name Real-Time Safe Functions
   *\{*/
  /** \brief Propagate the transmission maps of all managed handles. */
  void propagate()
  {
    for (typename HandleMap::iterator it = handle_map_.begin(); it != handle_map_.end(); ++it)
    {
      it->second.propagate();
    }
  }
  /*\}*/
private:
  typedef std::map<std::string, HandleType> HandleMap;
  HandleMap handle_map_;
};

// Convenience typedefs

/** Interface for propagating actuator positions to joint positions on a set of transmissions. */
typedef TransmissionInterface<ActuatorToJointPositionHandle> ActuatorToJointPositionInterface;

/** Interface for propagating actuator velocities to joint velocities on a set of transmissions. */
typedef TransmissionInterface<ActuatorToJointVelocityHandle> ActuatorToJointVelocityInterface;

/** Interface for propagating actuator efforts to joint efforts on a set of transmissions. */
typedef TransmissionInterface<ActuatorToJointEffortHandle> ActuatorToJointEffortInterface;

/** Interface for propagating joint positions to actuator positions on a set of transmissions. */
typedef TransmissionInterface<JointToActuatorPositionHandle> JointToActuatorPositionInterface;

/** Interface for propagating joint velocities to actuator velocities on a set of transmissions. */
typedef TransmissionInterface<JointToActuatorVelocityHandle> JointToActuatorVelocityInterface;

/** Interface for propagating joint efforts to actuator efforts on a set of transmissions. */
typedef TransmissionInterface<JointToActuatorEffortHandle> JointToActuatorEffortInterface;


} // transmission_interface

#endif // TRANSMISSION_INTERFACE_TRANSMISSION_INTERFACE_H
