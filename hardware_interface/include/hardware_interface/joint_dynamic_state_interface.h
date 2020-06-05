///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2020, CNRS
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

/// \author Olivier Stasse
/// \author Wim Meeussen

#pragma once


#include <hardware_interface/internal/hardware_resource_manager.h>
#include <cassert>
#include <string>

namespace hardware_interface
{

/**
 * \brief A handle used to read the state of a single joint.
 * This class is generic and handle a std::vector<double>.
 */
class JointDynamicStateHandle
{
public:
  JointDynamicStateHandle() = delete;

  /**
   * \param name The name of the joint
   * \param interface_name The interface name, preferably this should follow
   * the standard sets by ros_control for interoperability among controllers.
   * \param state_vector The vector of informations.
   */
  JointDynamicStateHandle(const std::string& name,
                   const std::string& interface_name,
                   const std::vector<double> * state_vector)
    : name_(name),
      interface_name_(interface_name),
      state_vector_(state_vector)
  {
    if (!state_vector)
    {      throw HardwareInterfaceException
          ("Cannot create handle '" +
           name + " for interface " +
           interface_name +
           "'. state_vector is null.");
    }
  }

  JointDynamicStateHandle
  (const JointDynamicStateHandle &joint_dyn_handle)
      :name_(joint_dyn_handle.getName()),
       interface_name_(joint_dyn_handle.getInterfaceName()),
       state_vector_(joint_dyn_handle.getStateVectorPtr())
  {
  }
      
  JointDynamicStateHandle & operator=
  (const JointDynamicStateHandle &joint_dyn_handle)
  {
    name_ =joint_dyn_handle.getName();
    interface_name_=joint_dyn_handle.getInterfaceName();
    state_vector_=joint_dyn_handle.getStateVectorPtr();
    return *this;
  }
  
  std::string getName() const {return name_;}
  std::string getInterfaceName() const {return interface_name_;}
        
  const std::vector<double> &
  getStateVector() const
  {
    assert(state_vector_);
    return *state_vector_;
  }

  const std::vector<double> *
  getStateVectorPtr() const
  { return state_vector_; }
  
private:
  std::string name_;
  std::string interface_name_;
  const std::vector<double> * state_vector_;
};

/** \brief Hardware interface to support reading the state of an array of joints
 *
 * This \ref HardwareInterface supports reading the state of an array of named
 * joints, each of which has a std::vector.
 *
 */
class JointDynamicStateInterface :
      public HardwareResourceManager<JointDynamicStateHandle> {};

}
