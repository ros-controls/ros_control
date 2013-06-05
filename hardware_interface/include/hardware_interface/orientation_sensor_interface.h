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

/// \author: Adolfo Rodriguez Tsouroukdissian

#ifndef HARDWARE_INTERFACE_ORIENTATION_SENSOR_INTERFACE_H
#define HARDWARE_INTERFACE_ORIENTATION_SENSOR_INTERFACE_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <hardware_interface/internal/demangle_symbol.h>
#include <hardware_interface/internal/named_resource_manager.h>
#include <hardware_interface/hardware_interface.h>
#include <string>
#include <vector>

namespace hardware_interface
{

/// A handle used to read the state of an orientation sensor.
class OrientationSensorHandle
{
public:
  typedef Eigen::Map<Eigen::Quaternion<double> > Quaternion;
  typedef Covariance                             OrientationCovariance;

  OrientationSensorHandle(const std::string& name, double* orientation, double* orientation_covariance)
    : name_(name),
      orientation_(orientation),
      orientation_covariance_(orientation_covariance)
  {}

  std::string getName()                                   const {return name_;}
  const Quaternion& getOrientation()                      const {return orientation_;}
  const OrientationCovariance& getOrientationCovariance() const {return orientation_covariance_;}

private:
  std::string           name_;
  Quaternion            orientation_;
  OrientationCovariance orientation_covariance_;
};

class OrientationSensorInterface : public HardwareInterface
{
public:
  /// Get the vector of orientation sensor names registered to this interface.
  std::vector<std::string> getSensorNames() const
  {
    return handle_map_.getNames();
  }

  /** \brief Register a new orientation sensor with this interface.
   *
   * \param name The name of the new sensor
   * \param orientation A pointer to the storage of the orientation value: a quaternion (x,y,z,w)
   * \param orientation_covariance A pointer to the storage of the orientation covariance value:
   *        a row major matrix about (x,y,z)
   *
   */
  void registerSensor(const std::string& name,
                      double* orientation,
                      double* orientation_covariance)
  {
    OrientationSensorHandle handle(name,
                                   orientation,
                                   orientation_covariance);
    handle_map_.insert(name, handle);
  }

  /** \brief Get a \ref OrientationSensorHandle for accessing a orientation sensor's state.
   *
   * \param name The name of the sensor
   * \return A \ref OrientationSensorHandle corresponding to the sensor identified by \c name
   *
   */
  OrientationSensorHandle getSensorHandle(const std::string& name)
  {
    try
    {
      return handle_map_.get(name);
    }
    catch(...)
    {
      throw HardwareInterfaceException("Could not find orientation sensor '" + name + "' in " +
                                       internal::demangledTypeName(*this));
    }
  }

protected:
  internal::NamedResourceManager<OrientationSensorHandle> handle_map_;
};

}

#endif // HARDWARE_INTERFACE_ORIENTATION_SENSOR_INTERFACE_H
