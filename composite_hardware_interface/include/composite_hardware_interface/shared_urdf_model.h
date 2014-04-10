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

#ifndef COMPOSITE_HARDWARE_INTERFACE__SHARED_URDF_MODEL_H
#define COMPOSITE_HARDWARE_INTERFACE__SHARED_URDF_MODEL_H

#include <ros/ros.h>
#include <urdf/model.h>

namespace composite_hardware_interface
{

/** \brief Shared robot model
 *
 * URDF model instance used thru all robot devices.
 *
 */
class SharedUrdfModel
{
public:

  SharedUrdfModel(const ros::NodeHandle& nh) :
    node_(nh), initialized_(false)
  {
  }

  /** \brief Initializes model from the parameter server.
   *
   * If model already initialized do nothing and return true.
   *
   * \returns True if initializing success
   */
  bool init()
  {
    if(!initialized_)
    {
      std::string robot_description;

      if (!ros::param::search(node_.getNamespace(), "robot_description", robot_description))
      {
        ROS_ERROR("No robot_description parameter found.", node_.getNamespace().c_str());
        return false;
      }

      if (!urdf_model_.initParam(robot_description))
      {
        ROS_ERROR("Could not load robot model from a URDF.", node_.getNamespace().c_str());
        return false;
      }

      initialized_ = true;
    }

    return true;
  }

  /** \brief Return robot URDF model.
   *
   *  If the model is not initialized, initializes it from the parameter server first.
   *
   * \returns Model reference
   */
  const urdf::Model& get()
  {
    init();
    return urdf_model_;
  }

private:
  bool initialized_;
  ros::NodeHandle node_;
  urdf::Model urdf_model_;
};

}

#endif

