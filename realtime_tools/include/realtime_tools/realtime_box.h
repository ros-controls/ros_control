/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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

// Author: Stuart Glaser

#ifndef REALTIME_BOX_H__
#define REALTIME_BOX_H__

#include <string>
#include <ros/node_handle.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

namespace realtime_tools {

/*!

  Strongly suggested that you use a boost::shared_ptr in this box to
  guarantee realtime safety.

 */
template <class T>
class RealtimeBox
{
public:
  RealtimeBox(const T &initial = T()) : thing_(initial)
  {
  }

  void set(const T &value)
  {
    boost::mutex::scoped_lock guard(thing_lock_RT_);
    thing_ = value;
  }

  void get(T &ref)
  {
    boost::mutex::scoped_lock guard(thing_lock_RT_);
    ref = thing_;
  }

private:
  // The thing that's in the box.
  T thing_;

  // Protects access to the thing in the box.  This mutex is
  // guaranteed to be locked for no longer than the duration of the
  // copy, so as long as the copy is realtime safe and the OS has
  // priority inversion for mutexes, this lock can be safely locked
  // from within realtime.
  boost::mutex thing_lock_RT_;
};

} // namespace

#endif
