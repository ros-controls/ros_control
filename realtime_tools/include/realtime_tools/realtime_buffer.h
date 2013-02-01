/*
 * Copyright (c) 2013 hiDOF, Inc.
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

/*
 * Publishing ROS messages is difficult, as the publish function is
 * not realtime safe.  This class provides the proper locking so that
 * you can call publish in realtime and a separate (non-realtime)
 * thread will ensure that the message gets published over ROS.
 *
 * Author: Wim Meeussen
 */

#ifndef REALTIME_TOOLS_REALTIME_BUFFER
#define REALTIME_TOOLS_REALTIME_BUFFER

#include <boost/thread/mutex.hpp>

namespace realtime_tools
{

template <class T>
class RealtimeBuffer
{
 public:
  RealtimeBuffer()
    :new_data_available_(false), running_(true)
  {
    // allocate memory
    non_realtime_data_ = new T();
    realtime_data_ = new T();
  }

  ~RealtimeBuffer()
  {
    if (non_realtime_data_)
      delete non_realtime_data_;
    if (realtime_data_)
      delete realtime_data_;
  }


  T* readFromRT()
  {
    if (mutex_.try_lock())
    {
      // swap pointers
      if (new_data_available_)
      {
	T* tmp = realtime_data_;
	realtime_data_ = non_realtime_data_;
	non_realtime_data_ = tmp;
	new_data_available_ = false;
      }
      mutex_.unlock();
    }
    return realtime_data_;
  }



  void writeFromNonRT(const T& data)
  {
    // get lock
    lock();

    // copy data into non-realtime buffer
    *non_realtime_data_ = data;
    new_data_available_ = true;

    // release lock
    mutex_.unlock();
  }


  void initRT(const T& data)
  {
    *non_realtime_data_ = data;    
    *realtime_data_ = data;    
  }

 private:
  void lock()
  {
#ifdef NON_POLLING
    mutex_.lock();
#else
    while (!mutex_.try_lock())
      usleep(500);
#endif
  }

  T* realtime_data_;
  T* non_realtime_data_;
  bool new_data_available_;
  bool running_;
  boost::mutex mutex_;


}; // class
}// namespace

#endif
