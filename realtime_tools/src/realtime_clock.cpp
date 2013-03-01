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

#include <realtime_tools/realtime_clock.h>

namespace realtime_tools
{

  
  RealtimeClock::RealtimeClock()
    :lock_misses_(0), 
     system_time_(ros::Time()), 
     last_realtime_time_(ros::Time()),
     running_(true),
     initialized_(false)
  {
    // thread for time loop
    thread_ = boost::thread(boost::bind(&RealtimeClock::loop, this));
  }
  

  RealtimeClock::~RealtimeClock()
  {
    running_ = false;
    thread_.join();
  }



  ros::Time RealtimeClock::getSystemTime(const ros::Time& realtime_time)
  {
    if (mutex_.try_lock())
    {
      // update time offset when we have a new system time measurement in the last cycle
      if (lock_misses_ == 0 && system_time_ != ros::Time())
      {
	// get additional offset caused by period of realtime loop
	ros::Duration period_offset;
	if (last_realtime_time_ != ros::Time())
	  period_offset = ros::Duration((realtime_time - last_realtime_time_).toSec()/2.0);

	if (!initialized_)
        {
	  clock_offset_ = system_time_ + period_offset - realtime_time;
	  initialized_ = true;
	}
	else
	  clock_offset_ = clock_offset_*0.9999 + (system_time_ + period_offset - realtime_time)*0.0001;
      }
      system_time_ = ros::Time();
      lock_misses_ = 0;
      mutex_.unlock();
    }
	
    else
      lock_misses_++;

    last_realtime_time_ = realtime_time;

    // return time
    return realtime_time + clock_offset_;
  }



  void RealtimeClock::loop()
  {
    ros::Rate r(750);
    while (running_)
    {
      // get lock
      lock();

      // store system time
      system_time_ = ros::Time::now();
      
      // warning, using non-locked 'lock_misses_', but it's just for debugging
      if (lock_misses_ > 100)
	ROS_WARN_THROTTLE(1.0, "Time estimator has trouble transferring data between non-RT and RT");

      // release lock
      mutex_.unlock();
      r.sleep();
    }
  }


  void RealtimeClock::lock()
  {
#ifdef NON_POLLING
    mutex_.lock();
#else
    while (!mutex_.try_lock())
      usleep(500);
#endif
  }

}// namespace

