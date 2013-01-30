/*
 * Copyright (c) 2008, Willow Garage, Inc.
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
 * Author: Stuart Glaser
 */
#ifndef REALTIME_PUBLISHER_H
#define REALTIME_PUBLISHER_H

#include <string>
#include <ros/node_handle.h>
#include <boost/utility.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>

namespace realtime_tools {

template <class Msg>
class RealtimePublisher : boost::noncopyable
{

public:
  /// The msg_ variable contains the data that will get published on the ROS topic.
  Msg msg_;
  
  /**  \brief Constructor for the realtime publisher
   *
   * \param node the nodehandle that specifies the namespace (or prefix) that is used to advertise the ROS topic
   * \param topic the topic name to advertise
   * \param queue_size the size of the outgoing ROS buffer
   * \param latched . optional argument (defaults to false) to specify is publisher is latched or not
   */
  RealtimePublisher(const ros::NodeHandle &node, const std::string &topic, int queue_size, bool latched=false)
    : topic_(topic), node_(node), is_running_(false), keep_running_(false), turn_(REALTIME)
  {
    construct(queue_size, latched);
  }

  RealtimePublisher()
    : is_running_(false), keep_running_(false), turn_(REALTIME)
  {
  }

  /// Destructor
  ~RealtimePublisher()
  {
    stop();
    while (is_running())
      usleep(100);

    publisher_.shutdown();
  }

  void init(const ros::NodeHandle &node, const std::string &topic, int queue_size, bool latched=false)
  {
    topic_ = topic;
    node_ = node;
    construct(queue_size, latched);
  }

  /// Stop the realtime publisher from sending out more ROS messages
  void stop()
  {
    keep_running_ = false;
#ifdef NON_POLLING
    updated_cond_.notify_one();  // So the publishing loop can exit
#endif
  }

  /**  \brief Try to get the data lock from realtime
   *
   * To publish data from the realtime loop, you need to run trylock to
   * attempt to get unique access to the msg_ variable. Trylock returns
   * true if the lock was aquired, and false if it failed to get the lock.
   */
  bool trylock()
  {
    if (msg_mutex_.try_lock())
    {
      if (turn_ == REALTIME)
      {
        return true;
      }
      else
      {
        msg_mutex_.unlock();
        return false;
      }
    }
    else
    {
      return false;
    }
  }

  /**  \brief Unlock the msg_ variable
   *
   * After a successful trylock and after the data is written to the mgs_
   * variable, the lock has to be released for the message to get
   * published on the specified topic.
   */
  void unlockAndPublish()
  {
    turn_ = NON_REALTIME;
    msg_mutex_.unlock();
#ifdef NON_POLLING
    updated_cond_.notify_one();
#endif
  }

  /**  \brief Get the data lock form non-realtime
   *
   * To publish data from the realtime loop, you need to run trylock to
   * attempt to get unique access to the msg_ variable. Trylock returns
   * true if the lock was aquired, and false if it failed to get the lock.
   */
  void lock()
  {
#ifdef NON_POLLING
    msg_mutex_.lock();
#else
    // never actually block on the lock
    while (!msg_mutex_.try_lock())
      usleep(200);
#endif
  }

  /**  \brief Unlocks the data without publishing anything
   *
   */
  void unlock()
  {
    msg_mutex_.unlock();
  }

private:
  void construct(int queue_size, bool latched=false)
  {
    publisher_ = node_.advertise<Msg>(topic_, queue_size, latched);
    keep_running_ = true;
    thread_ = boost::thread(&RealtimePublisher::publishingLoop, this);
  }


  bool is_running() const { return is_running_; }

  void publishingLoop()
  {
    is_running_ = true;
    turn_ = REALTIME;

    while (keep_running_)
    {
      Msg outgoing;

      // Locks msg_ and copies it
      lock();
      while (turn_ != NON_REALTIME && keep_running_)
      {
#ifdef NON_POLLING
	updated_cond_.wait(lock);
#else
	unlock();
	usleep(500);
	lock();
#endif
      }
      outgoing = msg_;
      turn_ = REALTIME;

      unlock();

      // Sends the outgoing message
      if (keep_running_)
        publisher_.publish(outgoing);
    }
    is_running_ = false;
  }

  std::string topic_;
  ros::NodeHandle node_;
  ros::Publisher publisher_;
  volatile bool is_running_;
  volatile bool keep_running_;

  boost::thread thread_;

  boost::mutex msg_mutex_;  // Protects msg_

#ifdef NON_POLLING
  boost::condition_variable updated_cond_;
#endif

  enum {REALTIME, NON_REALTIME};
  int turn_;  // Who's turn is it to use msg_?
};

}

#endif
