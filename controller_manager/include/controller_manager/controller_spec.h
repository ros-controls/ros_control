///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012, hiDOF, INC and Willow Garage, Inc
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Willow Garage Inc, hiDOF Inc, nor the names of its
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
 * Author: Wim Meeussen
 */


#ifndef CONTROLLER_MANAGER_CONTROLLER_SPEC_H
#define CONTROLLER_MANAGER_CONTROLLER_SPEC_H

#pragma GCC diagnostic ignored "-Wextra"

#include <map>
#include <string>
#include <vector>
#include <controller_interface/controller_base.h>
#include <boost/circular_buffer.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>

namespace controller_manager
{

typedef boost::accumulators::accumulator_set<
  double, boost::accumulators::stats<boost::accumulators::tag::max,
                                     boost::accumulators::tag::mean,
                                     boost::accumulators::tag::variance> > TimeStatistics;

struct Statistics {
  TimeStatistics acc;
  ros::Time time_last_control_loop_overrun;
  unsigned int num_control_loop_overruns;
  double max;
  boost::circular_buffer<double> max1;
Statistics() : num_control_loop_overruns(0), max(0), max1(60) {}
};

struct ControllerSpec {
  std::string name;
  std::string type;
  boost::shared_ptr<controller_interface::ControllerBase> c;
  boost::shared_ptr<Statistics> stats;

  ControllerSpec() : stats(new Statistics) {}
  ControllerSpec(const ControllerSpec &spec)
    : name(spec.name), type(spec.type), c(spec.c), stats(spec.stats) {}
};

}

#endif

