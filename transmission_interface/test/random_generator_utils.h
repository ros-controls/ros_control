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

#pragma once


#include <cstdlib>
#include <ctime>
#include <vector>

using std::vector;

/// \brief Generator of pseudo-random double in the range [min_val, max_val].
// NOTE: Based on example code available at:
// http://stackoverflow.com/questions/2860673/initializing-a-c-vector-to-random-values-fast
struct RandomDoubleGenerator
{
public:
  RandomDoubleGenerator(double min_val, double max_val)
    : min_val_(min_val),
      max_val_(max_val) {srand(time(nullptr));}
  double operator()()
  {
    const double range = max_val_ - min_val_;
    return rand() / static_cast<double>(RAND_MAX) * range + min_val_;
  }
private:
  double min_val_;
  double max_val_;
};

/// \brief Generator of a vector of pseudo-random doubles.
vector<double> randomVector(const vector<double>::size_type size, RandomDoubleGenerator& generator)
{
  vector<double> out;
  out.reserve(size);
  for (vector<double>::size_type i = 0; i < size; ++i) {out.push_back(generator());}
  return out;
}
