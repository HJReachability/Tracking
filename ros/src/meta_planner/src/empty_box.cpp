/*
 * Copyright (c) 2017, The Regents of the University of California (Regents).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Please contact the author(s) of this library if you have any questions.
 * Authors: David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// Defines an empty n-dimensional box which inherits from Environment.
// Defaults to the unit box.
//
///////////////////////////////////////////////////////////////////////////////

#include <meta_planner/empty_box.h>

#include <ros/ros.h>

EmptyBox::EmptyBox(size_t dimension)
  : Environment(),
    dimension_(dimension),
    min_(0.0),
    max_(1.0) {}

// Derived classes must be able to sample uniformly from the state space.
VectorXd EmptyBox::Sample() {
  // Create a uniform distribution on the proper support for each dimension.
  std::uniform_real_distribution<double> unif(min_, max_);

  // Sample each dimension from this distribution.
  VectorXd sample(dimension_);
  for (size_t ii = 0; ii < dimension_; ii++)
    sample(ii) = unif(rng_);

  return sample;
}

// Derived classes must provide a collision checker which returns true if
// and only if the provided state is a valid collision-free configuration.
bool EmptyBox::IsValid(const VectorXd& state) const {
#ifdef ENABLE_DEBUG_MESSAGES
  if (state.size() != dimension_)
    ROS_ERROR("Improperly-sized state vector (%zu vs. %zu).",
              state.size(), dimension_);
#endif

  // No obstacles. Just check bounds.
  for (size_t ii = 0; ii < state.size(); ii++)
    if (state(ii) < min_ || state(ii) > max_)
      return false;

  return true;
}

// Set bounds. For simplicity we assume that the min/max is the same
// in all dimensions.
void EmptyBox::SetBounds(double min, double max) {
  min_ = min;
  max_ = max;
}
