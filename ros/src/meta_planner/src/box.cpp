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
// Defines an n-dimensional box which inherits from Environment. Defaults to
// the unit box.
//
///////////////////////////////////////////////////////////////////////////////

#include <meta_planner/box.h>

// Factory method. Use this instead of the constructor.
Box::Ptr Box::Create(size_t dimension) {
  Box::Ptr ptr(new Box(dimension));
  return ptr;
}

// Constructor. Don't use this. Use the factory method instead.
Box::Box(size_t dimension)
  : Environment(),
    dimension_(dimension),
    lower_(VectorXd::Zero(dimension)),
    upper_(VectorXd::Constant(dimension, 1.0)) {}

// Inherited from Environment, but can be overriden by child classes.
VectorXd Box::Sample() const {
  VectorXd sample(dimension_);

  // Sample each dimension from this distribution.
  for (size_t ii = 0; ii < dimension_; ii++) {
    std::uniform_real_distribution<double> unif(lower_(ii), upper_(ii));
    sample(ii) = unif(rng_);
  }

  return sample;
}

// Inherited from Environment, but can be overriden by child classes.
// Returns true if the state is a valid configuration.
bool Box::IsValid(const VectorXd& state) const {
#ifdef ENABLE_DEBUG_MESSAGES
  if (state.size() != dimension_)
    ROS_ERROR("Improperly sized state vector (%zu vs. %zu).",
              state.size(), dimension_);
#endif

  // No obstacles. Just check bounds.
  for (size_t ii = 0; ii < state.size(); ii++)
    if (state(ii) < lower_(ii) || state(ii) > upper_(ii))
      return false;

  return true;
}

// Set bounds in each dimension.
void Box::SetBounds(const VectorXd& lower, const VectorXd& upper) {
#ifdef ENABLE_DEBUG_MESSAGES
  if (lower.size() != dimension_ || upper.size() != dimension_) {
    ROS_ERROR("Improperly sized lower/upper bounds. Did not set.");
    return;
  }
#endif

  lower_ = lower;
  upper_ = upper;
}

// Get the lower bounds at the specified dimensions.
VectorXd Box::LowerBounds(const std::vector<size_t>& dimensions) const {
  VectorXd punctured = VectorXd::Zero(dimensions.size());

  for (size_t ii = 0; ii < dimensions.size(); ii++) {
#ifdef ENABLE_DEBUG_MESSAGES
    if (dimensions[ii] >= dimension_) {
      ROS_ERROR("Tried to access bound for a non-existent dimension: %zu.",
                dimensions[ii]);
      continue;
    }
#endif

    punctured[ii] = lower_[dimensions[ii]];
  }

  return punctured;
}

// Get the upper bounds at the specified dimensions.
VectorXd Box::UpperBounds(const std::vector<size_t>& dimensions) const {
  VectorXd punctured = VectorXd::Zero(dimensions.size());

  for (size_t ii = 0; ii < dimensions.size(); ii++) {
#ifdef ENABLE_DEBUG_MESSAGES
    if (dimensions[ii] >= dimension_) {
      ROS_ERROR("Tried to access bound for a non-existent dimension: %zu.",
                dimensions[ii]);
      continue;
    }
#endif

    punctured[ii] = upper_[dimensions[ii]];
  }

  return punctured;
}
