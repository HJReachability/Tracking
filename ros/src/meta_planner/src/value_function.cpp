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
// Defines the ValueFunction class.
//
///////////////////////////////////////////////////////////////////////////////

#include <meta_planner/value_function.h>

#include <boost/filesystem.hpp>

namespace meta {

namespace fs = boost::filesystem;

// Factory method. Use this instead of the constructor.
// Note that this class is const-only, which means that once it is
// instantiated it can never be changed.
ValueFunction::ConstPtr ValueFunction::
Create(const std::string& directory,
       const Dynamics::ConstPtr& dynamics,
       size_t x_dim, size_t u_dim) {
  ValueFunction::ConstPtr ptr(
    new ValueFunction(directory, dynamics, x_dim, u_dim));
  return ptr;
}

// Constructor. Don't use this. Use the factory method instead.
ValueFunction::ValueFunction(const std::string& directory,
                             const Dynamics::ConstPtr& dynamics,
                             size_t x_dim, size_t u_dim)
  : x_dim_(x_dim),
    u_dim_(u_dim),
    dynamics_(dynamics),
    initialized_(true) {
  // Extract a list of files from this directory.
  std::vector<std::string> file_names;
  const fs::path path(PRECOMPUTATION_DIR + directory);
  for (auto iter = fs::directory_iterator(path);
       iter != fs::directory_iterator();
       iter++) {
    if (fs::is_regular_file(*iter) && iter->path().extension() == ".mat")
      file_names.push_back(iter->path().filename().string());
  }

  if (file_names.size() == 0) {
    ROS_ERROR("No valid files in this directory: %s.",
              std::string(PRECOMPUTATION_DIR + directory).c_str());
    initialized_ = false;
    return;
  }

  // Load each subsystem from file.
  for (const auto& file : file_names) {
    subsystems_.push_back(SubsystemValueFunction::Create(
      PRECOMPUTATION_DIR + directory + file));
    initialized_ &= subsystems_.back()->IsInitialized();
  }

  // Set max planner speed and check consistency.
  max_planner_speed_ = subsystems_.front()->MaxPlannerSpeed();
  for (const auto& subsystem : subsystems_) {
    if (std::abs(max_planner_speed_ - subsystem->MaxPlannerSpeed()) > 1e-8) {
      ROS_ERROR("Max planner speed was not consistent across subsystems.");
      initialized_ = false;
      return;
    }
  }

  // Check that all subsystem dimensions are mutually exclusive and
  // cover the full state space.
  std::unordered_set<size_t> dims;
  for (size_t ii = 0; ii < x_dim_; ii++)
    dims.insert(ii);

  for (const auto& subsystem : subsystems_) {
    for (size_t ii : subsystem->StateDimensions()) {
      if (dims.count(ii) == 0) {
        // Another subsystem already has this dimension.
        ROS_ERROR("Multiple subsystems have dimension %zu.", ii);
        initialized_ = false;
        return;
      }

      // Remove this dimension from the set.
      dims.erase(ii);
    }
  }

  // Check if there are any dimensions remaining.
  if (dims.size() > 0) {
    ROS_ERROR("Not all dimensions are accounted for in ValueFunction.");
    initialized_ = false;
    return;
  }

  // Make sure dynamics pointer is valid.
  if (dynamics_.get() == NULL) {
    ROS_ERROR("Dynamics pointer was null.");
    initialized_ = false;
  }
}

// Combine values of different subsystems.
double ValueFunction::Value(const VectorXd& state) const {
  double max_value = -std::numeric_limits<double>::infinity();

  for (const auto& subsystem : subsystems_)
    max_value = std::max(max_value, subsystem->Value(state));

  return max_value;
}

// Combine gradients from different subsystems.
VectorXd ValueFunction::Gradient(const VectorXd& state) const {
  VectorXd gradient(state.size());

  for (const auto& subsystem : subsystems_) {
    const VectorXd subsystem_gradient = subsystem->Gradient(state);
    const std::vector<size_t>& dims = subsystem->ControlDimensions();

    for (size_t ii = 0; ii < dims.size(); ii++)
      gradient(dims[ii]) = subsystem_gradient(ii);
  }

  return gradient;
}

// Get the tracking error bound in this spatial dimension.
double ValueFunction::TrackingBound(size_t dimension) const {
  bool found = false;

  // Get corresponding full state dimension.
  const size_t full_dim = dynamics_->SpatialDimension(dimension);

  // Loop through all subsystems to find the one containing this dimension.
  // NOTE: if we want to do this frequently, we should just store a map.
  for (const auto& subsystem : subsystems_) {
    const std::vector<size_t>& state_dims = subsystem->StateDimensions();

    for (size_t ii = 0; ii < state_dims.size(); ii++) {
      if (state_dims[ii] == full_dim)
        return subsystem->TrackingBound(full_dim);
    }
  }

  // Catch not found.
  ROS_WARN("Could not find the tracking error bound in dimension %zu.",
           dimension);

  return std::numeric_limits<double>::infinity();
}

} //\namespace meta
