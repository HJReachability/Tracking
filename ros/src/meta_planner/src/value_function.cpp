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
  }

  // Load each subsystem from file.
  for (const auto& file : file_names) {
    subsystems_.push_back(SubsystemValueFunction::Create(
      PRECOMPUTATION_DIR + directory + file));
    initialized_ &= subsystems_.back()->IsInitialized();
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

// Get the tracking error bound in the subsystem containing this dimension.
double ValueFunction::TrackingBound(size_t dimension) const {
  bool found = false;

  // Loop through all subsystems to find the one containing this dimension.
  // NOTE: if we want to do this frequently, we should just store a map.
  for (const auto& subsystem : subsystems_) {
    for (size_t ii : subsystem->StateDimensions())
      if (ii == dimension) {
        found = true;
        break;
      }

    if (found)
      return subsystem->TrackingBound();
  }

  // Catch not found.
  if (!found)
    ROS_WARN("Could not find the tracking error bound in dimension %zu.",
             dimension);

  return std::numeric_limits<double>::infinity();
}

} //\namespace meta
