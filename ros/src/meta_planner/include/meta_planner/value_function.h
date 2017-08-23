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

#ifndef META_PLANNER_VALUE_FUNCTION_H
#define META_PLANNER_VALUE_FUNCTION_H

#include <meta_planner/subsystem_value_function.h>
#include <meta_planner/dynamics.h>
#include <meta_planner/types.h>
#include <meta_planner/uncopyable.h>

#include <ros/ros.h>
#include <limits>
#include <unordered_set>
#include <memory>

namespace meta {

class ValueFunction : private Uncopyable {
public:
  typedef std::shared_ptr<const ValueFunction> ConstPtr;

  // Destructor.
  ~ValueFunction() {}

  // Factory method. Use this instead of the constructor.
  // Note that this class is const-only, which means that once it is
  // instantiated it can never be changed.
  static ConstPtr Create(const std::string& directory,
                         const Dynamics::ConstPtr& dynamics,
                         size_t x_dim, size_t u_dim);

  // Linearly interpolate to get the value/gradient at a particular state.
  double Value(const VectorXd& state) const;
  VectorXd Gradient(const VectorXd& state) const;

  // Get the optimal control at a particular state.
  inline VectorXd OptimalControl(const VectorXd& state) const {
    return dynamics_->OptimalControl(state, Gradient(state));
  }

  // Get the tracking error bound in the subsystem containing this dimension.
  double TrackingBound(size_t dimension) const;

  // Max planner speed.
  inline double MaxPlannerSpeed() const { return max_planner_speed_; }

  // Was this ValueFunction properly initialized?
  inline bool IsInitialized() const { return initialized_; }

private:
  explicit ValueFunction(const std::string& directory,
                         const Dynamics::ConstPtr& dynamics,
                         size_t x_dim, size_t u_dim);

  // State/control space dimensions.
  const size_t x_dim_;
  const size_t u_dim_;

  // Dynamics.
  const Dynamics::ConstPtr dynamics_;

  // Planner max speed.
  double max_planner_speed_;

  // List of value functions for independent subsystems.
  std::vector<SubsystemValueFunction::ConstPtr> subsystems_;

  // Was this value function initialized/loaded properly?
  bool initialized_;
};

} //\namespace meta

#endif
