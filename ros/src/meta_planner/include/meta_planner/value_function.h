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
// Defines the ValueFunction class. Many functions in this class are declared
// virtual so that analytical value functions may inherit from this class.
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
  virtual ~ValueFunction() {}

  // Factory method. Use this instead of the constructor.
  // Note that this class is const-only, which means that once it is
  // instantiated it can never be changed.
  static ConstPtr Create(const std::string& directory,
                         const Dynamics::ConstPtr& dynamics,
                         size_t x_dim, size_t u_dim, ValueFunctionId id);

  // Linearly interpolate to get the value/gradient at a particular state.
  virtual double Value(const VectorXd& state) const;
  virtual VectorXd Gradient(const VectorXd& state) const;

  // Get the optimal control at a particular state.
  virtual inline VectorXd OptimalControl(const VectorXd& state) const {
    return dynamics_->OptimalControl(state, Gradient(state));
  }

  // Get the tracking error bound in this spatial dimension.
  virtual double TrackingBound(size_t dimension) const;

  // Priority of the optimal control at the given state. This is a number
  // between 0 and 1, where 1 means the final control signal should be exactly
  // the optimal control signal computed by this value function.
  virtual double Priority(const VectorXd& state) const;

  // Get the dynamics.
  inline Dynamics::ConstPtr GetDynamics() const { return dynamics_; }

  // Max planner speed in the given spatial dimension.
  inline double MaxPlannerSpeed(size_t ii) const {
    return max_planner_speed_(ii);
  }

  // Get the ID of this value function.
  inline ValueFunctionId Id() const { return id_; }

  // Was this ValueFunction properly initialized?
  inline bool IsInitialized() const { return initialized_; }

protected:
  // Constuctor for use by derived classes.
  explicit ValueFunction(const Dynamics::ConstPtr& dynamics,
                         size_t x_dim, size_t u_dim, ValueFunctionId id)
    : dynamics_(dynamics),
      x_dim_(x_dim),
      u_dim_(u_dim),
      id_(id),
      initialized_(true) {
    if (!dynamics.get())
      ROS_ERROR("Dynamics pointer was null.");
  }

  // Identifier.
  const ValueFunctionId id_;

  // State/control space dimensions.
  const size_t x_dim_;
  const size_t u_dim_;

  // Dynamics.
  const Dynamics::ConstPtr dynamics_;

  // Planner max speed in each spatial dimension.
  Vector3d max_planner_speed_;

  // Was this value function initialized/loaded properly?
  bool initialized_;

private:
  // Constructor for use by this class.
  explicit ValueFunction(const std::string& directory,
                         const Dynamics::ConstPtr& dynamics,
                         size_t x_dim, size_t u_dim, ValueFunctionId id);

  // List of value functions for independent subsystems.
  std::vector<SubsystemValueFunction::ConstPtr> subsystems_;
};

} //\namespace meta

#endif
