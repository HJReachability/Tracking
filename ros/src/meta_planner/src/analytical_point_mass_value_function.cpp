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
// Defines the AnalyticalPointMassValueFunction class, which inherits from
// the ValueFunction class and implements a custom OptimalControl function.
// Instead of loading subsystems, for simplicity all parameters are read
// from the ROS parameter server and this class does not utilize explicit
// subsystem classes.
//
///////////////////////////////////////////////////////////////////////////////

#include <meta_planner/analytical_point_mass_value_function.h>

namespace meta {

// Factory method. Use this instead of the constructor.
// Note that this class is const-only, which means that once it is
// instantiated it can never be changed. Note that we must pass in
// the maximum planner speed in each geometric dimension.
AnalyticalPointMassValueFunction::ConstPtr AnalyticalPointMassValueFunction::
Create(const Vector3d& max_planner_speed,
       const Dynamics::ConstPtr& dynamics,
       size_t x_dim, size_t u_dim) {
  AnalyticalPointMassValueFunction::ConstPtr ptr(
    new AnalyticalPointMassValueFunction(max_planner_speed, dynamics, x_dim, u_dim));
  return ptr;
}

// Linearly interpolate to get the value/gradient at a particular state.
double AnalyticalPointMassValueFunction::
Value(const VectorXd& state) const {
  // TODO!
  ROS_ERROR("Unimplemented virtual method Value.");
  return 0.0;
}

VectorXd AnalyticalPointMassValueFunction::
Gradient(const VectorXd& state) const {
  // TODO!
  ROS_ERROR("Unimplemented virtual method Gradient.");
  return VectorXd::Zero(x_dim_);
}

// Get the optimal control at a particular state.
VectorXd AnalyticalPointMassValueFunction::
OptimalControl(const VectorXd& state) const {
  // TODO!
  ROS_ERROR("Unimplemented virtual method OptimalControl.");
  return VectorXd::Zero(u_dim_);
}

// Get the tracking error bound in this spatial dimension.
double AnalyticalPointMassValueFunction::
TrackingBound(size_t dimension) const {
  // TODO!
  ROS_ERROR("Unimplemented virtual method TrackingBound.");
  return 0.0;
}

// Constructor.
AnalyticalPointMassValueFunction::
AnalyticalPointMassValueFunction(const Vector3d& max_planner_speed,
                                 const Dynamics::ConstPtr& dynamics,
                                 size_t x_dim, size_t u_dim)
  : ValueFunction(dynamics, x_dim, u_dim),
    max_planner_speed_(max_planner_speed)
    initialized_(false) {
  // TODO!
  ROS_ERROR("Unimplemented constructor for AnalyticalPointMassValueFunction.");
}

} //\namespace meta
