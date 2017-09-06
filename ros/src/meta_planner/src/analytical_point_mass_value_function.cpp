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
 * Authors: David Fridovich-Keil    ( dfk@eecs.berkeley.edu )
 *          Jaime Fernandez Fisac   ( jfisac@eecs.berkeley.edu )
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

const size_t AnalyticalPointMassValueFunction::p_dim_ = 3;

// Factory method. Use this instead of the constructor.
// Note that this class is const-only, which means that once it is
// instantiated it can never be changed. Note that we must pass in
// the maximum planner speed in each geometric dimension.
AnalyticalPointMassValueFunction::ConstPtr AnalyticalPointMassValueFunction::
Create(const Vector3d& max_planner_speed,
       const Vector3d& max_tracker_control,
       const Vector3d& min_tracker_control,
       const Vector3d& max_vel_disturbance,
       const Vector3d& max_acc_disturbance,
       const Dynamics::ConstPtr& dynamics,
       ValueFunctionId id) {
  AnalyticalPointMassValueFunction::ConstPtr ptr(
    new AnalyticalPointMassValueFunction(max_planner_speed, max_tracker_control,
                                         min_tracker_control, max_vel_disturbance,
                                         max_acc_disturbance, dynamics, id));
  return ptr;
}

// Analytically evaluate value/gradient at a particular state.
double AnalyticalPointMassValueFunction::
Value(const VectorXd& state) const {
  double V = 0;
  for (size_t dim = 0; dim < p_dim_; dim++){
    const double x = state(dim);
    const double v = state(p_dim_ + dim);
    const double v_ref = max_planner_speed_(dim);

    // Value surface A: + for x "below" convex Acceleration parabola.
    const double V_A = -x +
      (0.5 * (v - v_ref)*(v - v_ref) - v_ref*v_ref) /
      (a_max_(dim) - d_a_(dim));
    //    const double V_A = ( 1/2*pow(v-v_ref,2) - pow(v_ref,2) )
    //      /   ( a_max_(dim) - d_a_(dim) ) - x;

    // Value surface B: + for x "above" concave Braking parabola.
    const double V_B = x -
      (-0.5 * (v + v_ref)*(v + v_ref) + v_ref*v_ref) /
      (a_max_(dim) - d_a_(dim));
    //    const double V_B = x - ( -1/2*pow(v+v_ref,2) + pow(v_ref,2) )
    //      /   ( a_max_(dim) - d_a_(dim) );

    // Value function is the maximum of the above two surfaces.
    const double V_this_dim = std::max(V_A, V_B);
    V = std::max(V, V_this_dim);
  }

  return V;
}

VectorXd AnalyticalPointMassValueFunction::
Gradient(const VectorXd& state) const {
  VectorXd grad_V = VectorXd::Zero(x_dim_);

  // Loop through each subsystem and populate grad_V.
  for (size_t dim = 0; dim < p_dim_; dim++){
    const double x = state(dim);
    const double v = state(p_dim_ + dim);
    const double v_ref = max_planner_speed_(dim);

    // Value surface A: + for x "below" convex Acceleration parabola.
    const double V_A = -x +
      (0.5 * (v - v_ref)*(v - v_ref) - v_ref*v_ref) /
      (a_max_(dim) - d_a_(dim));
    // Value surface B: + for x "above" concave Braking parabola.
    const double V_B = x -
      (-0.5 * (v + v_ref)*(v + v_ref) + v_ref*v_ref) /
      (a_max_(dim) - d_a_(dim));
    if (V_A > V_B) {
      grad_V(dim) = -1.0;         // if on A side, grad points towards -pos
      grad_V(p_dim_ + dim) = (v - v_ref) / (a_max_(dim) - d_a_(dim));
    } else {
      grad_V(dim) = 1.0;          // if on B side, grad points towards +pos
      grad_V(p_dim_ + dim) = (v + v_ref) / (a_max_(dim) - d_a_(dim));
    }
  }

  return grad_V;
}

// Get the optimal control at a particular state.
VectorXd AnalyticalPointMassValueFunction::
OptimalControl(const VectorXd& state) const {
  Vector3d u_opt = Vector3d::Zero();

  for (size_t dim = 0; dim < p_dim_; dim++){
    const double x = state(dim);
    const double v = state(p_dim_+dim);
    const double v_ref = max_planner_speed_(dim);
    VectorXd grad_V = VectorXd::Zero(x_dim_);

    // Value surface A: + for x "below" convex Acceleration parabola.
    const double V_A = -x +
      (0.5 * (v - v_ref)*(v - v_ref) - v_ref*v_ref) /
      (a_max_(dim) - d_a_(dim));
    // Value surface B: + for x "above" concave Braking parabola.
    const double V_B = x -
      (-0.5 * (v + v_ref)*(v + v_ref) + v_ref*v_ref) /
      (a_max_(dim) - d_a_(dim));
    if (V_A > V_B) {
      // If on A side, accelerate.
      u_opt(dim) = u2a_(dim) > 0.0 ? u_max_(dim) : u_min_(dim);
    } else {
      // If on B side, brake.
      u_opt(dim) = u2a_(dim) > 0.0 ? u_min_(dim) : u_max_(dim);
    }
  }

  return u_opt;
}

// Priority of the optimal control at the given state. This is a number
// between 0 and 1, where 1 means the final control signal should be exactly
// the optimal control signal computed by this value function.
double AnalyticalPointMassValueFunction::Priority(const VectorXd& state) const {
  const double V = Value(state);
  //HACK! JFF NOTE: The threshold should probably be externally set via config
  const double relative_high = 0.50; // 50% of max inside value
  const double relative_low  = 0.05; // 5% of max inside value
  const double V_safest = Value(VectorXd::Zero(6));
  const double V_high = relative_high * V_safest;
  const double V_low = relative_low * V_safest;
  double priority = 1.0 - std::min(std::max(0.0,(V-V_low)/(V_high-V_low)),1.0);
  return priority;
}

// Get the tracking error bound in this spatial dimension.
double AnalyticalPointMassValueFunction::
TrackingBound(size_t dim) const {
  // Return a single positive number (semi-length of interval centered on 0)
  // This is equal to the position at the intersection between parabolas.
  const double v_ref = max_planner_speed_(dim);
  return 0.5 * v_ref*v_ref / (a_max_(dim) - d_a_(dim));
}

// Constructor.
AnalyticalPointMassValueFunction::
AnalyticalPointMassValueFunction(const Vector3d& max_planner_speed,
                                 const Vector3d& max_tracker_control,
                                 const Vector3d& min_tracker_control,
                                 const Vector3d& max_vel_disturbance,
                                 const Vector3d& max_acc_disturbance,
                                 const Dynamics::ConstPtr& dynamics,
                                 ValueFunctionId id)
  : ValueFunction(dynamics, 6, 3, id),
    u_max_(max_tracker_control),
    u_min_(min_tracker_control),
    d_v_(max_vel_disturbance),
    d_a_(max_acc_disturbance) {

  // Compute max acceleration (NOTE: assumed symmetric even if u_max != -u_min).
  const VectorXd x_dot_max = dynamics_->Evaluate(VectorXd::Zero(6), u_max_);
  a_max_ = x_dot_max.tail<3>().cwiseAbs();
  // Compute control gains.
  u2a_ = x_dot_max.tail<3>().cwiseQuotient( 0.5 * (u_max_ - u_min_) );

  // Set max planner speed.
  max_planner_speed_ = max_planner_speed; // v_ref is a duplicate of this

}

} //\namespace meta
