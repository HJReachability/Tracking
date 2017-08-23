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
// Defines the NearHoverQuadNoYaw class. Assumes that state 'x' entried are:
// * x(0) -- x
// * x(1) -- x_dot
// * x(2) -- y
// * x(3) -- y_dot
// * x(4) -- z
// * x(5) -- z_dot
//
// Also assumes that entried in control 'u' are:
// * u(0) -- pitch
// * u(1) -- roll
// * u(2) -- thrust
//
///////////////////////////////////////////////////////////////////////////////

#include <meta_planner/near_hover_quad_no_yaw.h>

namespace meta {

// Dimensions.
const size_t NearHoverQuadNoYaw::X_DIM = 6;
const size_t NearHoverQuadNoYaw::U_DIM = 3;

// Factory method. Use this instead of the constructor.
Dynamics::ConstPtr NearHoverQuadNoYaw::Create(const VectorXd& lower_u,
                                              const VectorXd& upper_u) {
  Dynamics::ConstPtr ptr(new NearHoverQuadNoYaw(lower_u, upper_u));
  return ptr;
}

// Derived classes must be able to compute an optimal control given
// the gradient of the value function at the specified state.
// In this case (linear dynamics), the state is irrelevant given the
// gradient of the value function at that state.
VectorXd NearHoverQuadNoYaw::OptimalControl(
  const VectorXd& x, const VectorXd& value_gradient) const {
  // Set each dimension of optimal control to upper/lower bound depending
  // on the sign of the gradient in that dimension. We want to minimize the
  // inner product between the projected gradient and control.
  // If the gradient is 0, then sets control to zero by default.
  VectorXd optimal_control(VectorXd::Zero(lower_u_.size()));
  optimal_control(0) = (value_gradient(1) < 0.0) ? upper_u_(0) : lower_u_(0);
  optimal_control(1) = (value_gradient(3) < 0.0) ? upper_u_(1) : lower_u_(1);
  optimal_control(2) = (value_gradient(5) < 0.0) ? upper_u_(2) : lower_u_(2);

  return optimal_control;
}

// Derived classes must be able to translate a geometric trajectory
// (i.e. through Euclidean space) into a full state space trajectory.
std::vector<VectorXd> NearHoverQuadNoYaw::LiftGeometricTrajectory(
  const std::vector<VectorXd>& states,
  const std::vector<double>& times) const {
  // Number of entries in trajectory.
  size_t num_waypoints = states.size();

#ifdef ENABLE_DEBUG_MESSAGES
  if (states.size() != times.size()) {
    ROS_WARN("Inconsistent number of states, times, and values.");
    num_waypoints = std::min(states.size(), times.size());
  }
#endif

  // Create a clean, empty trajectory.
  std::vector<VectorXd> full_states;
  VectorXd velocity(VectorXd::Zero(X_DIM));

  // Loop through the geometric trajectory and get the velocity with a
  // forward difference.
  for (size_t ii = 0; ii < num_waypoints - 1; ii++) {
    velocity = (states[ii + 1] - states[ii]) / (times[ii + 1] - times[ii]);

    // Populate full state vector.
    VectorXd full(X_DIM);
    full(0) = states[ii](0);
    full(2) = states[ii](2);
    full(4) = states[ii](4);

    full(1) = velocity(0);
    full(3) = velocity(1);
    full(5) = velocity(2);

    // Append to trajectory.
    full_states.push_back(full);
  }

  // Catch final waypoint.
  VectorXd full(X_DIM);
  full(0) = states.back()(0);
  full(2) = states.back()(2);
  full(4) = states.back()(4);

  full(1) = velocity(0);
  full(3) = velocity(1);
  full(5) = velocity(2);

  full_states.push_back(full);

  return full_states;
}

// Private constructor. Use the factory method instead.
NearHoverQuadNoYaw::NearHoverQuadNoYaw(
  const VectorXd& lower_u, const VectorXd& upper_u)
  : Dynamics(lower_u, upper_u) {}

} //\namespace meta
