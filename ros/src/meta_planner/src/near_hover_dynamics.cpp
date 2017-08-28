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
 * Authors: Jaime Fernandez Fisac   ( jfisac@eecs.berkeley.edu )
 *          David Fridovich-Keil    ( dfk@eecs.berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// Defines the NearHoverDynamics class (7D quadrotor model used by Crazyflie).
//
///////////////////////////////////////////////////////////////////////////////

// System dynamics
// (quadrotor in near hover, assuming small attitude angles)
//    dx/dt     =  vx_G
//    dy/dt     =  vy_G
//    dz/dt     =  vz_G
//    dvx_G/dt  =  T*sin(theta)*cos(psi) - T*sin(phi)*sin(psi)
//    dvy_G/dt  =  T*sin(phi)*cos(psi)   + T*sin(theta)*sin(psi)
//    dvz_G/dt  =  T*cos(phi)*cos(theta) - g
//    dpsi/dt   =  w
//
//    control: u = [ phi, theta, w, T]
//
// Note: angle sign convention is based on positive accelerations on FLU frame
// (theta PITCH DOWN / phi ROLL LEFT / psi YAW left)

#include <meta_planner/linear_dynamics.h>

// Factory method. Use this instead of the constructor.
Dynamics::ConstPtr NearHoverDynamics::Create(const double g,
                                             const VectorXd& lower_u,
                                             const VectorXd& upper_u) {
  Dynamics::ConstPtr ptr(new NearHoverDynamics(g, lower_u, upper_u));
  return ptr;
}

// Derived from parent virtual operator, gives the time derivative of state
// as a function of current state and control. See above description for details.
VectorXd NearHoverDynamics::operator()(const VectorXd& x, const VectorXd& u) const {
  VectorXd& xdot;
  xdot[0] = x[3];
  xdot[1] = x[4];
  xdot[2] = x[5];
  xdot[3] = u[3]*sin(u[1])*cos(x[6]) - u[3]*sin(u[0])*sin(x[6]);
  xdot[4] = u[3]*sin(u[0])*cos(x[6]) + u[3]*sin(u[1])*sin(x[6]);
  xdot[5] = u[3]*cos(u[0])*cos(u[1]) - g_;
  xdot[6] = u[2];
  return xdot;
}

// Derived classes must be able to compute an optimal control given
// the gradient of the value function at the specified state.
// This function is currently not implemented for NearHover.
VectorXd NearHoverDynamics::OptimalControl(const VectorXd& x,
                                        const VectorXd& value_gradient) const {

  // The optimal control is the solution to a nonconvex optimization problem
  // with the inner product <grad(V),xdot(x,u)> as the objective.
  // No solution method is currently implemented.
  // Instead, the optimal control is set to zero by default.
  VectorXd optimal_control(VectorXd::Zero(lower_u_.size()));

  return optimal_control;
}

// Private constructor. Use the factory method instead.
NearHoverDynamics::NearHoverDynamics(const double g,
                                     const VectorXd& lower_u,
                                     const VectorXd& upper_u)
  : Dynamics(lower_u, upper_u), g_(g) {}
