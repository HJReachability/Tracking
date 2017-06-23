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
// Defines the RrtConnect class, which wraps the OMPL class of the same name
// and inherits from the Planner abstract class. For simplicity, we assume that
// the state space is a real-valued vector space with box constraints, i.e.
// an instance of the Box subclass of Environment.
//
// We follow these ( http://ompl.kavrakilab.org/geometricPlanningSE3.html )
// instructions for using OMPL geometric planners.
//
// TODO: This can easily be turned into a generic wrapper for any OMPL
// geometric planner.
//
///////////////////////////////////////////////////////////////////////////////

#include <meta_planner/rrt_connect.h>

RrtConnect::RrtConnect(double velocity)
  : Planner(),
    velocity_((velocity <= 0.0) ? 1.0 : velocity) {
#ifdef ENABLE_DEBUG_MESSAGES
  if (velocity_ <= 0.0)
    ROS_ERROR("Velocity was negative. Setting to unity.");
#endif
}

// Derived classes must plan trajectories between two points.
Trajectory RrtConnect::Plan(const VectorXd& start, const VectorXd& stop,
                            const Box& space) const {
#ifdef ENABLE_DEBUG_MESSAGES
  if (start.size() != stop.size() || start.size() != space.Dimension()) {
    ROS_ERROR("Start/stop state dimensions inconsistent with space dimension.");
    return Trajectory();
  }
#endif

  // Create the OMPL state space corresponding to this environment.
  auto ompl_space(std::make_shared<ob::RealVectorStateSpace>());

  // Set bounds for the environment.
  const VectorXd& lower = space.LowerBounds();
  const VectorXd& upper = space.UpperBounds();
  ob::RealVectorBounds ompl_bounds(space.Dimension());

  for (size_t ii = 0; ii < space.Dimension(); ii++) {
    ompl_bounds.setLow(ii, lower(ii));
    ompl_bounds.setHigh(ii, upper(ii));
  }

  ompl_space->setBounds(ompl_bounds);

  // Create a SimpleSetup instance and set the state validity checker function.
  og::SimpleSetup ompl_setup(ompl_space);
  ompl_setup.setStateValidityChecker([&](const ob::State* state) {
      return space.IsValid(FromOmplState(state, space.Dimension())); });

  // Set the start and stop states.
  ob::ScopedState<ob::RealVectorStateSpace> ompl_start(ompl_space);
  ob::ScopedState<ob::RealVectorStateSpace> ompl_stop(ompl_space);
  for (size_t ii = 0; ii < space.Dimension(); ii++) {
    ompl_start[ii] = start(ii);
    ompl_stop[ii] = stop(ii);
  }

  ompl_setup.setStartAndGoalStates(ompl_start, ompl_stop);

  // Set the planner.
  // TODO: This is the only part that would need to change to make this
  // class a more general OMPL geometric planner wrapper.
  ob::SpaceInformationPtr ompl_space_info(new ob::SpaceInformation(ompl_space));
  ob::PlannerPtr ompl_planner(new og::RRTConnect(ompl_space_info));
  ompl_setup.setPlanner(ompl_planner);

  // Solve. Parameter is the amount of time (in seconds) used by the solver.
  ob::PlannerStatus solved = ompl_setup.solve(1.0);

  if (solved) {
    const og::PathGeometric& solution = ompl_setup.getSolutionPath();

    // Populate the Trajectory with states and time stamps.
    // TODO: Make sure this includes the start/stop states.
    Trajectory traj;
    double time = 0.0;
    for (size_t ii = 0; ii < solution.getStateCount(); ii++) {
      const VectorXd state =
        FromOmplState(solution.getState(ii), space.Dimension());

      // Catch first state.
      if (ii == 0)
        traj.Add(state, time);

      // Handle all other states.
      else {
        time += (state - traj.LastState()).norm() / velocity_;
        traj.Add(state, time);
      }
    }

    return traj;
  }

  ROS_WARN("RRT Connect could not compute a solution.");
  return Trajectory();
}

// Convert between OMPL states and VectorXds.
VectorXd RrtConnect::FromOmplState(const ob::State* state,
                                   size_t dimension) const {
#ifdef ENABLE_DEBUG_MESSAGES
  if (!state) {
    ROS_ERROR("State pointer was null.");
    return VectorXd::Zero(1);
  }
#endif

  const ob::RealVectorStateSpace::StateType* cast_state =
    static_cast<const ob::RealVectorStateSpace::StateType*>(state);

  VectorXd converted(dimension);
  for (size_t ii = 0; ii < converted.size(); ii++)
    converted(ii) = cast_state->values[ii];

  return converted;
}
