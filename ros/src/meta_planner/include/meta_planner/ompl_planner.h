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
// Defines the OmplPlanner class, which wraps the OMPL geometric planners
// and inherits from the Planner abstract class. For simplicity, we assume that
// the state space is a real-valued vector space with box constraints, i.e.
// an instance of the Box subclass of Environment.
//
// We follow these ( http://ompl.kavrakilab.org/geometricPlanningSE3.html )
// instructions for using OMPL geometric planners.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef META_PLANNER_OMPL_PLANNER_H
#define META_PLANNER_OMPL_PLANNER_H

#include <meta_planner/planner.h>
#include <meta_planner/box.h>
#include <meta_planner/types.h>

#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/TypedSpaceInformation.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <memory>

namespace ob = ompl::base;
namespace og = ompl::geometric;

template<typename PlannerType>
class OmplPlanner : public Planner {
public:
  ~OmplPlanner() {}

  static Planner::ConstPtr Create(const ValueFunction::ConstPtr& value,
                       const Box::ConstPtr& space,
                       const std::vector<size_t>& dimensions,
                       double speed);


  // Derived classes must plan trajectories between two points.
  Trajectory::Ptr Plan(
    const VectorXd& start, const VectorXd& stop, double start_time = 0.0) const;

private:
  explicit OmplPlanner(const ValueFunction::ConstPtr& value,
                       const Box::ConstPtr& space,
                       const std::vector<size_t>& dimensions,
                       double speed);

  // Convert between OMPL states and VectorXds.
  VectorXd FromOmplState(const ob::State* state) const;

  // Robot speed.
  const double speed_;
};

// ------------------------------- IMPLEMENTATION --------------------------- //

template<typename PlannerType>
OmplPlanner<PlannerType>::OmplPlanner(const ValueFunction::ConstPtr& value,
                                      const Box::ConstPtr& space,
                                      const std::vector<size_t>& dimensions,
                                      double speed)
  : Planner(value, space, dimensions),
    speed_((speed <= 0.0) ? 1.0 : speed) {
#ifdef ENABLE_DEBUG_MESSAGES
  if (speed_ <= 0.0)
    ROS_ERROR("Speed was negative. Setting to unity.");
#endif
}


// Create OmplPlanner pointer.
template<typename PlannerType>
inline Planner::ConstPtr OmplPlanner<PlannerType>::
Create(const ValueFunction::ConstPtr& value,
       const Box::ConstPtr& space,
       const std::vector<size_t>& dimensions,
       double speed){
 Planner::ConstPtr ptr(new OmplPlanner<PlannerType>(value, space, dimensions, speed));
 return ptr;
}



// Derived classes must plan trajectories between two points.
template<typename PlannerType>
Trajectory::Ptr OmplPlanner<PlannerType>::Plan(
  const VectorXd& start, const VectorXd& stop, double start_time) const {
#ifdef ENABLE_DEBUG_MESSAGES
  if (start.size() != stop.size() || start.size() != space_->Dimension()) {
    ROS_ERROR("Start/stop state dimensions inconsistent with space dimension.");
    return nullptr;
  }
#endif

  // Create the OMPL state space corresponding to this environment.
  auto ompl_space(
    std::make_shared<ob::RealVectorStateSpace>(dimensions_.size()));

  // Set bounds for the environment.
  const VectorXd& lower = space_->LowerBounds(dimensions_);
  const VectorXd& upper = space_->UpperBounds(dimensions_);
  ob::RealVectorBounds ompl_bounds(dimensions_.size());

  for (size_t ii = 0; ii < dimensions_.size(); ii++) {
    ompl_bounds.setLow(ii, lower(ii));
    ompl_bounds.setHigh(ii, upper(ii));
  }

  ompl_space->setBounds(ompl_bounds);

  // Create a SimpleSetup instance and set the state validity checker function.
  og::SimpleSetup ompl_setup(ompl_space);
  ompl_setup.setStateValidityChecker([&](const ob::State* state) {
      return space_->IsValid(FromOmplState(state), value_->TrackingBound()); });

  // Set the start and stop states.
  ob::ScopedState<ob::RealVectorStateSpace> ompl_start(ompl_space);
  ob::ScopedState<ob::RealVectorStateSpace> ompl_stop(ompl_space);
  for (size_t ii = 0; ii < dimensions_.size(); ii++) {
    ompl_start[ii] = start(dimensions_[ii]);
    ompl_stop[ii] = stop(dimensions_[ii]);
  }

  ompl_setup.setStartAndGoalStates(ompl_start, ompl_stop);

  // Set the planner.
  ob::PlannerPtr ompl_planner(
    new PlannerType(ompl_setup.getSpaceInformation()));
  ompl_setup.setPlanner(ompl_planner);

  // Solve. Parameter is the amount of time (in seconds) used by the solver.
  const ob::PlannerStatus solved = ompl_setup.solve(1.0);

  if (solved) {
    const og::PathGeometric& solution = ompl_setup.getSolutionPath();

    // Populate the Trajectory with states and time stamps.
    Trajectory::Ptr traj = Trajectory::Create();
    double time = start_time;
    for (size_t ii = 0; ii < solution.getStateCount(); ii++) {
      const VectorXd state = FromOmplState(solution.getState(ii));

      // Catch first state.
      if (ii == 0)
        traj->Add(time, state, value_);

      // Handle all other states.
      // Assuming speed is isotropic.
      // TODO: make this more general.
      else {
        time += (state - traj->LastState()).norm() / speed_;
        traj->Add(time, state, value_);
      }
    }

    return traj;
  }

  ROS_WARN("OMPL Planner could not compute a solution.");
  return nullptr;
}

// Convert between OMPL states and VectorXds.
template<typename PlannerType>
VectorXd OmplPlanner<PlannerType>::FromOmplState(
  const ob::State* state) const {
#ifdef ENABLE_DEBUG_MESSAGES
  if (!state) {
    ROS_ERROR("State pointer was null.");
    return VectorXd::Zero(space_->Dimension());
  }
#endif

  const ob::RealVectorStateSpace::StateType* cast_state =
    static_cast<const ob::RealVectorStateSpace::StateType*>(state);

  VectorXd converted = VectorXd::Zero(space_->Dimension());
  for (size_t ii = 0; ii < dimensions_.size(); ii++)
    converted(dimensions_[ii]) = cast_state->values[ii];

  return converted;
}


#endif
