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
// Defines the MetaPlanner class. The MetaPlanner samples random points in
// the state space and then spawns off different Planners to plan Trajectories
// between these points (RRT-style).
//
///////////////////////////////////////////////////////////////////////////////

#include <meta_planner/meta_planner.h>

namespace meta {

// Plan a trajectory using the given (ordered) list of Planners.
// (1) Set up a new RRT-like structure to hold the meta plan.
// (2) Sample a new point in the state space.
// (3) Find nearest neighbor.
// (4) Plan a trajectory (starting with most aggressive planner).
// (5) Try to connect to the goal point.
// (6) Stop when we have a feasible trajectory. Otherwise go to (2).
Trajectory::Ptr MetaPlanner::Plan(
  const Vector3d& start, const Vector3d& stop,
  const std::vector<Planner::ConstPtr>& planners) const {
  // (1) Set up a new RRT-like structure to hold the meta plan.
  const ros::Time start_time = ros::Time::now();
  WaypointTree tree(start, stop, start_time.toSec());

  bool done = false;
  while (!done && (ros::Time::now() - start_time).toSec() < 1.0) {
    // (2) Sample a new point in the state space.
    Vector3d sample = space_->Sample();

    // (3) Find the nearest neighbor.
    const size_t kNumNeighbors = 1;
    const std::vector<Waypoint::ConstPtr> neighbors =
      tree.KnnSearch(sample, kNumNeighbors);

    if (neighbors.size() == 0 ||
        (neighbors[0]->point_ - sample).norm() > max_connection_radius_)
      continue;

    // (4) Plan a trajectory (starting with most aggressive planner).
    Trajectory::Ptr traj;
    for (const auto& planner : planners) {
      traj = planner->Plan(neighbors[0]->point_, sample, neighbors[0]->time_);

      if (traj != nullptr)
        break;
    }

    if (traj == nullptr)
      continue;

    // (5) Try to connect to the goal point.
    Trajectory::Ptr goal_traj;
    if ((sample - stop).norm() <= max_connection_radius_) {
      for (const auto& planner : planners) {
        goal_traj = planner->Plan(sample, stop, traj->LastTime());

        if (goal_traj != nullptr)
          break;
      }
    }

    // (6) Stop when we have a feasible trajectory. Otherwise go to (2).
    const Waypoint::ConstPtr waypoint = Waypoint::Create(
      sample, traj->LastTime(), traj, neighbors[0]);

    tree.Insert(waypoint, false);

    if (goal_traj != nullptr) {
      // Connect to the goal. NOTE: the first point in goal_traj coincides with
      // the last point in traj, but when we merge the two trajectories the
      // std::map insertion rules will prevent duplicates.
      const Waypoint::ConstPtr goal = Waypoint::Create(
        stop, goal_traj->LastTime(), goal_traj, waypoint);

      tree.Insert(goal, true);

      // TODO: in future we could have some other stopping criteria.
      done = true;
    }
  }

  if (done) {
    // Get the best (fastest) trajectory out of the tree.
    const Trajectory::Ptr best = tree.BestTrajectory();
    return best;
  }

  ROS_ERROR("Meta planner failed.");
  return nullptr;
}

} //\namespace meta
