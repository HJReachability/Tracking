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
// Defines the WaypointTree class. The WaypointTree class handles queries like
// finding the nearest k points, as well as the length (in time) of the
// shortest path to the goal.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef META_PLANNER_WAYPOINT_TREE_H
#define META_PLANNER_WAYPOINT_TREE_H

#include <meta_planner/waypoint.h>
#include <meta_planner/flann_tree.h>
#include <meta_planner/types.h>
#include <meta_planner/uncopyable.h>

#include <list>

class WaypointTree : private Uncopyable {
public:
  explicit WaypointTree(const VectorXd& start, const VectorXd& stop);
  ~WaypointTree() {}

  // Find nearest neighbors in the tree.
  inline std::vector<Waypoint::ConstPtr>
  KnnSearch(VectorXd& query, size_t k) const {
    return kdtree_.KnnSearch(query, k);
  }

  inline std::vector<Waypoint::ConstPtr>
  RadiusSearch(VectorXd& query, double r) const {
    return kdtree_.RadiusSearch(query, r);
  }

  // Add Waypoint to tree.
  void Insert(const Waypoint::ConstPtr& waypoint, bool is_terminal);

  // Get best (fastest) trajectory (if it exists).
  Trajectory::Ptr BestTrajectory() const;

private:
  // Root of the tree.
  const Waypoint::ConstPtr root_;

  // Best terminal waypoint.
  Waypoint::ConstPtr terminus_;

  // Kdtree storing all waypoints for easy nearest neighbor searching.
  FlannTree kdtree_;
};

#endif
