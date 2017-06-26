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

#ifndef META_PLANNER_RRT_CONNECT_H
#define META_PLANNER_RRT_CONNECT_H

#include <meta_planner/planner.h>
#include <meta_planner/box.h>

#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/TypedSpaceInformation.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <memory>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class RrtConnect : public Planner {
public:
  ~RrtConnect() {}
  explicit RrtConnect(double velocity);

  // Derived classes must plan trajectories between two points.
  Trajectory Plan(const VectorXd& start, const VectorXd& stop, const Box& space) const;

private:
  // Convert between OMPL states and VectorXds.
  VectorXd FromOmplState(const ob::State* state, size_t dimension) const;

  // Robot velocity.
  const double velocity_;
};

#endif
