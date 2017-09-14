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
// Defines the Planner abstract class interface. For now, all Planners must
// operate within a Box. This is because of the way in which subspaces are
// specified in the constructor.
//
// Planners take in two different ValueFunctions: one for the waypoint prior
// to this plan, and one for the waypoint at the end of this plan.
// These ValueFunctions should be set such that the outgoing ID is precisely
// 1 + the incoming ID. In the numerical case, value functions will be ordered:
// (1), (1 -> 2), (2 -> 3), ... so that planner 1 gets values (1) and (1 -> 2),
// planner 2 gets values (1 -> 2) and (2 -> 3), etc. In the analytic case,
// value functions will be ordered (1), (2), (3), ... so that planner 1 gets
// values (1) and (2), planner 2 gets values (2) and (3), etc. This way,
// collision checking may be done using the SwitchingTrackingBound() function
// provided by the outgoing value function for both numeric/analytic cases.
//
// In short, the outgoing value function is what is typically meant by _the_
// value function associated with this Planner. The incoming value function
// is only used for generating switching tracking bounds.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef META_PLANNER_PLANNER_H
#define META_PLANNER_PLANNER_H

#include <meta_planner/value_function.h>
#include <meta_planner/trajectory.h>
#include <meta_planner/environment.h>
#include <meta_planner/box.h>
#include <meta_planner/types.h>
#include <meta_planner/uncopyable.h>

#include <memory>

#include <ros/ros.h>

namespace meta {

class Planner : private Uncopyable {
public:
  typedef std::shared_ptr<const Planner> ConstPtr;

  // Destructor.
  virtual ~Planner() {}

  // Derived classes must plan trajectories between two points.
  // Budget is the time the planner is allowed to take during planning.
  virtual Trajectory::Ptr Plan(const Vector3d& start,
                               const Vector3d& stop,
                               double start_time = 0.0,
                               double budget = 1.0) const = 0;

  // Shortest possible time to go from start to stop for this planner.
  double BestPossibleTime(const Vector3d& start, const Vector3d& stop) const;

  // Get the value function associated to this planner. The way incoming and
  // outgoing value functions are intended to be used, this corresponds to
  // the outgoing value function.
  inline ValueFunction::ConstPtr GetIncomingValueFunction() const {
    return incoming_value_;
  }

  inline ValueFunction::ConstPtr GetOutgoingValueFunction() const {
    return outgoing_value_;
  }

protected:
  explicit Planner(const ValueFunction::ConstPtr& incoming_value,
                   const ValueFunction::ConstPtr& outgoing_value,
                   const Box::ConstPtr& space)
    : incoming_value_(incoming_value),
      outgoing_value_(outgoing_value),
      space_(space) {
    if (incoming_value_->Id() + 1!= outgoing_value_->Id())
      ROS_ERROR("Outgoing value function not successor to incoming one.");
  }

  // Value functions.
  const ValueFunction::ConstPtr incoming_value_;
  const ValueFunction::ConstPtr outgoing_value_;

  // State space (with collision checking).
  const Box::ConstPtr space_;
};

} //\namespace meta

#endif
