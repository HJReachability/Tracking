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
  virtual Trajectory::Ptr Plan(const VectorXd& start,
                               const VectorXd& stop,
                               double start_time = 0.0) const = 0;

protected:
  explicit Planner(const ValueFunction::ConstPtr& value,
                   const Box::ConstPtr& space)
    : value_(value),
      space_(space) {}

  // Value function.
  const ValueFunction::ConstPtr value_;

  // State space (with collision checking).
  const Box::ConstPtr space_;
};

} //\namespace meta

#endif
