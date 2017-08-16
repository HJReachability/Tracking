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
// Defines the Environment abstract class interface.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef META_PLANNER_ENVIRONMENT_H
#define META_PLANNER_ENVIRONMENT_H

#include <meta_planner/types.h>
#include <meta_planner/uncopyable.h>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <random>
#include <string>

class Environment : private Uncopyable {
public:
  virtual ~Environment() {}

  // Derived classes must be able to sample uniformly from the state space.
  virtual VectorXd Sample() const = 0;

  // Derived classes must provide a collision checker which returns true if
  // and only if the provided state is a valid collision-free configuration.
  virtual bool IsValid(const VectorXd& state, double tracking_bound) const = 0;

  // Derived classes must have some sort of visualization through RVIZ.
  virtual void Visualize(const ros::Publisher& pub,
                         const std::string& frame_id) const = 0;

protected:
  explicit Environment()
    : rd_(), rng_(rd_()) {}

  // Random number generation.
  std::random_device rd_;
  mutable std::default_random_engine rng_;
};

#endif
