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
// Defines the Trajectory struct.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef META_PLANNER_TRAJECTORY_H
#define META_PLANNER_TRAJECTORY_H

#include <meta_planner/types.h>

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <vector>

class Trajectory {
public:
  // Clear out this Trajectory.
  inline void Clear() {
    states_.clear();
    times_.clear();
  }

  // Add a (point, time) tuple to this Trajectory.
  inline void Add(const VectorXd& point, double time) {
    states_.push_back(point);
    times_.push_back(time);
  }

  // Check if this trajectory is empty.
  inline bool IsEmpty() const {
    return states_.size() == 0;
  }

  // Number of waypoints.
  inline size_t Size() const {
    return states_.size();
  }

  // Total time length of the trajectory.
  inline double Time() const {
    return times_.back() - times_.front();
  }

  // Accessors.
  inline const VectorXd& LastState() const { return states_.back(); }
  inline const VectorXd& FirstState() const { return states_.front(); }
  inline double LastTime() const { return times_.back(); }
  inline double FirstTime() const { return times_.front(); }

  // Visualize this trajectory in RVIZ.
  void Visualize(const ros::Publisher& pub, const std::string& frame_id) const;

private:
  // Compute the color (on a red-blue colormap) at a particular index.
  std_msgs::ColorRGBA Colormap(size_t index) const;

  // List of states and corresponding times.
  std::vector<VectorXd> states_;
  std::vector<double> times_;
};

#endif
