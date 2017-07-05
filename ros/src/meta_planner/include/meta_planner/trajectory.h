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
#include <map>
#include <string>
#include <iostream>
#include <exception>

class Trajectory {
public:
  // Clear out this Trajectory.
  void Clear();

  // Add a (state, time) tuple to this Trajectory.
  void Add(const VectorXd& state, double time);

  // Check if this trajectory is empty.
  bool IsEmpty() const;

  // Number of waypoints.
  size_t Size() const;

  // Total time length of the trajectory.
  double Time() const;

  // Accessors.
  const VectorXd& LastState() const;
  const VectorXd& FirstState() const;
  double LastTime() const;
  double FirstTime() const;

  // Find the state corresponding to a particular time via linear interpolation.
  VectorXd Interpolate(double time) const;

  // Visualize this trajectory in RVIZ.
  void Visualize(const ros::Publisher& pub, const std::string& frame_id) const;

  // Print this trajectory to stdout.
  void Print(const std::string& prefix) const;

private:
  // Compute the color (on a red-blue colormap) at a particular time.
  std_msgs::ColorRGBA Colormap(double time) const;

  // Map from time stamp to corresponding state.
  std::map<double, VectorXd> map_;
};

// ---------------------- IMPLEMENT INLINE FUNCTIONS ------------------------ //

// Clear out this Trajectory.
inline void Trajectory::Clear() {
  map_.clear();
}

// Add a (state, time) tuple to this Trajectory.
inline void Trajectory::Add(const VectorXd& state, double time) {
  map_.insert({time, state});
}

// Check if this trajectory is empty.
inline bool Trajectory::IsEmpty() const {
  return map_.empty();
}

// Number of waypoints.
inline size_t Trajectory::Size() const {
  return map_.size();
}

// Total time length of the trajectory.
inline double Trajectory::Time() const {
#ifdef ENABLE_DEBUG_MESSAGES
  if (IsEmpty()) {
    ROS_WARN("Tried to get time length of empty trajectory.");
    return 0.0;
  }
#endif

  return LastTime() - FirstTime();
}

// Accessors.
inline const VectorXd& Trajectory::LastState() const {
#ifdef ENABLE_DEBUG_MESSAGES
  if (IsEmpty()) {
    ROS_WARN("Tried to get last state of empty trajectory.");
    throw std::underflow_error("Attempted last state of empty trajectory.");
  }
#endif

  return (--map_.end())->second;
}

inline const VectorXd& Trajectory::FirstState() const {
#ifdef ENABLE_DEBUG_MESSAGES
  if (IsEmpty()) {
    ROS_WARN("Tried to get first state of empty trajectory.");
    throw std::underflow_error("Attempted first state of empty trajectory.");
  }
#endif

  return map_.begin()->second;
}

inline double Trajectory::LastTime() const {
#ifdef ENABLE_DEBUG_MESSAGES
  if (IsEmpty()) {
    ROS_WARN("Tried to get last time of empty trajectory.");
    throw std::underflow_error("Attempted last time of empty trajectory.");
  }
#endif

  return (--map_.end())->first;
}

inline double Trajectory::FirstTime() const {
#ifdef ENABLE_DEBUG_MESSAGES
  if (IsEmpty()) {
    ROS_WARN("Tried to get first time of empty trajectory.");
    throw std::underflow_error("Attempted first time of empty trajectory.");
  }
#endif

  return map_.begin()->first;
}


#endif
