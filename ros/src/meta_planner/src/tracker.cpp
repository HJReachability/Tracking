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
// Defines the Tracker class.
//
///////////////////////////////////////////////////////////////////////////////

#include <meta_planner/tracker.h>

Tracker::Tracker()
  : initialized_(false) {}

Tracker::~Tracker() {}

// Initialize this class with all parameters and callbacks.
bool Tracker::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "meta_planner");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }
  initialized_ = true;

  return true;
}

// Load all parameters from config files.
bool Tracker::LoadParameters(const ros::NodeHandle& n) {
  std::string key;

  // Control update time step.
  if (!ros::param::search("meta_planner/control/time_step", key)) return false;
  if (!ros::param::get(key, time_step_)) return false;

  // Topics and frame ids.
  if (!ros::param::search("meta_planner/topics/control", key)) return false;
  if (!ros::param::get(key, control_topic_)) return false;

  if (!ros::param::search("meta_planner/topics/sensor", key)) return false;
  if (!ros::param::get(key, sensor_topic_)) return false;

  if (!ros::param::search("meta_planner/topics/rrt_connect", key)) return false;
  if (!ros::param::get(key, rrt_connect_vis_topic_)) return false;

  if (!ros::param::search("meta_planner/frames/fixed", key)) return false;
  if (!ros::param::get(key, fixed_frame_id_)) return false;

  if (!ros::param::search("meta_planner/frames/tracker", key)) return false;
  if (!ros::param::get(key, tracker_frame_id_)) return false;

  return true;
}

// Register all callbacks and publishers.
bool Tracker::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Sensor subscriber.
  // sensor_sub_ = nl.subscribe(sensor_topic_.c_str(), 10, &this->SensorCallback())

  // Visualization publisher(s).
  rrt_connect_vis_pub_ = nl.advertise<visualization_msgs::Marker>(
    rrt_connect_vis_topic_.c_str(), 10, false);

  control_pub_ = nl.advertise<geometry_msgs::Vector3>(
    control_topic_.c_str(), 10, false);

  // Timer.
  timer_ =
    nl.createTimer(ros::Duration(time_step_), &Tracker::TimerCallback, this);

  return true;
}


// Callback for processing sensor measurements.
// void Tracker::SensorCallback(const SomeMessageType::ConstPtr& msg);
//   Replan trajectory.

// Callback for applying tracking controller.
void Tracker::TimerCallback(const ros::TimerEvent& e) {
#if 0
  // 0) Get current TF.
  const VectorXd state = getCurrentState();
  // 1) Compute relative state.
  planner_state = ctraj_.Interpolate(current_time);
  rel_state = state - planner.lift(planner_state); 
  // 2) Get corresponding planner.
  planner = ctraj_.getPlanner(current_time);
  // 3) Interpolate gradient to get optimal control.
  opt_control = planner.getOptimalControl(rel_state);
  // 4) Apply optimal control.
  control_pub_.publish(opt_control);
#endif

}
