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
#include <crazyflie_utils/angles.h>

#include <stdlib.h>

namespace meta {

Tracker::Tracker()
  : in_flight_(false),
    been_updated_(false),
    initialized_(false) {}

Tracker::~Tracker() {}

// Initialize this class with all parameters and callbacks.
bool Tracker::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "tracker");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  // Set control upper/lower bounds as Eigen::Vectors.
  VectorXd control_upper_vec(control_dim_);
  VectorXd control_lower_vec(control_dim_);
  for (size_t ii = 0; ii < control_dim_; ii++) {
    control_upper_vec(ii) = control_upper_[ii];
    control_lower_vec(ii) = control_lower_[ii];
  }

  // Set up dynamics.
  dynamics_ = NearHoverQuadNoYaw::Create(control_lower_vec, control_upper_vec);

  // Set the initial state and reference to zero.
  state_ = VectorXd::Zero(state_dim_);
  reference_ = VectorXd::Zero(state_dim_);

  // Populate list of value functions.
  if (numerical_mode_) {
    for (size_t ii = 0; ii < value_directories_.size(); ii++) {
      const ValueFunction::ConstPtr value =
        ValueFunction::Create(value_directories_[ii], dynamics_,
                              state_dim_, control_dim_,
                              static_cast<ValueFunctionId>(ii));

      values_.push_back(value);
    }
  } else {
    for (size_t ii = 0; ii < max_planner_speeds_.size(); ii++) {
      // Generate inputs for AnalyticalPointMassValueFunction.
      // SEMI-HACK! Manually feeding control/disturbance bounds.
      const Vector3d max_planner_speed =
        Vector3d::Constant(max_planner_speeds_[ii]);
      const Vector3d max_velocity_disturbance =
        Vector3d::Constant(max_velocity_disturbances_[ii]);
      const Vector3d max_acceleration_disturbance =
        Vector3d::Constant(max_acceleration_disturbances_[ii]);
      const Vector3d velocity_expansion = Vector3d::Constant(0.1);

      // Create analytical value function.
      const AnalyticalPointMassValueFunction::ConstPtr value =
        AnalyticalPointMassValueFunction::Create(max_planner_speed,
                                                 max_velocity_disturbance,
                                                 max_acceleration_disturbance,
                                                 velocity_expansion,
                                                 dynamics_,
                                                 static_cast<ValueFunctionId>(ii));

      values_.push_back(value);
    }
  }

  if (values_.size() % 2 != 0) {
    ROS_ERROR("%s: Must provide pairs of value functions.", name_.c_str());
    return false;
  }

  // Start control and bound values at most/least conservative.
  control_value_ = values_.back();
  bound_value_ = values_.front();

  // Wait a little for the simulator to begin.
  //  ros::Duration(0.5).sleep();

  initialized_ = true;
  return true;
}

// Load all parameters from config files.
bool Tracker::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Control parameters.
  if (!nl.getParam("meta/control/time_step", time_step_)) return false;

  int dimension = 1;
  if (!nl.getParam("meta/control/dim", dimension)) return false;
  control_dim_ = static_cast<size_t>(dimension);

  if (!nl.getParam("meta/control/upper", control_upper_)) return false;
  if (!nl.getParam("meta/control/lower", control_lower_)) return false;

  if (control_upper_.size() != control_dim_ ||
      control_lower_.size() != control_dim_) {
    ROS_ERROR("%s: Upper and/or lower bounds are in the wrong dimension.",
              name_.c_str());
    return false;
  }

  // Planner parameters.
  if (!nl.getParam("meta/planners/numerical_mode", numerical_mode_)) return false;
  if (!nl.getParam("meta/planners/value_directories", value_directories_))
    return false;

  if (value_directories_.size() == 0) {
    ROS_ERROR("%s: Must specify at least one value function directory.",
              name_.c_str());
    return false;
  }

  if (!nl.getParam("meta/planners/max_speeds", max_planner_speeds_)) return false;
  if (!nl.getParam("meta/planners/max_velocity_disturbances",
                   max_velocity_disturbances_))
    return false;
  if (!nl.getParam("meta/planners/max_acceleration_disturbances",
                   max_acceleration_disturbances_))
    return false;

  if (max_planner_speeds_.size() != max_velocity_disturbances_.size() ||
      max_planner_speeds_.size() != max_acceleration_disturbances_.size()) {
    ROS_ERROR("%s: Must specify max speed/velocity/acceleration disturbances.",
              name_.c_str());
    return false;
  }

  // State space parameters.
  if (!nl.getParam("meta/state/dim", dimension)) return false;
  state_dim_ = static_cast<size_t>(dimension);

  // Topics and frame ids.
  if (!nl.getParam("meta/topics/control", control_topic_)) return false;
  if (!nl.getParam("meta/topics/in_flight", in_flight_topic_)) return false;
  if (!nl.getParam("meta/topics/state", state_topic_)) return false;
  if (!nl.getParam("meta/topics/reference", reference_topic_)) return false;
  if (!nl.getParam("meta/topics/controller_id", controller_id_topic_))
    return false;

  if (!nl.getParam("meta/frames/fixed", fixed_frame_id_)) return false;
  if (!nl.getParam("meta/frames/tracker", tracker_frame_id_)) return false;
  if (!nl.getParam("meta/frames/planner", planner_frame_id_)) return false;

  return true;
}

// Register all callbacks and publishers.
bool Tracker::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Subscribers.
  state_sub_ = nl.subscribe(
    state_topic_.c_str(), 1, &Tracker::StateCallback, this);

  reference_sub_ = nl.subscribe(
    reference_topic_.c_str(), 1, &Tracker::ReferenceCallback, this);

  controller_id_sub_ = nl.subscribe(
    controller_id_topic_.c_str(), 1, &Tracker::ControllerIdCallback, this);

  in_flight_sub_ = nl.subscribe(
    in_flight_topic_.c_str(), 1, &Tracker::InFlightCallback, this);

  // Actual publishers.
  control_pub_ = nl.advertise<crazyflie_msgs::NoYawControlStamped>(
    control_topic_.c_str(), 1, false);

  // Timer.
  timer_ =
    nl.createTimer(ros::Duration(time_step_), &Tracker::TimerCallback, this);

  return true;
}

// Callback for processing state updates.
void Tracker::
StateCallback(const crazyflie_msgs::PositionStateStamped::ConstPtr& msg) {
  // HACK! Assuming state format.
  state_(0) = msg->state.x;
  state_(1) = msg->state.y;
  state_(2) = msg->state.z;
  state_(3) = msg->state.x_dot;
  state_(4) = msg->state.y_dot;
  state_(5) = msg->state.z_dot;

  been_updated_ = true;
}

// Callback for processing state updates.
void Tracker::ReferenceCallback(
  const crazyflie_msgs::PositionStateStamped::ConstPtr& msg) {
  // HACK! Assuming state format.
  reference_(0) = msg->state.x;
  reference_(1) = msg->state.y;
  reference_(2) = msg->state.z;
  reference_(3) = msg->state.x_dot;
  reference_(4) = msg->state.y_dot;
  reference_(5) = msg->state.z_dot;
}

// Callback for processing state updates.
void Tracker::ControllerIdCallback(
  const meta_planner_msgs::ControllerId::ConstPtr& msg) {
  control_value_ = values_[msg->control_value_function_id];
  bound_value_ = values_[msg->bound_value_function_id];
}

// Callback for applying tracking controller.
void Tracker::TimerCallback(const ros::TimerEvent& e) {
  if (!in_flight_ || !been_updated_)
    return;

  const VectorXd relative_state = state_ - reference_;
  const Vector3d planner_position = dynamics_->Puncture(reference_);

  // (1) Get corresponding control and bound value function.
  const double priority = control_value_->Priority(relative_state);

  // (2) Interpolate gradient to get optimal control.
  const VectorXd optimal_control =
    control_value_->OptimalControl(relative_state);

  // (3) Publish optimal control with priority in (0, 1).
  crazyflie_msgs::NoYawControlStamped control_msg;
  control_msg.header.stamp = ros::Time::now();

  // NOTE! Remember, control is assumed to be [pitch, roll, thrust].
  control_msg.control.pitch =
    crazyflie_utils::angles::WrapAngleRadians(optimal_control(0));
  control_msg.control.roll =
    crazyflie_utils::angles::WrapAngleRadians(optimal_control(1));
  control_msg.control.thrust = optimal_control(2);
  control_msg.control.priority = priority;

  control_pub_.publish(control_msg);
}

} //\namespace meta
