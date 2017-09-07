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
  : initialized_(false) {}

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

  // Set state space bounds.
  VectorXd state_upper_vec(state_dim_);
  VectorXd state_lower_vec(state_dim_);
  for (size_t ii = 0; ii < state_dim_; ii++) {
    state_upper_vec(ii) = state_upper_[ii];
    state_lower_vec(ii) = state_lower_[ii];
  }

  // Set the initial state and goal.
  // HACK! Assuming state layout.
  const size_t kXDim = 0;
  const size_t kYDim = 1;
  const size_t kZDim = 2;
  const size_t kVxDim = 3;
  const size_t kVyDim = 4;
  const size_t kVzDim = 5;

  const double kSmallNumber = 1.5;

  state_ = 0.5 * (state_lower_vec + state_upper_vec);
  state_(kVxDim) = 0.0;
  state_(kVyDim) = 0.0;
  state_(kVzDim) = 0.0;

  // Create value functions.
  if (numerical_mode_) {
    for (size_t ii = 0; ii < value_directories_.size(); ii++) {
      // NOTE: Assuming the 6D quadrotor model and geometric planner in 3D.
      // Load up the value function.
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
      const Vector3d max_tracker_control(control_upper_[0],
                                         control_upper_[1],
                                         control_upper_[2]);
      const Vector3d min_tracker_control(control_lower_[0],
                                         control_lower_[1],
                                         control_lower_[2]);
      const Vector3d max_velocity_disturbance =
        Vector3d::Constant(max_velocity_disturbances_[ii]);
      const Vector3d max_acceleration_disturbance =
        Vector3d::Constant(max_acceleration_disturbances_[ii]);

      // Create analytical value function.
      const AnalyticalPointMassValueFunction::ConstPtr value =
        AnalyticalPointMassValueFunction::Create(max_planner_speed,
                                                 max_tracker_control,
                                                 min_tracker_control,
                                                 max_velocity_disturbance,
                                                 max_acceleration_disturbance,
                                                 dynamics_,
                                                 static_cast<ValueFunctionId>(ii));

      values_.push_back(value);
    }
  }

  // Initialize trajectory to null as a cue to Hover(). Hover will reset the
  // trajectory to just hover in place.
  traj_ = nullptr;
  Hover();

  // Wait a little for the simulator to begin.
  ros::Duration(0.5).sleep();

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
  if (!nl.getParam("meta/meta/max_runtime", max_meta_runtime_)) return false;
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

  if (!nl.getParam("meta/state/upper", state_upper_)) return false;
  if (!nl.getParam("meta/state/lower", state_lower_)) return false;

  // Topics and frame ids.
  if (!nl.getParam("meta/topics/control", control_topic_)) return false;
  if (!nl.getParam("meta/topics/traj", traj_topic_)) return false;
  if (!nl.getParam("meta/topics/request_traj", request_traj_topic_)) return false;
  if (!nl.getParam("meta/topics/trigger_replan", trigger_replan_topic_)) return false;

  if (!nl.getParam("meta/topics/state", state_topic_)) return false;
  if (!nl.getParam("meta/topics/reference", reference_topic_)) return false;
  if (!nl.getParam("meta/topics/vis/traj", traj_vis_topic_)) return false;
  if (!nl.getParam("meta/topics/vis/tracking_bound", tracking_bound_topic_))
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
    state_topic_.c_str(), 10, &Tracker::StateCallback, this);

  traj_sub_ = nl.subscribe(
    traj_topic_.c_str(), 10, &Tracker::TrajectoryCallback, this);

  trigger_replan_sub_ = nl.subscribe(
    trigger_replan_topic_.c_str(), 10, &Tracker::TriggerReplanCallback, this);

  // Visualization publisher(s).
  environment_pub_ = nl.advertise<visualization_msgs::Marker>(
    environment_topic_.c_str(), 10, false);

  traj_vis_pub_ = nl.advertise<visualization_msgs::Marker>(
    traj_vis_topic_.c_str(), 10, false);

  tracking_bound_pub_ = nl.advertise<visualization_msgs::Marker>(
    tracking_bound_topic_.c_str(), 10, false);

  // Actual publishers.
  control_pub_ = nl.advertise<crazyflie_msgs::NoYawControlStamped>(
    control_topic_.c_str(), 10, false);

  reference_pub_ = nl.advertise<crazyflie_msgs::PositionStateStamped>(
    reference_topic_.c_str(), 10, false);

  request_traj_pub_ = nl.advertise<meta_planner_msgs::TrajectoryRequest>(
    request_traj_topic_.c_str(), 10, false);

  // Timer.
  timer_ =
    nl.createTimer(ros::Duration(time_step_), &Tracker::TimerCallback, this);

  return true;
}

// Callback for processing state updates.
void Tracker::StateCallback(const crazyflie_msgs::PositionStateStamped::ConstPtr& msg) {
  // HACK! Assuming state format.
  state_(0) = msg->state.x;
  state_(1) = msg->state.y;
  state_(2) = msg->state.z;
  state_(3) = msg->state.x_dot;
  state_(4) = msg->state.y_dot;
  state_(5) = msg->state.z_dot;
}

// Callback for processing trajectory updates.
void Tracker::TrajectoryCallback(const meta_planner_msgs::Trajectory::ConstPtr& msg) {
  traj_ = Trajectory::Create(msg, values_);
}

// Callback for when the MetaPlanner sees a new obstacle and wants the Tracker to
// hover and request a new trajectory.
void Tracker::TriggerReplanCallback(const std_msgs::Empty::ConstPtr& msg) {
  // Set trajectory to be the remainder of this trajectory, then hovering
  // at the end for a while.
  Hover();

  // Request a new trajectory starting from the state we will be in after
  // the max_meta_runtime_ has elapsed.
  RequestNewTrajectory();
}


// Callback for applying tracking controller.
void Tracker::TimerCallback(const ros::TimerEvent& e) {
  ros::Time current_time = ros::Time::now();

  // (1) If current time is near the end of the current trajectory, just hover and
  //     post a request for a new trajectory.
  if (current_time.toSec() > traj_->LastTime() - max_meta_runtime_) {
    ROS_WARN_THROTTLE(1.0, "%s: Nearing end of trajector. Replanning.",
             name_.c_str());

    // Set trajectory to be the remainder of this trajectory, then hovering
    // at the end for a while.
    Hover();

    // Request a new trajectory starting from the state we will be in after
    // the max_meta_runtime_ has elapsed.
    RequestNewTrajectory();

    return;
  }

  const VectorXd planner_state = traj_->GetState(current_time.toSec());
  const VectorXd relative_state = state_ - planner_state;

  const Vector3d planner_position = dynamics_->Puncture(planner_state);

  // Publish planner state on tf.
  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = fixed_frame_id_;
  transform_stamped.header.stamp = current_time;

  transform_stamped.child_frame_id = planner_frame_id_;

  transform_stamped.transform.translation.x = planner_position(0);
  transform_stamped.transform.translation.y = planner_position(1);
  transform_stamped.transform.translation.z = planner_position(2);

  transform_stamped.transform.rotation.x = 0;
  transform_stamped.transform.rotation.y = 0;
  transform_stamped.transform.rotation.z = 0;
  transform_stamped.transform.rotation.w = 1;

  br_.sendTransform(transform_stamped);

  // Publish planner position to the reference topic.
  // HACK! Assuming planner state order.
  crazyflie_msgs::PositionStateStamped reference;
  reference.header.stamp = current_time;

  reference.state.x = planner_position(0);
  reference.state.y = planner_position(1);
  reference.state.z = planner_position(2);

  reference.state.x_dot = planner_state(3);
  reference.state.y_dot = planner_state(4);
  reference.state.z_dot = planner_state(5);

  reference_pub_.publish(reference);

  // (2) Get corresponding value function.
  const ValueFunction::ConstPtr value = traj_->GetValueFunction(current_time.toSec());

  // Visualize the tracking bound.
  visualization_msgs::Marker tracking_bound_marker;
  tracking_bound_marker.ns = "bound";
  tracking_bound_marker.header.frame_id = planner_frame_id_;
  tracking_bound_marker.header.stamp = current_time;
  tracking_bound_marker.id = 0;
  tracking_bound_marker.type = visualization_msgs::Marker::CUBE;
  tracking_bound_marker.action = visualization_msgs::Marker::ADD;

  tracking_bound_marker.scale.x = 2.0 * value->TrackingBound(0);
  tracking_bound_marker.scale.y = 2.0 * value->TrackingBound(1);
  tracking_bound_marker.scale.z = 2.0 * value->TrackingBound(2);

  tracking_bound_marker.color.a = 0.3;
  tracking_bound_marker.color.r = 0.9;
  tracking_bound_marker.color.g = 0.2;
  tracking_bound_marker.color.b = 0.9;

  tracking_bound_pub_.publish(tracking_bound_marker);

  // Warn if outside tracking error bound.
  double min_dist_to_bound = std::numeric_limits<double>::infinity();
  for (size_t ii = 0; ii < 3; ii++) {
    const double signed_dist = value->TrackingBound(ii) -
      std::abs(relative_state(dynamics_->SpatialDimension(ii)));

    min_dist_to_bound = std::min(min_dist_to_bound, signed_dist);
  }

  if (min_dist_to_bound <= 0.0) {
    ROS_WARN_THROTTLE(1.0, "%s: Leaving the tracking error bound.", name_.c_str());
    //    std::terminate();
  }

  // (3) Interpolate gradient to get optimal control.
  const VectorXd optimal_control = value->OptimalControl(relative_state);

  // (4) Publish optimal control with priority in (0, 1).
  crazyflie_msgs::NoYawControlStamped control_msg;
  control_msg.header.stamp = ros::Time::now();

  // NOTE! Remember, control is assumed to be [pitch, roll, thrust].
  control_msg.control.pitch = crazyflie_utils::angles::WrapAngleRadians(optimal_control(0));
  control_msg.control.roll = crazyflie_utils::angles::WrapAngleRadians(optimal_control(1));
  control_msg.control.thrust = optimal_control(2);
  control_msg.control.priority = value->Priority(relative_state);

  control_pub_.publish(control_msg);

  // Visualize trajectory.
  traj_->Visualize(traj_vis_pub_, fixed_frame_id_, dynamics_);
}

// Request a new trajectory from the meta planner.
void Tracker::RequestNewTrajectory() const {
  ROS_INFO("%s: Requesting a new trajectory.", name_.c_str());

  // Determine time and state where we will receive the new trajectory.
  // This is when/where the new trajectory should start from.
  const double start_time = ros::Time::now().toSec() + max_meta_runtime_;
  const VectorXd start_state = traj_->GetState(start_time);

  // Populate request.
  meta_planner_msgs::TrajectoryRequest msg;
  msg.start_time = start_time;
  msg.start_state.dimension = state_dim_;

  for (size_t ii = 0; ii < start_state.size(); ii++)
    msg.start_state.state.push_back(start_state(ii));

  request_traj_pub_.publish(msg);
}

// Set the trajectory to continue the existing trajectory, then hover at the end.
// If no current trajectory exists, simply hover at the current state.
void Tracker::Hover() {
  ROS_INFO("%s: Setting a hover trajectory.", name_.c_str());

  // Get the current time.
  const double now = ros::Time::now().toSec();

  // Catch null trajectory, which should only occur on startup.
  // Hover at the current trajectory for max_meta_runtime_ + some small amount.
  // Set the value function for this trajectory to be the one for the least
  // aggressive planner, ie. for the smallest tracking bubble.
  if (traj_ == nullptr) {
    traj_ = Trajectory::Create();

    // Get a zero-velocity version of the current state.
    // HACK! Assuming state layout.
    VectorXd hover_state = state_;
    hover_state(3) = 0.0;
    hover_state(4) = 0.0;
    hover_state(5) = 0.0;

    traj_->Add(now, hover_state, values_.back());
    traj_->Add(now + max_meta_runtime_ + 1.0, hover_state, values_.back());
    return;
  }

  // Non-null current trajectory.
  // Copy over the remainder of the current trajectory.
  Trajectory::Ptr hover = Trajectory::Create(traj_, now);

  // Get the last state and make sure it has zero velocity.
  // HACK! Assuming state layout.
  VectorXd last_state = hover->LastState();
  last_state(3) = 0.0;
  last_state(4) = 0.0;
  last_state(5) = 0.0;

  // Hover at the last state.
  // HACK! Assuming can hover using last value function. Is this true?
  hover->Add(hover->LastTime() + max_meta_runtime_ + 1.0,
             last_state,
             hover->LastValueFunction());

  traj_ = hover;
}

} //\namespace meta
