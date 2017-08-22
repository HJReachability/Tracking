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
// Defines the Simulator class. Holds a BallsInBox environment and sends
// simulated sensor measurements consisting of detected balls in range.
//
///////////////////////////////////////////////////////////////////////////////

#include <demo/simulator.h>
#include <random>

namespace meta {

Simulator::Simulator()
  : initialized_(false) {}

Simulator::~Simulator() {}

// Initialize this class with all parameters and callbacks.
bool Simulator::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "simulator");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  // Initialize state space.
  space_ = BallsInBox::Create(state_dim_);

  // Set state space bounds.
  VectorXd state_upper_vec(state_dim_);
  VectorXd state_lower_vec(state_dim_);
  for (size_t ii = 0; ii < state_dim_; ii++) {
    state_upper_vec(ii) = state_upper_[ii];
    state_lower_vec(ii) = state_lower_[ii];
  }

  space_->SetBounds(state_lower_vec, state_upper_vec);

  // Add obstacles.
  std::random_device rd;
  std::default_random_engine rng(rd());
  std::uniform_real_distribution<double> uniform_radius(0.5, 2.0);

  // Add an obstacle with a random radius at a random location.
  for (size_t ii = 0; ii < num_obstacles_; ii++)
    space_->AddObstacle(space_->Sample(), uniform_radius(rng));

  // Initialize current state and control.
  state_ = state_lower_vec;
  control_ = VectorXd::Zero(control_dim_);

  // Set the initial time.
  time_ = ros::Time::now();

  initialized_ = true;
  return true;
}

// Load all parameters from config files.
bool Simulator::LoadParameters(const ros::NodeHandle& n) {
  std::string key;

  // Sensor radius.
  if (!ros::param::search("meta/simulator/sensor_radius", key)) return false;
  if (!ros::param::get(key, sensor_radius_)) return false;

  // Number of obstacles.
  int num_obstacles = 1;
  if (!ros::param::search("meta/simulator/num_obstacles", key)) return false;
  if (!ros::param::get(key, num_obstacles)) return false;
  num_obstacles_ = static_cast<size_t>(num_obstacles);

  // Control parameters.
  if (!ros::param::search("meta/simulator/time_step", key)) return false;
  if (!ros::param::get(key, time_step_)) return false;

  int dimension = 1;
  if (!ros::param::search("meta/control/dim", key)) return false;
  if (!ros::param::get(key, dimension)) return false;
  control_dim_ = static_cast<size_t>(dimension);

  // State space parameters.
  if (!ros::param::search("meta/state/dim", key)) return false;
  if (!ros::param::get(key, dimension)) return false;
  state_dim_ = static_cast<size_t>(dimension);

  if (!ros::param::search("meta/state/upper", key)) return false;
  if (!ros::param::get(key, state_upper_)) return false;

  if (!ros::param::search("meta/state/lower", key)) return false;
  if (!ros::param::get(key, state_lower_)) return false;

  // Topics and frame ids.
  if (!ros::param::search("meta/topics/control", key)) return false;
  if (!ros::param::get(key, control_topic_)) return false;

  if (!ros::param::search("meta/topics/sensor", key)) return false;
  if (!ros::param::get(key, sensor_topic_)) return false;

  if (!ros::param::search("meta/topics/sensor_radius", key)) return false;
  if (!ros::param::get(key, sensor_radius_topic_)) return false;

  if (!ros::param::search("meta/topics/true_environment", key)) return false;
  if (!ros::param::get(key, environment_topic_)) return false;

  if (!ros::param::search("meta/frames/fixed", key)) return false;
  if (!ros::param::get(key, fixed_frame_id_)) return false;

  if (!ros::param::search("meta/frames/tracker", key)) return false;
  if (!ros::param::get(key, robot_frame_id_)) return false;

  // TODO! Load environment parameters.

  return true;
}

// Register all callbacks and publishers.
bool Simulator::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Subscriber.
  control_sub_ = nl.subscribe(
    control_topic_.c_str(), 10, &Simulator::ControlCallback, this);

  // Publishers.
  environment_pub_ = nl.advertise<visualization_msgs::Marker>(
    environment_topic_.c_str(), 10, false);

  sensor_radius_pub_ = nl.advertise<visualization_msgs::Marker>(
    sensor_radius_topic_.c_str(), 10, false);

  sensor_pub_ = nl.advertise<geometry_msgs::Quaternion>(
    sensor_topic_.c_str(), 10, false);

  // Timer.
  timer_ = nl.createTimer(
    ros::Duration(time_step_), &Simulator::TimerCallback, this);

  return true;
}


// Callback for processing control signals.
void Simulator::ControlCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
  control_(0) = msg->x;
  control_(1) = msg->y;
  control_(2) = msg->z;
}

// Timer callback for generating sensor measurements and updating
// state based on last received control signal.
void Simulator::TimerCallback(const ros::TimerEvent& e) {
  // Update state.
  const ros::Time now = ros::Time::now();
  const double dt = (now - time_).toSec();
  for (size_t ii = 0; ii < state_.size(); ii++)
    state_(ii) += control_(ii) * dt;

  time_ = now;

  // Broadcast tf.
  geometry_msgs::TransformStamped transform_stamped;

  transform_stamped.header.frame_id = fixed_frame_id_;
  transform_stamped.header.stamp = now;

  transform_stamped.child_frame_id = robot_frame_id_;

  transform_stamped.transform.translation.x = state_(0);
  transform_stamped.transform.translation.y = state_(1);
  transform_stamped.transform.translation.z = state_(2);

  transform_stamped.transform.rotation.x = 0;
  transform_stamped.transform.rotation.y = 0;
  transform_stamped.transform.rotation.z = 0;
  transform_stamped.transform.rotation.w = 1;

  br_.sendTransform(transform_stamped);

  // Publish sensor message if an obstacle is within range.
  // TODO! Parameterize sensor radius.
  VectorXd obstacle_position(3);
  double obstacle_radius = -1.0;

  if (space_->SenseObstacle(state_.head(3), sensor_radius_,
                            obstacle_position, obstacle_radius)) {
    geometry_msgs::Quaternion q;
    q.x = obstacle_position(0);
    q.y = obstacle_position(1);
    q.z = obstacle_position(2);
    q.w = obstacle_radius;

    sensor_pub_.publish(q);
  }

  // Visualize the environment.
  space_->Visualize(environment_pub_, fixed_frame_id_);

   // Visualize the sensor radius.
  visualization_msgs::Marker sensor_radius_marker;
  sensor_radius_marker.ns = "sensor";
  sensor_radius_marker.header.frame_id = robot_frame_id_;
  sensor_radius_marker.header.stamp = now;
  sensor_radius_marker.id = 0;
  sensor_radius_marker.type = visualization_msgs::Marker::SPHERE;
  sensor_radius_marker.action = visualization_msgs::Marker::ADD;

  sensor_radius_marker.scale.x = 2.0 * sensor_radius_;
  sensor_radius_marker.scale.y = 2.0 * sensor_radius_;
  sensor_radius_marker.scale.z = 2.0 * sensor_radius_;

  sensor_radius_marker.color.a = 0.2;
  sensor_radius_marker.color.r = 0.8;
  sensor_radius_marker.color.g = 0.0;
  sensor_radius_marker.color.b = 0.2;

  sensor_radius_pub_.publish(sensor_radius_marker);
}

} //\namespace meta
