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

#include <demo/sensor.h>
#include <random>

namespace meta {

Sensor::Sensor()
  : tf_listener_(tf_buffer_),
    initialized_(false) {}

Sensor::~Sensor() {}

// Initialize this class with all parameters and callbacks.
bool Sensor::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "sensor");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  // Initialize state space.
  space_ = BallsInBox::Create();

  // Dynamics with dummy control bounds. We only need puncturing functionality.
  dynamics_ = NearHoverQuadNoYaw::Create(VectorXd::Zero(control_dim_),
                                         VectorXd::Zero(control_dim_));

  // Set state space bounds.
  VectorXd state_upper_vec(state_dim_);
  VectorXd state_lower_vec(state_dim_);
  for (size_t ii = 0; ii < state_dim_; ii++) {
    state_upper_vec(ii) = state_upper_[ii];
    state_lower_vec(ii) = state_lower_[ii];
  }

  space_->SetBounds(dynamics_->Puncture(state_lower_vec),
                    dynamics_->Puncture(state_upper_vec));

  // Add obstacles.
  std::random_device rd;
  std::default_random_engine rng(rd());
  std::uniform_real_distribution<double> uniform_radius(0.5, 2.0);

  // Add an obstacle with a random radius at a random location.
  for (size_t ii = 0; ii < num_obstacles_; ii++)
    space_->AddObstacle(space_->Sample(), uniform_radius(rng));

  initialized_ = true;
  return true;
}

// Load all parameters from config files.
bool Sensor::LoadParameters(const ros::NodeHandle& n) {
  std::string key;

  // Sensor radius.
  if (!ros::param::search("meta/sensor/sensor_radius", key)) return false;
  if (!ros::param::get(key, sensor_radius_)) return false;

  // Number of obstacles.
  int num_obstacles = 1;
  if (!ros::param::search("meta/sensor/num_obstacles", key)) return false;
  if (!ros::param::get(key, num_obstacles)) return false;
  num_obstacles_ = static_cast<size_t>(num_obstacles);

  // Time step.
  if (!ros::param::search("meta/sensor/time_step", key)) return false;
  if (!ros::param::get(key, time_step_)) return false;

  // State space parameters.
  int dimension = 1;
  if (!ros::param::search("meta/control/dim", key)) return false;
  if (!ros::param::get(key, dimension)) return false;
  control_dim_ = static_cast<size_t>(dimension);

  if (!ros::param::search("meta/state/dim", key)) return false;
  if (!ros::param::get(key, dimension)) return false;
  state_dim_ = static_cast<size_t>(dimension);

  if (!ros::param::search("meta/state/upper", key)) return false;
  if (!ros::param::get(key, state_upper_)) return false;

  if (!ros::param::search("meta/state/lower", key)) return false;
  if (!ros::param::get(key, state_lower_)) return false;

  // Topics and frame ids.
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
bool Sensor::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

   // Publishers.
  environment_pub_ = nl.advertise<visualization_msgs::Marker>(
    environment_topic_.c_str(), 10, false);

  sensor_radius_pub_ = nl.advertise<visualization_msgs::Marker>(
    sensor_radius_topic_.c_str(), 10, false);

  sensor_pub_ = nl.advertise<geometry_msgs::Quaternion>(
    sensor_topic_.c_str(), 10, false);

  // Timer.
  timer_ = nl.createTimer(
    ros::Duration(time_step_), &Sensor::TimerCallback, this);

  return true;
}

// Timer callback for generating sensor measurements and updating
// state based on last received control signal.
void Sensor::TimerCallback(const ros::TimerEvent& e) {
  // Read position from TF.
  const ros::Time right_now = ros::Time::now();

  // Get the current transform from tf.
  geometry_msgs::TransformStamped tf;

  try {
    tf = tf_buffer_.lookupTransform(
      fixed_frame_id_.c_str(), robot_frame_id_.c_str(), right_now);
  } catch(tf2::TransformException &ex) {
    ROS_WARN("%s: %s", name_.c_str(), ex.what());
    ROS_WARN("%s: Could not determine current state.", name_.c_str());
    return;
  }

  // Extract translation.
  const Vector3d position(tf.transform.translation.x,
                          tf.transform.translation.y,
                          tf.transform.translation.z);

  // Publish sensor message if an obstacle is within range.
  // TODO! Parameterize sensor radius.
  Vector3d obstacle_position;
  double obstacle_radius = -1.0;

  if (space_->SenseObstacle(position, sensor_radius_,
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
  sensor_radius_marker.header.stamp = right_now;
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