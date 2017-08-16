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

Simulator::Simulator()
  : initialized_(false),
    dimension_(3) {}

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

  // Initialize current state and control to zero.
  state_ = VectorXd::Zero(dimension_);
  control_ = VectorXd::Zero(dimension_);

  // Initialize state space. For now, use an empty box.
  // TODO: populate with obstacles.
  space_ = BallsInBox::Create(dimension_);

  // Box has corners at (0, 0, 0) and (10, 10, 10) for 3D.
  const VectorXd lower(VectorXd::Constant(3, 0.0));
  const VectorXd upper(VectorXd::Constant(3, 10.0));

  space_->SetBounds(lower, upper);
  const size_t kNumObstacles = 5;

  std::random_device rd;
  std::default_random_engine rng(rd());
  std::uniform_real_distribution<double> uniform_radius(0.3, 1.0);

  // Add an obstacle with a random radius at a random location.
  for (size_t ii = 0; ii < kNumObstacles; ii++)
    space_->AddObstacle(space_->Sample(), uniform_radius(rng));

  // Set the initial time.
  time_ = ros::Time::now();

  initialized_ = true;
  return true;
}

// Load all parameters from config files.
bool Simulator::LoadParameters(const ros::NodeHandle& n) {
  std::string key;

  // Control time step.
  if (!ros::param::search("meta/simulator/time_step", key)) return false;
  if (!ros::param::get(key, time_step_)) return false;

  // Topics and frame ids.
  if (!ros::param::search("meta/topics/control", key)) return false;
  if (!ros::param::get(key, control_topic_)) return false;

  if (!ros::param::search("meta/topics/sensor", key)) return false;
  if (!ros::param::get(key, sensor_topic_)) return false;

  if (!ros::param::search("meta/topics/vis", key)) return false;
  if (!ros::param::get(key, vis_topic_)) return false;

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
  vis_pub_ = nl.advertise<visualization_msgs::Marker>(
    vis_topic_.c_str(), 10, false);

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
  // TODO! Publish a translucent sphere showing the sensor radius.
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
  double sensor_radius = 2.0;

  if (space_->SenseObstacle(state_.head(3), sensor_radius,
                            obstacle_position, obstacle_radius)) {
    geometry_msgs::Quaternion q;
    q.x = obstacle_position(0);
    q.y = obstacle_position(1);
    q.z = obstacle_position(2);
    q.w = obstacle_radius;

    sensor_pub_.publish(q);
  }

  space_->Visualize(vis_pub_, fixed_frame_id_);
}
