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

  std::cout << "Just initialized" << std::endl;
  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  std::cout << "Loaded parameters" << std::endl;
  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  // Initialize current state and control to zero.
  state_ = VectorXd::Zero(dimension_);
  control_ = VectorXd::Zero(dimension_);
  std::cout << "Testing" << std::endl;

  // Initialize state space. For now, use an empty box.
  // TODO: populate with obstacles.
  space_ = BallsInBox::Create(dimension_); //Create returns a pointer to a BallsInBox object
  VectorXd lower(dimension_);
  VectorXd upper(dimension_);
  for (size_t ii = 0; ii < dimension_; ii++){
    lower(ii) = 0;
    upper(ii) = 1;
  }
  space_->SetBounds(lower, upper);
  std::cout << "Set bounds" << std::endl;
  const size_t kNumObstacles = 5;

  std::random_device r;
  std::default_random_engine engine(r()); 
  std::uniform_real_distribution<double> uniform_dist(0.05, 0.3);

  for (size_t ii = 0; ii < kNumObstacles; ii++){
    space_->AddObstacle(space_->Sample(), uniform_dist(engine));
  }
  
  std::cout << "Added obstacles" << std::endl;

  space_->Visualize(vis_pub_, fixed_frame_id_);
  std::cout << "Called visualize function" << std::endl;

  initialized_ = true;
  return true;
}

// Load all parameters from config files.
bool Simulator::LoadParameters(const ros::NodeHandle& n) {
  std::string key;

  // Control time step.
  if (!ros::param::search("meta_planner/control/time_step", key)) return false;
  if (!ros::param::get(key, time_step_)) return false;
  std::cout << "control/time_step" << std::endl;

  // Topics and frame ids.
  if (!ros::param::search("meta_planner/topics/control", key)) return false;
  if (!ros::param::get(key, control_topic_)) return false;
  std::cout << "topics/control" << std::endl;

  if (!ros::param::search("meta_planner/topics/sensor", key)) return false;
  if (!ros::param::get(key, sensor_topic_)) return false;
  std::cout << "topics/sensor" << std::endl;

  if (!ros::param::search("meta_planner/topics/vis", key)) return false;
  if (!ros::param::get(key, vis_topic_)) return false;
  std::cout << "topics/vis" << std::endl;

  if (!ros::param::search("meta_planner/frames/fixed", key)) return false;
  if (!ros::param::get(key, fixed_frame_id_)) return false;
  std::cout << "frames/fixed" << std::endl;

  if (!ros::param::search("meta_planner/frames/tracker", key)) return false;
  if (!ros::param::get(key, robot_frame_id_)) return false;
  std::cout << "frames/tracker" << std::endl;

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

  sensor_pub_ = nl.advertise<geometry_msgs::Vector3>(
    sensor_topic_.c_str(), 10, false);

  // Timer.
  timer_ = nl.createTimer(
    ros::Duration(time_step_), &Simulator::TimerCallback, this);

  return true;
}


// Callback for processing control signals.
void Simulator::ControlCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
  // TODO!
  // control_ = msg;
}

// Timer callback for generating sensor measurements and updating
// state based on last received control signal.
void Simulator::TimerCallback(const ros::TimerEvent& e) {
  // TODO!
  // state_ += control_ * time_step_;
  // if (abs(state_ - obstacle_state) <= delta){
  // sensor_pub_.publish(obstacleSensed);
  // }
}
