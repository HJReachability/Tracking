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
  space_ = BallsInBox::Create(dimension_); //Create returns a pointer to a BallsInBox object
  VectorXd lower(dimension_);
  VectorXd upper(dimension_);
  //Box has corners at (0, 0, 0) and (1, 1, 1) for 3D
  for (size_t ii = 0; ii < dimension_; ii++){
    lower(ii) = 0.0;
    upper(ii) = 1.0;
  }
  space_->SetBounds(lower, upper);
  const size_t kNumObstacles = 5; 

  std::random_device r;
  std::default_random_engine engine(r()); 
  std::uniform_real_distribution<double> uniform_dist(0.05, 0.3);
  //Add an obstacle with a random radius at a random location
  for (size_t ii = 0; ii < kNumObstacles; ii++){
    space_->AddObstacle(space_->Sample(), uniform_dist(engine));
  }

  time_ = ros::WallTime::now();

  initialized_ = true;
  return true;
}

// Load all parameters from config files.
bool Simulator::LoadParameters(const ros::NodeHandle& n) {
  std::string key;

  // Control time step.
  if (!ros::param::search("meta_planner/control/time_step", key)) return false;
  if (!ros::param::get(key, time_step_)) return false;

  // Topics and frame ids.
  if (!ros::param::search("meta_planner/topics/control", key)) return false;
  if (!ros::param::get(key, control_topic_)) return false;

  if (!ros::param::search("meta_planner/topics/sensor", key)) return false;
  if (!ros::param::get(key, sensor_topic_)) return false;

  if (!ros::param::search("meta_planner/topics/vis", key)) return false;
  if (!ros::param::get(key, vis_topic_)) return false;

  if (!ros::param::search("meta_planner/frames/fixed", key)) return false;
  if (!ros::param::get(key, fixed_frame_id_)) return false;

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

  sensor_pub_ = nl.advertise<geometry_msgs::Quaternion>(
    sensor_topic_.c_str(), 10, false);

  // Timer.
  timer_ = nl.createTimer(
    ros::Duration(time_step_), &Simulator::TimerCallback, this);

  return true;
}


// Callback for processing control signals.
void Simulator::ControlCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
  // TODO!
  control_(0) = msg->x;
  control_(1) = msg->y;
  control_(2) = msg->z;
}

// Timer callback for generating sensor measurements and updating
// state based on last received control signal.
void Simulator::TimerCallback(const ros::TimerEvent& e) {
  // TODO!
  //Update state
  const ros::WallTime now = ros::WallTime::now();
  const double dt = (now - time_).toSec();
  time_ = now;
  for (size_t ii = 0; ii < state_.size(); ii++)
    state_(ii) += control_(ii) * dt;
  

  // Broadcast tf
  geometry_msgs::TransformStamped transform_stamped;
 
  transform_stamped.header.frame_id = fixed_frame_id_;
  transform_stamped.header.stamp = ros::Time::now();
  
  transform_stamped.child_frame_id = robot_frame_id_;

  transform_stamped.transform.translation.x = state_(0);
  transform_stamped.transform.translation.y = state_(1);
  transform_stamped.transform.translation.z = state_(2);
 
  transform_stamped.transform.rotation.x = 0;
  transform_stamped.transform.rotation.y = 0;
  transform_stamped.transform.rotation.z = 0;
  transform_stamped.transform.rotation.w = 1;
  
  br_.sendTransform(transform_stamped);


  //Publish sensor message if an obstacle is within range
  VectorXd point(3);
  for (size_t ii = 0; ii < point.size(); ii++){
    point(ii) = 0;
  }
  double radius = 0;
  double sensingDist = 0.05;
  bool obstacle_sensed = space_->SenseObstacle(state_, point, radius, sensingDist);
  if (obstacle_sensed) {
    geometry_msgs::Quaternion q;
    q.x = point(0);
    q.y = point(1);
    q.z = point(2);
    q.w = radius;
    sensor_pub_.publish(q);
  }

  space_->Visualize(vis_pub_, fixed_frame_id_);
 
}
