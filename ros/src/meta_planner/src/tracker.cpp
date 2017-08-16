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
  : initialized_(false),
    tf_listener_(tf_buffer_) {}

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

  // Initialize current state to zero.
  state_ = VectorXd::Zero(dimension_);

  // Initialize state space. For now, use an empty box.
  // TODO: parameterize this somehow and integrate with occupancy grid.
  space_ = BallsInBox::Create(dimension_);

  // Create planner variable.
  const size_t kAmbientDimension = 3;
  const double kVelocity = 1.0;
  std::vector<size_t> dimensions(kAmbientDimension);
  std::iota(dimensions.begin(), dimensions.end(), 0);

  // Create a nullptr for the ValueFunction.
  const ValueFunction::ConstPtr null_value(NULL);

  // Fill in the list of planners for the meta-planner to use.
  const Planner::ConstPtr planner = OmplPlanner<og::RRTConnect>::Create(
    null_value, space_, dimensions, kVelocity);

  planners_.push_back(planner);

  // Generate an initial trajectory.
  // TODO! Change the goal here to be something read from a topic.
  VectorXd goal(3);
  for (size_t ii = 0; ii < goal.size(); ii++)
    goal(ii) = 1.0;

  std::cout << "About to call meta planner." << std::endl;

  const MetaPlanner meta(space_);
  traj_ = meta.Plan(state_, goal, planners_);

  std::cout << "Meta planner succeeded." << std::endl << std::flush;

  initialized_ = true;
  return true;
}

// Load all parameters from config files.
bool Tracker::LoadParameters(const ros::NodeHandle& n) {
  std::string key;

  // Control update time step.
  if (!ros::param::search("meta_planner/control/time_step", key)) return false;
  if (!ros::param::get(key, time_step_)) return false;

  // State space parameters.
  int dimension = 1;
  if (!ros::param::search("meta_planner/state_space/dimension", key)) return false;
  if (!ros::param::get(key, dimension)) return false;
  dimension_ = static_cast<size_t>(dimension);

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
  sensor_sub_ = nl.subscribe(sensor_topic_.c_str(), 10, &Tracker::SensorCallback, this);

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


// Callback for processing sensor measurements. Replan trajectory.
void Tracker::SensorCallback(const geometry_msgs::Quaternion::ConstPtr& msg){
  VectorXd point(3);
  point(0) = msg->x;
  point(1) = msg->y;
  point(2) = msg->z;

  double radius = msg->w;

  // Check if our version of the map has already seen this point.
  if (!(space_->IsObstacle(point, radius))) {
    space_->AddObstacle(point, radius);

    // Run meta_planner.
    // TODO! Change the goal here to be something read from a topic.
    VectorXd goal(3);
    for (size_t ii = 0; ii < goal.size(); ii++)
      goal(ii) = 1;

    std::cout << "About to call meta planner." << std::endl;

    const MetaPlanner meta(space_);
    traj_ = meta.Plan(state_, goal, planners_);

    std::cout << "Meta planner succeeded." << std::endl;
  }
}


// Callback for applying tracking controller.
void Tracker::TimerCallback(const ros::TimerEvent& e) {
  const ros::Time current_time = ros::Time::now();

  // TODO! In a real (non-point mass) system, we will need to query some sort of
  // state filter to get our current state. For now, we just query tf and get position.

  // 0) Get current TF.
  geometry_msgs::TransformStamped tf;

  try {
    tf = tf_buffer_.lookupTransform(
      fixed_frame_id_.c_str(), tracker_frame_id_.c_str(), current_time);
  } catch(tf2::TransformException &ex) {
    ROS_WARN("%s: %s", name_.c_str(), ex.what());
    ROS_WARN("%s: Could not determine current state.", name_.c_str());
    //    return;
  }

  // 1) Compute relative state.
  VectorXd state(3);
  state(0) = tf.transform.translation.x;
  state(1) = tf.transform.translation.y;
  state(2) = tf.transform.translation.z;

  std::cout << "State is: " << state.transpose() << std::endl;

  const VectorXd planner_state = traj_->GetState(current_time.toSec());
  const VectorXd relative_state = state - planner_state;

  std::cout << "Relative state is: " << relative_state.transpose() << std::endl;

  // 2) Get corresponding value function.
  const ValueFunction::ConstPtr value = traj_->GetValueFunction(current_time.toSec());

  // 3) Interpolate gradient to get optimal control.
  const VectorXd optimal_control = value->OptimalControl(relative_state);

  std::cout << "Optimal control is: " << optimal_control.transpose() << std::endl;

  // 4) Apply optimal control.
  geometry_msgs::Vector3 control_msg;
  control_msg.x = optimal_control(0);
  control_msg.y = optimal_control(1);
  control_msg.z = optimal_control(2);

  control_pub_.publish(control_msg);
}
