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

  // TODO! Remove this assumption.
  if (state_dim_ != control_dim_) {
    ROS_ERROR("%s: State and control dimensions must be the same.",
              name_.c_str());
    return false;
  }

  // Initialize state space. For now, use an empty box.
  // TODO: Parameterize this somehow and integrate with occupancy grid.
  space_ = BallsInBox::Create(state_dim_);

  // Set state space bounds.
  VectorXd state_upper_vec(control_dim_);
  VectorXd state_lower_vec(control_dim_);
  for (size_t ii = 0; ii < state_dim_; ii++) {
    state_upper_vec(ii) = state_upper_[ii];
    state_lower_vec(ii) = state_lower_[ii];
  }

  space_->SetBounds(state_lower_vec, state_upper_vec);

  // Set the initial state and goal.
  state_ = state_lower_vec;
  goal_ = state_upper_vec;

  // Set control upper/lower bounds as Eigen::Vectors.
  VectorXd control_upper_vec(control_dim_);
  VectorXd control_lower_vec(control_dim_);
  for (size_t ii = 0; ii < control_dim_; ii++) {
    control_upper_vec(ii) = control_upper_[ii];
    control_lower_vec(ii) = control_lower_[ii];
  }

  // Create planners.
  for (size_t ii = 0; ii < value_files_.size(); ii++) {
    // NOTE: Assuming that planners operate in full state space for now.
    std::vector<size_t> dimensions(state_dim_);
    std::iota(dimensions.begin(), dimensions.end(), 0);

    // NOTE: Hardcoding dynamics for now. Assuming direct velocity control
    // in each dimension.
    const MatrixXd A(MatrixXd::Zero(state_dim_, state_dim_));
    const MatrixXd B(MatrixXd::Identity(state_dim_, state_dim_));
    const Dynamics::ConstPtr dynamics =
      LinearDynamics::Create(A, B, control_lower_vec, control_upper_vec);

    // Load up the value function.
    const ValueFunction::ConstPtr value =
      ValueFunction::Create(PRECOMPUTATION_DIR + value_files_[ii], dynamics);

#if 0
    const ValueFunction::ConstPtr value(NULL);
#endif

    // Create the planner.
    const Planner::ConstPtr planner =
      OmplPlanner<og::RRTConnect>::Create(value, space_,
                                          dimensions, max_speeds_[ii]);

    planners_.push_back(planner);
  }

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

  // Control parameters.
  if (!ros::param::search("meta/control/time_step", key)) return false;
  if (!ros::param::get(key, time_step_)) return false;

  int dimension = 1;
  if (!ros::param::search("meta/control/dim", key)) return false;
  if (!ros::param::get(key, dimension)) return false;
  control_dim_ = static_cast<size_t>(dimension);

  if (!ros::param::search("meta/control/upper", key)) return false;
  if (!ros::param::get(key, control_upper_)) return false;

  if (!ros::param::search("meta/control/lower", key)) return false;
  if (!ros::param::get(key, control_lower_)) return false;
  if (control_upper_.size() != control_dim_ ||
      control_lower_.size() != control_dim_) {
    ROS_ERROR("%s: Upper and/or lower bounds are in the wrong dimension.",
              name_.c_str());
    return false;
  }

  // Planner parameters.
  if (!ros::param::search("meta/planners/values", key)) return false;
  if (!ros::param::get(key, value_files_)) return false;
  if (value_files_.size() == 0) {
    ROS_ERROR("%s: Must specify at least one value function file.",
              name_.c_str());
    return false;
  }

  if (!ros::param::search("meta/planners/max_speeds", key)) return false;
  if (!ros::param::get(key, max_speeds_)) return false;
  if (max_speeds_.size() != value_files_.size()) {
    ROS_ERROR("%s: Must specify a max speed for each planner.", name_.c_str());
    return false;
  }

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

  if (!ros::param::search("meta/topics/rrt_connect", key)) return false;
  if (!ros::param::get(key, rrt_connect_vis_topic_)) return false;

  if (!ros::param::search("meta/frames/fixed", key)) return false;
  if (!ros::param::get(key, fixed_frame_id_)) return false;

  if (!ros::param::search("meta/frames/tracker", key)) return false;
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
    std::cout << "About to call meta planner." << std::endl;

    const MetaPlanner meta(space_);
    traj_ = meta.Plan(state_, goal_, planners_);

    std::cout << "Meta planner succeeded." << std::endl;
  }
}


// Callback for applying tracking controller.
void Tracker::TimerCallback(const ros::TimerEvent& e) {
  ros::Time current_time = ros::Time::now();

  std::cout << "Current time - traj end = " << current_time.toSec() - traj_->LastTime() << std::endl;

  // Rerun the meta planner if the current time is past the end of the
  // trajectory timeline.
  if (current_time.toSec() > traj_->LastTime()) {
    ROS_WARN("%s: Current time is past the end of the planned trajectory.",
             name_.c_str());

    // TODO! Change the goal here to be something read from a topic.
    std::cout << "About to call meta planner." << std::endl;

    const MetaPlanner meta(space_);
    traj_ = meta.Plan(state_, goal_, planners_);

    std::cout << "Meta planner succeeded." << std::endl;

    current_time = ros::Time::now();
    std::cout << "Current time - traj end = " << current_time.toSec() - traj_->LastTime() << std::endl;
  }

  // TODO! In a real (non-point mass) system, we will need to query some sort of
  // state filter to get our current state. For now, we just query tf and get position.

  // 0) Get current TF.
  geometry_msgs::TransformStamped tf;

  try {
    tf = tf_buffer_.lookupTransform(
      fixed_frame_id_.c_str(), tracker_frame_id_.c_str(), ros::Time(0));
  } catch(tf2::TransformException &ex) {
    ROS_WARN("%s: %s", name_.c_str(), ex.what());
    ROS_WARN("%s: Could not determine current state.", name_.c_str());
    ros::Duration(time_step_).sleep();
    return;
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

#if 0
  const VectorXd optimal_control = -max_speeds_[0] * relative_state / relative_state.norm();
#endif

  std::cout << "Optimal control is: " << optimal_control.transpose() << std::endl;

  // 4) Apply optimal control.
  geometry_msgs::Vector3 control_msg;
  control_msg.x = optimal_control(0);
  control_msg.y = optimal_control(1);
  control_msg.z = optimal_control(2);

  control_pub_.publish(control_msg);
}
