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


// Callback for processing sensor measurements.
void Tracker::SensorCallback(const geometry_msgs::Quaternion::ConstPtr& msg){
// Replan trajectory.
  VectorXd point(3);
  point(0) = msg->x;
  point(1) = msg->y;
  point(2) = msg->z;

  double radius = msg->w;
   
  if (!(space_->IsObstacle(point, radius))) {  
    //Add obstacle to the environment.
    space_->AddObstacle(point, radius);
    //Run meta_planner
    VectorXd goal(3);
    for (size_t ii = 0; ii < goal.size(); ii++)
      goal(ii) = 1;
    
    const MetaPlanner this_meta_planner(space_);
    traj_ = this_meta_planner.Plan(state_, goal, planners_);
  }
}


// Callback for applying tracking controller.
void Tracker::TimerCallback(const ros::TimerEvent& e) {
  geometry_msgs::Vector3 control;
  control.x = 1;
  control.y = 1;
  control.z = 1;
  control_pub_.publish(control);

#if 0
  // 0) Get current TF.
  geometry_msgs::TransformStamped tf;

  try {
    tf = tf_buffer_.lookupTransform(
      fixed_frame_id_.c_str(), tracker_frame_id_.c_str(), ros::Time::now());
  } catch(tf2::TransformException &ex) {
    ROS_WARN("%s: %s", name_.c_str(), ex.what());
    ROS_WARN("%s: Could not determine current state.", name_.c_str());
    ros::Duration(time_step_).sleep();
    return;
  }

  // Transform point cloud into world frame.
  const Vector3d translation(tf.transform.translation.x,
                             tf.transform.translation.y,
                             tf.transform.translation.z);
  const Quaterniond quat(tf.transform.rotation.w,
                         tf.transform.rotation.x,
                         tf.transform.rotation.y,
                         tf.transform.rotation.z);
  const Matrix3d rotation = quat.toRotationMatrix();

  // Process pose to get other state dimensions, e.g. velocity.
  // TODO!

  // 1) Compute relative state.
  planner_state = traj_.State(current_time);
  rel_state = state - planner_state;

  // 2) Get corresponding value function.
  const ValueFunction::ConstPtr value = traj_.ValueFunction(current_time);

  // 3) Interpolate gradient to get optimal control.
  opt_control = value->OptimalControl(rel_state);

  // 4) Apply optimal control.
  control_pub_.publish(opt_control);
#endif
}
