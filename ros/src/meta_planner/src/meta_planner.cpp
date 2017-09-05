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
// Defines the MetaPlanner class. The MetaPlanner samples random points in
// the state space and then spawns off different Planners to plan Trajectories
// between these points (RRT-style).
//
///////////////////////////////////////////////////////////////////////////////

#include <meta_planner/meta_planner.h>

namespace meta {

// Initialize this class from a ROS node.
bool MetaPlanner::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "meta_planner");

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

  // Initialize state space. For now, use an empty box.
  // TODO: Parameterize this somehow and integrate with occupancy grid.
  space_ = BallsInBox::Create();

  // Set state space bounds.
  VectorXd state_upper_vec(state_dim_);
  VectorXd state_lower_vec(state_dim_);
  for (size_t ii = 0; ii < state_dim_; ii++) {
    state_upper_vec(ii) = state_upper_[ii];
    state_lower_vec(ii) = state_lower_[ii];
  }

  space_->SetBounds(dynamics_->Puncture(state_lower_vec),
                    dynamics_->Puncture(state_upper_vec));

  // Set the initial state and goal.
  const double kSmallNumber = 1.5;

  been_updated_ = false;
  position_ = 0.5 * (dynamics_->Puncture(state_lower_vec) +
                     dynamics_->Puncture(state_upper_vec));
  goal_ = dynamics_->Puncture(state_upper_vec) -
    Vector3d::Constant(kSmallNumber);

  // Create planners.
  if (numerical_mode_) {
    for (size_t ii = 0; ii < value_directories_.size(); ii++) {
      // NOTE: Assuming the 6D quadrotor model and geometric planner in 3D.
      // Load up the value function.
      const ValueFunction::ConstPtr value =
        ValueFunction::Create(value_directories_[ii], dynamics_,
                              state_dim_, control_dim_,
                              static_cast<ValueFunctionId>(ii));

      // Create the planner.
      const Planner::ConstPtr planner =
        OmplPlanner<og::BITstar>::Create(value, space_);

      planners_.push_back(planner);
    }
  } else {
    for (size_t ii = 0; ii < max_planner_speeds_.size(); ii++) {
      // Generate inputs for AnalyticalPointMassValueFunction.
      // HACK! Assuming knowledge of the control/dynamics.
      const Vector3d max_planner_speed =
        Vector3d::Constant(max_planner_speeds_[ii]);
      const Vector3d max_tracker_control(control_upper_[0],
                                         control_upper_[1],
                                         control_upper_[2]);
      const Vector3d max_tracker_acceleration(
        constants::G * std::tan(control_upper_[0]),
        constants::G * std::tan(control_upper_[1]),
        control_upper_[2] - constants::G);
      const Vector3d max_velocity_disturbance =
        Vector3d::Constant(max_velocity_disturbances_[ii]);
      const Vector3d max_acceleration_disturbance =
        Vector3d::Constant(max_acceleration_disturbances_[ii]);

      // Create analytical value function.
      const AnalyticalPointMassValueFunction::ConstPtr value =
        AnalyticalPointMassValueFunction::Create(max_planner_speed,
                                                 max_tracker_control,
                                                 max_tracker_acceleration,
                                                 max_velocity_disturbance,
                                                 max_acceleration_disturbance,
                                                 dynamics_,
                                                 static_cast<ValueFunctionId>(ii));

      // Create the planner.
      const Planner::ConstPtr planner =
        OmplPlanner<og::BITstar>::Create(value, space_);

      planners_.push_back(planner);
    }
  }

  // Generate an initial trajectory and auto-publish.
  Plan(position_, goal_);

  // Publish environment.
  space_->Visualize(env_pub_, fixed_frame_id_);

  initialized_ = true;
  return true;
}

// Load parameters.
bool MetaPlanner::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Meta planning parameters.
  if (!nl.getParam("meta/meta/max_runtime", max_runtime_))
    return false;
  if (!nl.getParam("meta/meta/max_connection_radius", max_connection_radius_))
    return false;

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

  if (!nl.getParam("meta/state/upper", state_upper_)) return false;
  if (!nl.getParam("meta/state/lower", state_lower_)) return false;

  // Topics and frame ids.
  if (!nl.getParam("meta/topics/sensor", sensor_topic_)) return false;
  if (!nl.getParam("meta/topics/vis/known_environment", env_topic_)) return false;
  if (!nl.getParam("meta/topics/traj", traj_topic_)) return false;
  if (!nl.getParam("meta/topics/state", state_topic_)) return false;
  if (!nl.getParam("meta/topics/request_traj", request_traj_topic_)) return false;

  if (!nl.getParam("meta/frames/fixed", fixed_frame_id_)) return false;

  return true;
}

// Register callbacks.
bool MetaPlanner::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Subscribers.
  sensor_sub_ = nl.subscribe(
    sensor_topic_.c_str(), 10, &MetaPlanner::SensorCallback, this);

  state_sub_ = nl.subscribe(
    state_topic_.c_str(), 10, &MetaPlanner::StateCallback, this);

  request_traj_sub_ = nl.subscribe(
    request_traj_topic_.c_str(), 10, &MetaPlanner::RequestTrajectoryCallback, this);


  // Visualization publisher(s).
  env_pub_ = nl.advertise<visualization_msgs::Marker>(
    env_topic_.c_str(), 10, false);

  // Actual publishers.
  traj_pub_ = nl.advertise<meta_planner_msgs::Trajectory>(
    traj_topic_.c_str(), 10, false);

  return true;
}

// Callback for processing state updates.
void MetaPlanner::
StateCallback(const crazyflie_msgs::PositionStateStamped::ConstPtr& msg) {
  position_(0) = msg->state.x;
  position_(1) = msg->state.y;
  position_(2) = msg->state.z;
}

// Callback for processing sensor measurements. Replan trajectory.
void MetaPlanner::
SensorCallback(const meta_planner_msgs::SensorMeasurement::ConstPtr& msg) {
  bool unseen_obstacle = false;

  for (size_t ii = 0; ii < msg->num_obstacles; ii++) {
    const double radius = msg->radii[ii];
    const Vector3d point(msg->positions[ii].x,
                         msg->positions[ii].y,
                         msg->positions[ii].z);

    // Check if our version of the map has already seen this point.
    if (!(space_->IsObstacle(point, radius))) {
      space_->AddObstacle(point, radius);
      unseen_obstacle = true;
    }
  }

  if (unseen_obstacle) {
    // Replan and auto-publish.
    Plan(position_, goal_);

    // Publish environment.
    space_->Visualize(env_pub_, fixed_frame_id_);
  }
}

// Callback to handle requests for new trajectory.
void MetaPlanner::RequestTrajectoryCallback(const std_msgs::Empty::ConstPtr& msg) {
  ROS_INFO("%s: Recomputing trajectory.", name_.c_str());

  const ros::Time start_time = ros::Time::now();
  while (!Plan(position_, goal_)) {
    ROS_ERROR("%s: MetaPlanner failed. Retrying.", name_.c_str());
    ros::Duration(0.1).sleep();
  }

  ROS_INFO("%s: MetaPlanner succeeded after %2.5f seconds.",
           name_.c_str(), (ros::Time::now() - start_time).toSec());
}

// Plan a trajectory using the given (ordered) list of Planners.
// (1) Set up a new RRT-like structure to hold the meta plan.
// (2) Sample a new point in the state space.
// (3) Find nearest neighbor.
// (4) Plan a trajectory (starting with most aggressive planner).
// (5) Try to connect to the goal point.
// (6) Stop when we have a feasible trajectory. Otherwise go to (2).
// (7) When finished, convert to a message and publish.
bool MetaPlanner::Plan(const Vector3d& start, const Vector3d& stop) const {
  // (1) Set up a new RRT-like structure to hold the meta plan.
  const ros::Time start_time = ros::Time::now();
  WaypointTree tree(start, start_time.toSec());

  bool found = false;
  while ((ros::Time::now() - start_time).toSec() < max_runtime_) {
    // (2) Sample a new point in the state space.
    Vector3d sample = space_->Sample();

    // Throw out this sample if it could never lead to a faster trajectory than
    // the best one currently.
    // NOTE! This test assumes that the first planner is the fastest.
    // NOTE! If no valid trajectory has been found, the tree's best time will
    // be infinite, so this test will automatically fail.
    if (planners_.front()->BestPossibleTime(start, sample) +
        planners_.front()->BestPossibleTime(sample, stop) > tree.BestTime())
      continue;

    // (3) Find the nearest neighbor.
    const size_t kNumNeighbors = 1;
    const std::vector<Waypoint::ConstPtr> neighbors =
      tree.KnnSearch(sample, kNumNeighbors);

    if (neighbors.size() != kNumNeighbors ||
        (neighbors[0]->point_ - sample).norm() > max_connection_radius_)
      continue;

    // (4) Plan a trajectory (starting with most aggressive planner).
    Trajectory::Ptr traj = nullptr;
    for (size_t ii = 0; ii < planners_.size(); ii++) {
      const Planner::ConstPtr planner = planners_[ii];

      // Plan using 10% of the available total runtime.
      // NOTE! This is just a heuristic and could easily be changed.
      traj = planner->Plan(neighbors[0]->point_, sample,
                           neighbors[0]->time_, 0.1 * max_runtime_);

      if (traj != nullptr)
        break;
    }

    // Check if we could found a trajectory to this sample.
    if (traj == nullptr)
      continue;

    // (5) Try to connect to the goal point.
    Trajectory::Ptr goal_traj = nullptr;
    if ((sample - stop).norm() <= max_connection_radius_) {
      for (size_t ii = 0; ii < planners_.size(); ii++) {
        const Planner::ConstPtr planner = planners_[ii];

        // Plan using 10% of the available total runtime.
        // NOTE! This is just a heuristic and could easily be changed.
        goal_traj =
          planner->Plan(sample, stop, traj->LastTime(), 0.1 * max_runtime_);

        if (goal_traj != nullptr)
          break;
      }
    }

    // Insert the sample.
    const Waypoint::ConstPtr waypoint = Waypoint::Create(
      sample, traj->LastTime(), traj, neighbors[0]);

    tree.Insert(waypoint, false);

    // (6) If this sample was connected to the goal, update the tree terminus.
    if (goal_traj != nullptr) {
      // Connect to the goal.
      // NOTE: the first point in goal_traj coincides with the last point in
      // traj, but when we merge the two trajectories the std::map insertion
      // rules will prevent duplicates.
      const Waypoint::ConstPtr goal = Waypoint::Create(
        stop, goal_traj->LastTime(), goal_traj, waypoint);

      tree.Insert(goal, true);

      // Mark that we've found a valid trajectory.
      found = true;
      ROS_INFO("%s: Found a valid trajectory.", name_.c_str());
    }
  }

  if (found) {
    // Get the best (fastest) trajectory out of the tree.
    const Trajectory::ConstPtr best = tree.BestTrajectory();
    ROS_INFO("%s: Publishing trajectory of length %zu.",
             name_.c_str(), best->Size());

    traj_pub_.publish(best->ToRosMessage());
    return true;
  }

  return false;
}

} //\namespace meta
