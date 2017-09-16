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

#include <ompl/util/Console.h>

namespace meta {

// Initialize this class from a ROS node.
bool MetaPlanner::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "meta_planner");

  // Set the initial position and goal to zero. Position will be updated
  // via a message and goal will be read from the parameter server.
  position_ = Vector3d::Zero();
  goal_ = Vector3d::Zero();

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

  space_->Seed(seed_);

  // Create value functions.
  std::vector<ValueFunction::ConstPtr> values;

  if (numerical_mode_) {
    for (size_t ii = 0; ii < value_directories_.size(); ii++) {
      const ValueFunction::ConstPtr value =
        ValueFunction::Create(value_directories_[ii], dynamics_,
                              state_dim_, control_dim_,
                              static_cast<ValueFunctionId>(ii));

      values.push_back(value);
    }
  } else {
    for (size_t ii = 0; ii < max_planner_speeds_.size(); ii++) {
      // Generate inputs for AnalyticalPointMassValueFunction.
      // SEMI-HACK! Manually feeding control/disturbance bounds.
      const Vector3d max_planner_speed =
        Vector3d::Constant(max_planner_speeds_[ii]);
      const Vector3d max_velocity_disturbance =
        Vector3d::Constant(max_velocity_disturbances_[ii]);
      const Vector3d max_acceleration_disturbance =
        Vector3d::Constant(max_acceleration_disturbances_[ii]);
      const Vector3d velocity_expansion = Vector3d::Constant(0.1);

      // Create analytical value function.
      const AnalyticalPointMassValueFunction::ConstPtr value =
        AnalyticalPointMassValueFunction::Create(max_planner_speed,
                                                 max_velocity_disturbance,
                                                 max_acceleration_disturbance,
                                                 velocity_expansion,
                                                 dynamics_,
                                                 static_cast<ValueFunctionId>(ii));

      values.push_back(value);
    }
  }

  if (values.size() % 2 != 0) {
    ROS_ERROR("%s: Must provide value functions in pairs.", name_.c_str());
    return false;
  }

  // Create planners.
  for (size_t ii = 0; ii < values.size() - 1; ii += 2) {
    const Planner::ConstPtr planner =
      OmplPlanner<og::BITstar>::Create(values[ii], values[ii + 1], space_);

    planners_.push_back(planner);
  }

  // Set OMPL log level.
  ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_ERROR);

  // Generate an initial trajectory and auto-publish.
  //  Plan(position_, goal_, ros::Time::now().toSec());

  // Publish environment.
  space_->Visualize(env_pub_, fixed_frame_id_);

  initialized_ = true;
  return true;
}

// Load parameters.
bool MetaPlanner::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Random seed.
  int seed = 0;
  if (!nl.getParam("meta/random/seed", seed)) return false;
  seed_ = static_cast<unsigned int>(seed);

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

  // Goal position.
  double goal_x, goal_y, goal_z;
  if (!nl.getParam("meta/goal/x", goal_x)) return false;
  if (!nl.getParam("meta/goal/y", goal_y)) return false;
  if (!nl.getParam("meta/goal/z", goal_z)) return false;
  goal_ = Vector3d(goal_x, goal_y, goal_z);

  // Topics and frame ids.
  if (!nl.getParam("meta/topics/sensor", sensor_topic_)) return false;
  if (!nl.getParam("meta/topics/vis/known_environment", env_topic_)) return false;
  if (!nl.getParam("meta/topics/traj", traj_topic_)) return false;
  if (!nl.getParam("meta/topics/state", state_topic_)) return false;
  if (!nl.getParam("meta/topics/request_traj", request_traj_topic_)) return false;
  if (!nl.getParam("meta/topics/trigger_replan", trigger_replan_topic_)) return false;
  if (!nl.getParam("meta/topics/in_flight", in_flight_topic_)) return false;

  if (!nl.getParam("meta/frames/fixed", fixed_frame_id_)) return false;

  return true;
}

// Register callbacks.
bool MetaPlanner::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Subscribers.
  sensor_sub_ = nl.subscribe(
    sensor_topic_.c_str(), 1, &MetaPlanner::SensorCallback, this);

  state_sub_ = nl.subscribe(
    state_topic_.c_str(), 1, &MetaPlanner::StateCallback, this);

  request_traj_sub_ = nl.subscribe(
    request_traj_topic_.c_str(), 1, &MetaPlanner::RequestTrajectoryCallback, this);

  in_flight_sub_ = nl.subscribe(
    in_flight_topic_.c_str(), 1, &MetaPlanner::InFlightCallback, this);

  // Visualization publisher(s).
  env_pub_ = nl.advertise<visualization_msgs::Marker>(
    env_topic_.c_str(), 1, false);

  // Triggering a replan event.
  trigger_replan_pub_ = nl.advertise<std_msgs::Empty>(
    trigger_replan_topic_.c_str(), 1, false);

  // Actual publishers.
  traj_pub_ = nl.advertise<meta_planner_msgs::Trajectory>(
    traj_topic_.c_str(), 1, false);

  return true;
}

// Callback for processing state updates.
void MetaPlanner::
StateCallback(const crazyflie_msgs::PositionStateStamped::ConstPtr& msg) {
  position_(0) = msg->state.x;
  position_(1) = msg->state.y;
  position_(2) = msg->state.z;

  been_updated_ = true;
}

// Callback for processing sensor measurements. Replan trajectory.
void MetaPlanner::
SensorCallback(const meta_planner_msgs::SensorMeasurement::ConstPtr& msg) {
  if (!in_flight_)
    return;

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
    // Trigger a replan.
    trigger_replan_pub_.publish(std_msgs::Empty());

    // Publish environment.
    space_->Visualize(env_pub_, fixed_frame_id_);
  }
}

// Callback to handle requests for new trajectory.
void MetaPlanner::RequestTrajectoryCallback(
  const meta_planner_msgs::TrajectoryRequest::ConstPtr& msg) {
  // Only plan if position has been updated.
  if (!been_updated_)
    return;

  ROS_INFO("%s: Recomputing trajectory.", name_.c_str());
  const ros::Time current_time = ros::Time::now();

  // Unpack msg.
  const double start_time = msg->start_time;

  VectorXd start_state(msg->start_state.dimension);
  for (size_t ii = 0; ii < start_state.size(); ii++)
    start_state(ii) = msg->start_state.state[ii];

  const Vector3d start_position = dynamics_->Puncture(start_state);

  // Check if the start position is close to the goal. If so, just return
  // a hover trajectory at the goal (assuming the least aggressive planner).
  if (reached_goal_ ||
      (std::abs(start_position(0) - goal_(0)) <
       planners_.back()->GetOutgoingValueFunction()->TrackingBound(0) &&
       std::abs(start_position(1) - goal_(1)) <
       planners_.back()->GetOutgoingValueFunction()->TrackingBound(1) &&
       std::abs(start_position(2) - goal_(2)) <
       planners_.back()->GetOutgoingValueFunction()->TrackingBound(2)))
    reached_goal_ = true;

  if (reached_goal_) {
    ROS_INFO("%s: Reached end of trajectory. Hovering in place.", name_.c_str());

    // Same point == goal, but three times.
    // NOTE! The middle one should be set using a function called
    // GuaranteedSwitchingTime which is not yet implemented.
    const std::vector<Vector3d> positions = { goal_, goal_, goal_ };

    // Get the bound value.
    const ValueFunction::ConstPtr bound_value = (traj_ == nullptr) ?
      planners_.front()->GetIncomingValueFunction() :
      traj_->GetControlValueFunction(current_time.toSec());

    // Get control value.
    const ValueFunction::ConstPtr control_value =
      planners_.back()->GetOutgoingValueFunction();

    // Get times.
    double switching_time = 10.0;
    if (!numerical_mode_) {
      const AnalyticalPointMassValueFunction::ConstPtr analytic_control =
        std::static_pointer_cast<const AnalyticalPointMassValueFunction>(control_value);

      const AnalyticalPointMassValueFunction::ConstPtr analytic_bound =
        std::static_pointer_cast<const AnalyticalPointMassValueFunction>(bound_value);

      switching_time = analytic_control->GuaranteedSwitchingTime(0, analytic_bound);
      switching_time = std::max(switching_time,
        analytic_control->GuaranteedSwitchingTime(1, analytic_bound));
      switching_time = std::max(switching_time,
        analytic_control->GuaranteedSwitchingTime(2, analytic_bound));
    }

    const std::vector<double> times =
      { current_time.toSec(),
        current_time.toSec() + switching_time + 0.1,
        current_time.toSec() + switching_time + 100.0 };

    // Set up values.
    const std::vector<ValueFunction::ConstPtr> bound_values =
      { bound_value, control_value, control_value };
    const std::vector<ValueFunction::ConstPtr> control_values =
      { control_value, control_value, control_value };
    const std::vector<VectorXd> states =
      dynamics_->LiftGeometricTrajectory(positions, times);

    // Construct trajectory and publish.
    const Trajectory::Ptr hover =
      Trajectory::Create(times, states, control_values, bound_values);
    traj_ = hover;

    traj_pub_.publish(hover->ToRosMessage());
    return;
  }

  if (!Plan(start_position, goal_, start_time)) {
    ROS_ERROR("%s: MetaPlanner failed. Please come again.", name_.c_str());
    return;
  }

  ROS_INFO("%s: MetaPlanner succeeded after %2.5f seconds.",
           name_.c_str(), (ros::Time::now() - current_time).toSec());
}

// Plan a trajectory using the given (ordered) list of Planners.
// (1) Set up a new RRT-like structure to hold the meta plan.
// (2) Sample a new point in the state space.
// (3) Find nearest neighbor.
// (4) Plan a trajectory (starting with most aggressive planner).
// (5) Try to connect to the goal point.
// (6) Stop when we have a feasible trajectory. Otherwise go to (2).
// (7) When finished, convert to a message and publish.
bool MetaPlanner::Plan(const Vector3d& start, const Vector3d& stop,
                       double start_time) {
  // Only plan if position has been updated.
  if (!been_updated_)
    return false;

  // (1) Set up a new RRT-like structure to hold the meta plan.
  const ros::Time current_time = ros::Time::now();
  const ValueFunction::ConstPtr start_value = (traj_ == nullptr) ?
    planners_.back()->GetOutgoingValueFunction() :
    traj_->GetBoundValueFunction(start_time);

  WaypointTree tree(start, start_value, start_time);

  bool found = false;
  while ((ros::Time::now() - current_time).toSec() < max_runtime_) {
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

    // Throw out this sample if too far from the nearest point.
    if (neighbors.size() != kNumNeighbors ||
        (neighbors[0]->point_ - sample).norm() > max_connection_radius_)
      continue;

    Waypoint::ConstPtr neighbor = neighbors[0];

    // Extract value function and corresponding planner ID from last waypoint.
    // If value is null, (i.e. at root) then set to planners_.size() since
    // any planner is valid from the root. Convert value ID to planner ID
    // by dividing by 2 since each planner has two value functions.
    const Trajectory::ConstPtr neighbor_traj = neighbor->traj_;
    const ValueFunction::ConstPtr neighbor_val = neighbor->value_;

    const size_t neighbor_planner_id = neighbor_val->Id() / 2;

    // (4) Plan a trajectory (starting with the most aggressive planner and ending
    // with the next-most cautious planner).
    Trajectory::Ptr traj = nullptr;
    ValueFunction::ConstPtr value_used = nullptr;
    for (size_t ii = 0;
         ii < std::min(neighbor_planner_id + 2, planners_.size()); ii++) {
      const Planner::ConstPtr planner = planners_[ii];

      value_used = planner->GetIncomingValueFunction();
      const ValueFunction::ConstPtr possible_next_value =
        planner->GetOutgoingValueFunction();

      // Since we might always end up switching, make sure this point
      // is not closer than the guaranteed switching distance.
      // NOTE! This enforces backtracking only one planner at a time.
      // In full generality, we would just need to replace possible_next_value
      // with the most cautious value.
      bool sample_too_close = false;
      if (!numerical_mode_) {
        const AnalyticalPointMassValueFunction::ConstPtr analytic_used =
          std::static_pointer_cast<
          const AnalyticalPointMassValueFunction>(value_used);
        const AnalyticalPointMassValueFunction::ConstPtr analytic_next =
          std::static_pointer_cast<
            const AnalyticalPointMassValueFunction>(possible_next_value);


        if (std::abs(neighbor->point_(0) - sample(0)) <
            analytic_next->GuaranteedSwitchingDistance(0, analytic_used) &&
            std::abs(neighbor->point_(1) - sample(1)) <
            analytic_next->GuaranteedSwitchingDistance(1, analytic_used) &&
            std::abs(neighbor->point_(2) - sample(2)) <
            analytic_next->GuaranteedSwitchingDistance(2, analytic_used))
          sample_too_close = true;
      } else {
        if (std::abs(neighbor->point_(0) - sample(0)) <
            possible_next_value->GuaranteedSwitchingDistance(0, value_used) &&
            std::abs(neighbor->point_(1) - sample(1)) <
            possible_next_value->GuaranteedSwitchingDistance(1, value_used) &&
            std::abs(neighbor->point_(2) - sample(2)) <
            possible_next_value->GuaranteedSwitchingDistance(2, value_used))
          sample_too_close = true;
      }

      if (sample_too_close)
        continue;

      // Plan using 10% of the available total runtime.
      // NOTE! This is just a heuristic and could easily be changed.
      const double time = (neighbor_traj == nullptr) ?
        start_time : neighbor_traj->LastTime();

      traj = planner->Plan(neighbor->point_, sample, time, 0.1 * max_runtime_);

      if (traj != nullptr) {
        // When we succeed...
        // If we just planned with a more cautious planner than the one used
        // by the nearest neighbor, do a 1-step backtrack.
        if (ii > neighbor_planner_id) {
#if 0
          std::cout << "Switched from planner " << neighbor_planner_id
                    << " with value id " << neighbor_val->Id()
                    << " to planner " << ii
                    << " with value id " << value_used->Id() << std::endl;
#endif
          // Clone the neighbor.
          const Vector3d jittered(neighbor->point_(0) + 1e-4,
                                  neighbor->point_(1) + 1e-4,
                                  neighbor->point_(2) + 1e-4);

          const double time = (neighbor_traj == nullptr) ?
            start_time : neighbor_traj->FirstTime();

          if (time <= start_time + 1e-8) {
            ROS_INFO_THROTTLE(1.0, "%s: Tried to clone the root.", name_.c_str());

            // Didn't really succeed. Can't clone the root in general.
            traj = nullptr;
          } else {
            Waypoint::ConstPtr clone =
              Waypoint::Create(jittered,
                               value_used,
                               Trajectory::Create(neighbor_traj, time),
                               neighbor->parent_);

            // Swap out the control value function in the neighbor's trajectory
            // and update time stamps accordingly.
            clone->traj_->ExecuteSwitch(value_used);

            // Insert the clone.
            tree.Insert(clone, false);

            // Adjust the time stamps for the new trajectory to occur after the
            // updated neighbor's trajectory.
            traj->ResetStartTime(clone->traj_->LastTime());

            // Neighbor is now clone.
            neighbor = clone;
          }
        }

        break;
      }
    }

    // Check if we could found a trajectory to this sample.
    if (traj == nullptr)
      continue;

    // Insert the sample.
    const Waypoint::ConstPtr waypoint = Waypoint::Create(
      sample, value_used, traj, neighbor);

    tree.Insert(waypoint, false);

    // (5) Try to connect to the goal point.
    Trajectory::Ptr goal_traj = nullptr;
    ValueFunction::ConstPtr goal_value_used = nullptr;
    const size_t planner_used_id = value_used->Id() / 2;

    if ((sample - stop).norm() <= max_connection_radius_) {
      for (size_t ii = 0;
           ii < std::min(planner_used_id + 2, planners_.size()); ii++) {
        const Planner::ConstPtr planner = planners_[ii];
        goal_value_used = planner->GetIncomingValueFunction();

        // We are never gonna need to switch if this succeeds.
        // Plan using 10% of the available total runtime.
        // NOTE! This is just a heuristic and could easily be changed.
        goal_traj =
          planner->Plan(sample, stop, traj->LastTime(), 0.1 * max_runtime_);

        if (goal_traj != nullptr) {
          // When we succeed... don't need to clone because waypoint has no kids.
          // If we just planned with a more cautious planner than the one used
          // by the nearest neighbor, do a 1-step backtrack.
          if (ii > neighbor_planner_id) {
            // Swap out the control value function in the neighbor's trajectory
            // and update time stamps accordingly.
            waypoint->traj_->ExecuteSwitch(goal_value_used);

            // Adjust the time stamps for the new trajectory to occur after the
            // updated neighbor's trajectory.
            goal_traj->ResetStartTime(waypoint->traj_->LastTime());
          }

          break;
        }
      }
    }

    // (6) If this sample was connected to the goal, update the tree terminus.
    if (goal_traj != nullptr) {
      // Connect to the goal.
      // NOTE: the first point in goal_traj coincides with the last point in
      // traj, but when we merge the two trajectories the std::map insertion
      // rules will prevent duplicates.
      const Waypoint::ConstPtr goal = Waypoint::Create(
        stop, value_used, goal_traj, waypoint);

      tree.Insert(goal, true);

      // Mark that we've found a valid trajectory.
      found = true;
    }
  }

  if (found) {
    // Get the best (fastest) trajectory out of the tree.
    const Trajectory::ConstPtr best = tree.BestTrajectory();
    ROS_INFO("%s: Publishing trajectory of length %zu.",
             name_.c_str(), best->Size());

    traj_ = best;
    traj_pub_.publish(best->ToRosMessage());
    return true;
  }

  return false;
}

} //\namespace meta
