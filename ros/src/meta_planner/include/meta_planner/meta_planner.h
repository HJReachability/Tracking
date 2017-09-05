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

#ifndef META_PLANNER_META_PLANNER_H
#define META_PLANNER_META_PLANNER_H

#include <demo/balls_in_box.h>
#include <meta_planner/near_hover_quad_no_yaw.h>
#include <meta_planner/value_function.h>
#include <meta_planner/analytical_point_mass_value_function.h>
#include <meta_planner/waypoint_tree.h>
#include <meta_planner/waypoint.h>
#include <meta_planner/ompl_planner.h>
#include <meta_planner/environment.h>
#include <meta_planner/trajectory.h>
#include <meta_planner/types.h>
#include <meta_planner/uncopyable.h>

#include <meta_planner_msgs/Trajectory.h>
#include <meta_planner_msgs/SensorMeasurement.h>
#include <crazyflie_msgs/PositionStateStamped.h>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <limits>

namespace meta {

class MetaPlanner : private Uncopyable {
public:
  ~MetaPlanner() {}
  explicit MetaPlanner()
    : initialized_(false) {}

  // Initialize this class from a ROS node.
  bool Initialize(const ros::NodeHandle& n);

private:
  // Load parameters and register callbacks.
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Callback for processing state updates.
  void StateCallback(const crazyflie_msgs::PositionStateStamped::ConstPtr& msg);

  // Callback for processing sensor measurements.
  void SensorCallback(const meta_planner_msgs::SensorMeasurement::ConstPtr& msg);

  // Callback to handle requests for new trajectory.
  void RequestTrajectoryCallback(const std_msgs::Empty::ConstPtr& msg);

  // Plan a trajectory from the given start to stop points and auto-publish.
  // Returns whether meta planning was successful.
  bool Plan(const Vector3d& start, const Vector3d& stop) const;

  // List of planners and flag for whether to load value functions from disk or
  // create analytic versions given parameters read from ROS.
  std::vector<Planner::ConstPtr> planners_;
  bool numerical_mode_;
  std::vector<std::string> value_directories_;

  std::vector<double> max_planner_speeds_;
  std::vector<double> max_velocity_disturbances_;
  std::vector<double> max_acceleration_disturbances_;

  // Geometric goal point.
  Vector3d goal_;

  // Current position, with flag for whether been updated since initialization.
  Vector3d position_;
  bool been_updated_;

  // Spaces and dimensions.
  size_t state_dim_;
  size_t control_dim_;
  BallsInBox::Ptr space_;

  std::vector<double> state_upper_;
  std::vector<double> state_lower_;

  // Control upper/lower bounds.
  NearHoverQuadNoYaw::ConstPtr dynamics_;
  std::vector<double> control_upper_;
  std::vector<double> control_lower_;

  // Max time to spend searching for an optimal path.
  double max_runtime_;

  // Maximum distance between waypoints.
  double max_connection_radius_;

  // Publishers/subscribers and related topics.
  ros::Publisher traj_pub_;
  ros::Publisher env_pub_;
  ros::Subscriber state_sub_;
  ros::Subscriber sensor_sub_;
  ros::Subscriber request_traj_sub_;

  std::string traj_topic_;
  std::string env_topic_;
  std::string state_topic_;
  std::string sensor_topic_;
  std::string request_traj_topic_;

  // Frames.
  std::string fixed_frame_id_;

  // Initialization and naming.
  bool initialized_;
  std::string name_;
};

} //\namespace meta

#endif
