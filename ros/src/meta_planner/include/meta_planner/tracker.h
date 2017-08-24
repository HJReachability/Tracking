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

#ifndef META_PLANNER_TRACKER_H
#define META_PLANNER_TRACKER_H

#include <meta_planner/meta_planner.h>
#include <meta_planner/trajectory.h>
#include <meta_planner/box.h>
#include <demo/balls_in_box.h>
#include <meta_planner/types.h>
#include <meta_planner/uncopyable.h>
#include <meta_planner/ompl_planner.h>
#include <meta_planner/linear_dynamics.h>
#include <meta_planner/near_hover_quad_no_yaw.h>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <string>

namespace meta {

class Tracker : private Uncopyable {
public:
  explicit Tracker();
  ~Tracker();

  // Initialize this class with all parameters and callbacks.
  bool Initialize(const ros::NodeHandle& n);

private:
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Callback for processing sensor measurements.
  void SensorCallback(const geometry_msgs::Quaternion::ConstPtr& msg);

  // Callback for applying tracking controller.
  void TimerCallback(const ros::TimerEvent& e);

  // Run the meta planner.
  void RunMetaPlanner();

  // Current state and trajectory.
  VectorXd goal_;
  VectorXd state_;
  Trajectory::ConstPtr traj_;

  // Spaces and dimensions.
  size_t control_dim_;
  size_t state_dim_;
  BallsInBox::Ptr space_;

  std::vector<double> state_upper_;
  std::vector<double> state_lower_;

  // Control upper/lower bounds.
  NearHoverQuadNoYaw::ConstPtr dynamics_;
  std::vector<double> control_upper_;
  std::vector<double> control_lower_;

  // Planners and related parameters.
  std::vector<Planner::ConstPtr> planners_;
  std::vector<std::string> value_directories_;

  // Set a recurring timer for a discrete-time controller.
  ros::Timer timer_;
  double time_step_;

  // TF interfacing.
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster br_;

  // Publishers/subscribers and related topics.
  ros::Publisher control_pub_;
  ros::Publisher environment_pub_;
  ros::Publisher traj_pub_;
  ros::Publisher tracking_bound_pub_;
  ros::Subscriber sensor_sub_;

  std::string control_topic_;
  std::string environment_topic_;
  std::string traj_topic_;
  std::string tracking_bound_topic_;
  std::string sensor_topic_;

  // Frames of reference for reading current pose from tf tree.
  std::string fixed_frame_id_;
  std::string tracker_frame_id_;
  std::string planner_frame_id_;

  // Is this class initialized?
  bool initialized_;

  // Name of this class, for use in debug messages.
  std::string name_;
};

} //\namespace meta

#endif
