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
// Defines a Box environment with spherical obstacles. For simplicity, this
// does not bother with a kdtree index to speed up collision queries, since
// it is only for a simulated demo.
//
///////////////////////////////////////////////////////////////////////////////

#include <demo/balls_in_box.h>

namespace meta {

// Factory method. Use this instead of the constructor.
BallsInBox::Ptr BallsInBox::Create(size_t dimension) {
  BallsInBox::Ptr ptr(new BallsInBox(dimension));
  return ptr;
}

// Constructor. Don't use this. Use the factory method instead.
BallsInBox::BallsInBox(size_t dimension)
  : Box(dimension) {}

// Inherited collision checker from Box needs to be overwritten.
bool BallsInBox::IsValid(const VectorXd& state,
                         const ValueFunction::ConstPtr& value) const {
#ifdef ENABLE_DEBUG_MESSAGES
  if (state.size() != dimension_) {
    ROS_ERROR("Improperly sized state vector (%zu vs. %zu).",
              state.size(), dimension_);
    return false;
  }

  if (!value.get()) {
    ROS_ERROR("Value function pointer was null.");
    return false;
  }
#endif

  // Check bounds.
  for (size_t ii = 0; ii < state.size(); ii++) {
    const double bound = value->TrackingBound(ii);

    if (state(ii) < lower_(ii) + bound ||
        state(ii) > upper_(ii) - bound)
      return false;
  }

  // Check against each obstacle.
  // NOTE: assuming rectangular tracking bound.
  for (size_t ii = 0; ii < points_.size(); ii++) {
    const VectorXd& p = points_[ii];

    // Find the corner of the tracking bound which is closest to this obstacle.
    VectorXd corner(state.size());
    for (size_t jj = 0; jj < corner.size(); jj++)
      corner(jj) = (state(jj) - p(jj) > 0.0) ?
        state(jj) - value->TrackingBound(jj) :
        state(jj) + value->TrackingBound(jj);

    if ((corner - p).norm() <= radii_[ii])
      return false;
  }

  return true;
}


//Checks for obstacles within a sensing radius.
bool BallsInBox::SenseObstacle(const VectorXd& state, double sensor_radius,
                               VectorXd& obstacle_position,
                               double& obstacle_radius) const{
  for (size_t ii = 0; ii < points_.size(); ii++){
    if ((state - points_[ii]).norm() <= radii_[ii] + sensor_radius) {
      obstacle_position(0) = points_[ii](0);
      obstacle_position(1) = points_[ii](1);
      obstacle_position(2) = points_[ii](2);

      obstacle_radius = radii_[ii];

      return true;
    }
  }

  return false;
}

// Checks if a given obstacle is in the environment.
bool BallsInBox::IsObstacle(const VectorXd& obstacle_position,
                            double obstacle_radius) const {
  for (size_t ii = 0; ii < points_.size(); ii++)
    if ((obstacle_position - points_[ii]).norm() < 1e-8 &&
        std::abs(obstacle_radius - radii_[ii]) < 1e-8)
      return true;

  return false;
}


// Inherited visualizer from Box needs to be overwritten.
void BallsInBox::Visualize(const ros::Publisher& pub,
                           const std::string& frame_id) const {
  if (pub.getNumSubscribers() <= 0)
    return;

  // Set up box marker.
  visualization_msgs::Marker cube;
  cube.ns = "cube";
  cube.header.frame_id = frame_id;
  cube.header.stamp = ros::Time::now();
  cube.id = 0;
  cube.type = visualization_msgs::Marker::CUBE;
  cube.action = visualization_msgs::Marker::ADD;
  cube.color.a = 0.5;
  cube.color.r = 0.3;
  cube.color.g = 0.7;
  cube.color.b = 0.7;

  geometry_msgs::Point center;

  // Fill in center and scale.
  if (dimension_ >= 1) {
    cube.scale.x = upper_[0] - lower_[0];
    center.x = lower_[0] + 0.5 * cube.scale.x;
  }

  if (dimension_ >= 2) {
    cube.scale.y = upper_[1] - lower_[1];
    center.y = lower_[1] + 0.5 * cube.scale.y;
  }

  if (dimension_ >= 3) {
    cube.scale.z = upper_[2] - lower_[2];
    center.z = lower_[2] + 0.5 * cube.scale.z;
  }

  cube.pose.position = center;
  cube.pose.orientation.x = 0.0;
  cube.pose.orientation.y = 0.0;
  cube.pose.orientation.z = 0.0;
  cube.pose.orientation.w = 1.0;

  // Publish cube marker.
  pub.publish(cube);


  // TODO: visualize obstacles as SPHERE markers.
  for (size_t ii = 0; ii < points_.size(); ii++){
    visualization_msgs::Marker sphere;
    sphere.ns = "sphere";
    sphere.header.frame_id = frame_id;
    sphere.header.stamp = ros::Time::now();
    sphere.id = static_cast<int>(ii);
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;

    sphere.scale.x = 2.0 * radii_[ii];
    sphere.scale.y = 2.0 * radii_[ii];
    sphere.scale.z = 2.0 * radii_[ii];

    sphere.color.a = 0.9;
    sphere.color.r = 0.7;
    sphere.color.g = 0.5;
    sphere.color.b = 0.5;

    geometry_msgs::Point p;
    const VectorXd point = points_[ii];
    p.x = point(0);
    p.y = point(1);
    p.z = point(2);

    sphere.pose.position = p;

    // Publish sphere marker.
    pub.publish(sphere);
  }

}

// Add a spherical obstacle of the given radius to the environment.
void BallsInBox::AddObstacle(const VectorXd& point, double r) {
  const double kSmallNumber = 1e-8;

#ifdef ENABLE_DEBUG_MESSAGES
  if (point.size() != dimension_)
    ROS_ERROR("Improperly sized point (%zu vs. %zu).",
              point.size(), dimension_);

  if (r < kSmallNumber)
    ROS_ERROR("Radius was too small: %f.", r);
#endif

  points_.push_back(point);
  radii_.push_back(std::max(r, kSmallNumber));
}

} //\namespace meta
