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

// Factory method. Use this instead of the constructor.
BallsInBox::Ptr BallsInBox::Create(size_t dimension) {
  BallsInBox::Ptr ptr(new BallsInBox(dimension));
  return ptr;
}

// Constructor. Don't use this. Use the factory method instead.
BallsInBox::BallsInBox(size_t dimension)
  : Box(dimension) {}

// Inherited sampler from Box needs to be overwritten.
VectorXd BallsInBox::Sample() const {
  VectorXd sample(dimension_);

  bool done = false;
  while (!done) {
    // Sample each dimension from this distribution.
    for (size_t ii = 0; ii < dimension_; ii++) {
      std::uniform_real_distribution<double> unif(lower_(ii), upper_(ii));
      sample(ii) = unif(rng_);
    }

    // Check if this sample is valid.
    done = IsValid(sample);
  }

  return sample;
}

// Inherited collision checker from Box needs to be overwritten.
bool BallsInBox::IsValid(const VectorXd& state) const {
#ifdef ENABLE_DEBUG_MESSAGES
  if (state.size() != dimension_)
    ROS_ERROR("Improperly sized state vector (%zu vs. %zu).",
              state.size(), dimension_);
#endif

  // Check bounds.
  for (size_t ii = 0; ii < state.size(); ii++)
    if (state(ii) < lower_(ii) || state(ii) > upper_(ii))
      return false;

  // Check against each obstacle.
  for (size_t ii = 0; ii < points_.size(); ii++)
    if ((state - points_[ii]).norm() <= radii_[ii])
      return false;

  return true;
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
  cube.color.a = 0.25;
  cube.color.r = 0.2;
  cube.color.g = 0.2;
  cube.color.b = 0.2;

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

  // TODO: visualize obstacles as a SPHERE_LIST marker.
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
