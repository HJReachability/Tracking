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
// Defines an n-dimensional box which inherits from Environment. Defaults to
// the unit box.
//
///////////////////////////////////////////////////////////////////////////////

#include <meta_planner/box.h>

namespace meta {

// Factory method. Use this instead of the constructor.
Box::Ptr Box::Create() {
  Box::Ptr ptr(new Box());
  return ptr;
}

// Constructor. Don't use this. Use the factory method instead.
Box::Box()
  : Environment(),
    lower_(Vector3d::Zero()),
    upper_(Vector3d::Constant(1.0)) {}

// Inherited from Environment, but can be overwritten by child classes.
Vector3d Box::Sample() const {
  Vector3d sample;

  // Sample each dimension from this distribution.
  for (size_t ii = 0; ii < 3; ii++) {
    std::uniform_real_distribution<double> unif(lower_(ii), upper_(ii));
    sample(ii) = unif(rng_);
  }

  return sample;
}

// Inherited from Environment, but can be overwritten by child classes.
// Returns true if the state is a valid configuration.
// Takes in incoming and outgoing value functions. See planner.h for details.
bool Box::IsValid(const Vector3d& position,
                  const ValueFunction::ConstPtr& incoming_value,
                  const ValueFunction::ConstPtr& outgoing_value) const {
#ifdef ENABLE_DEBUG_MESSAGES
  if (!incoming_value.get() || !outgoing_value.get()) {
    ROS_ERROR("Value function pointer was null.");
    return false;
  }
#endif

  // No obstacles. Just check bounds.
  for (size_t ii = 0; ii < 3; ii++) {
    const double bound = outgoing_value->
      SwitchingTrackingBound(ii, incoming_value->MaxPlannerSpeed(ii));

    if (position(ii) < lower_(ii) + bound ||
        position(ii) > upper_(ii) - bound)
      return false;
  }

  return true;
}

// Inherited by Environment, but can be overwritten by child classes.
// Assumes that the first <=3 dimensions correspond to R^3.
void Box::Visualize(const ros::Publisher& pub,
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
  cube.scale.x = upper_(0) - lower_(0);
  center.x = lower_(0) + 0.5 * cube.scale.x;

  cube.scale.y = upper_(1) - lower_(1);
  center.y = lower_(1) + 0.5 * cube.scale.y;

  cube.scale.z = upper_(2) - lower_(2);
  center.z = lower_(2) + 0.5 * cube.scale.z;

  cube.pose.position = center;
  cube.pose.orientation.x = 0.0;
  cube.pose.orientation.y = 0.0;
  cube.pose.orientation.z = 0.0;
  cube.pose.orientation.w = 1.0;

  // Publish marker.
  pub.publish(cube);
}

// Set bounds in each dimension.
void Box::SetBounds(const Vector3d& lower, const Vector3d& upper) {
  lower_ = lower;
  upper_ = upper;
}

} //\namespace meta
