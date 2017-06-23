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
// Defines the Trajectory struct.
//
///////////////////////////////////////////////////////////////////////////////

#include <meta_planner/trajectory.h>

// Visualize this trajectory in RVIZ.
void Trajectory::Visualize(const ros::Publisher& pub,
                           const std::string& frame_id) const {
  if (pub.getNumSubscribers() <= 0)
    return;

  // Set up spheres marker.
  visualization_msgs::Marker spheres;
  spheres.ns = "spheres";
  spheres.header.frame_id  = frame_id;
  spheres.header.stamp = ros::Time::now();
  spheres.id = 0;
  spheres.type = visualization_msgs::Marker::SPHERE_LIST;
  spheres.action = visualization_msgs::Marker::ADD;

  spheres.scale.x = 0.5;
  spheres.scale.y = 0.5;
  spheres.scale.z = 0.5;

#if 0
  spheres.color.a = 0.5;
  spheres.color.r = 0.7;
  spheres.color.g = 0.0;
  spheres.color.b = 0.8;
#endif

  // Set up line strip marker.
  visualization_msgs::Marker lines;
  lines.ns = "lines";
  lines.header.frame_id  = frame_id;
  lines.header.stamp = ros::Time::now();
  lines.id = 0;
  lines.type = visualization_msgs::Marker::LINE_STRIP;
  lines.action = visualization_msgs::Marker::ADD;

  lines.scale.x = 0.1;

#if 0
  lines.color.a = 0.5;
  lines.color.r = 0.0;
  lines.color.g = 0.0;
  lines.color.b = 0.6;
#endif

  // Iterate through the trajectory and append to markers.
  for (size_t ii = 0; ii < Size(); ii++) {
    geometry_msgs::Point p;
    p.x = states_[ii](0);
    p.y = states_[ii](1);
    p.z = states_[ii](2);

    const std_msgs::ColorRGBA c = Colormap(ii);

    // Handle 'spheres' marker.
    spheres.points.push_back(p);
    spheres.colors.push_back(c);

    // Handle 'lines' marker.
    lines.points.push_back(p);
    lines.colors.push_back(c);
  }

  // Publish markers. Only publish 'lines' if more than one point in trajectory.
  pub.publish(spheres);

  if (Size() > 1)
    pub.publish(lines);
}

// Compute the color (on a red-blue colormap) at a particular index.
std_msgs::ColorRGBA Trajectory::Colormap(size_t index) const {
  std_msgs::ColorRGBA color;

#ifdef ENABLE_DEBUG_MESSAGES
  if (index >= Size()) {
    ROS_ERROR("Tried to compute colormap for out-of-bounds index.");

    color.r = 0.0;
    color.g = 0.0;
    color.b = 0.0;
    color.a = 1.0;
    return color;
  }
#endif

  // Compute total trajectory timescale.
  double total_time = Time();

  const double kSmallNumber = 1e-8;
  if (total_time < kSmallNumber) {
    ROS_WARN("Total time length of trajectory is too small.");
    total_time = kSmallNumber;
  }

  color.r = times_[index] / total_time;
  color.g = 0.0;
  color.b = 1.0 - color.r;
  color.a = 0.6;

  return color;
}
