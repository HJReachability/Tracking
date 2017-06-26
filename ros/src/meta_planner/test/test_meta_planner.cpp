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
// Unit tests for the meta_planner package.
//
///////////////////////////////////////////////////////////////////////////////

#include <meta_planner/box.h>
#include <meta_planner/trajectory.h>
#include <meta_planner/rrt_connect.h>
#include <meta_planner/types.h>

#include <gtest/gtest.h>

// Test the RrtConnect class. Make sure it can plan a trajectory in an empty
// unit box betweeen the two corners.
TEST(RrtConnect, TestUnitBox) {
  const double kVelocity = 1.0;
  const size_t kAmbientDimension = 3;

  // Pick start and stop states.
  const VectorXd start = VectorXd::Zero(kAmbientDimension);
  const VectorXd stop = VectorXd::Ones(kAmbientDimension);

  // Create unit box environment.
  Box box(kAmbientDimension);
  box.SetBounds(VectorXd::Zero(kAmbientDimension),
                VectorXd::Ones(kAmbientDimension));

  // Plan.
  const RrtConnect planner(kVelocity);
  const Trajectory traj = planner.Plan(start, stop, box);

  // Check that start and stop states match.
  const double kSmallNumber = 1e-8;
  EXPECT_LE((start - traj.FirstState()).norm(), kSmallNumber);
  EXPECT_LE((stop - traj.LastState()).norm(), kSmallNumber);

  traj.Print("Computed trajectory:");

  // Check that the time spent on the trajectory is at least the minimum
  // time to go along a straight line.
  EXPECT_GE(traj.Time(), (start - stop).norm() / kVelocity);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
