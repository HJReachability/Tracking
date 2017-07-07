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

#include <meta_planner/value_function.h>
#include <meta_planner/box.h>
#include <meta_planner/trajectory.h>
#include <meta_planner/ompl_planner.h>
#include <meta_planner/types.h>

#include <matio.h>
#include <stdio.h>
#include <algorithm>
#include <gtest/gtest.h>

// Test that MATIO can read in a small file correctly.
TEST(Matio, TestRead) {
  const std::string file_name =
    std::string(PRECOMPUTATION_DIR) + std::string("test.mat");
  const std::string var_name = "x";

  // Open a file pointer to this file.
  mat_t* matfp = Mat_Open(file_name.c_str(), MAT_ACC_RDONLY);
  ASSERT_TRUE(matfp != NULL);

  // Read the specified variable from this file.
  matvar_t* matvar = Mat_VarRead(matfp, var_name.c_str());
  ASSERT_TRUE(matvar != NULL);

  // Check content.
  ASSERT_EQ(matvar->rank, 2);
  ASSERT_EQ(matvar->dims[0], 1);
  ASSERT_EQ(matvar->dims[1], 3);
  ASSERT_EQ(matvar->isComplex, 0);
  ASSERT_EQ(matvar->data_type, MAT_T_DOUBLE);
  ASSERT_EQ(matvar->class_type, MAT_C_DOUBLE);

  const double (&data)[1][3] =
    *static_cast<const double (*)[1][3]>(matvar->data);

  for (size_t jj = 0; jj < 3; jj++)
    EXPECT_EQ(data[0][jj], static_cast<double>(jj + 1));

  // Free memory and close the file.
  Mat_VarFree(matvar);
  Mat_Close(matfp);
}

// Test the OmplPlanner class. Make sure it can plan a trajectory in an empty
// unit box betweeen the two corners.
TEST(OmplPlanner, TestUnitBox) {
  const double kVelocity = 1.0;
  const size_t kAmbientDimension = 3;

  // Pick start and stop states.
  const VectorXd start = VectorXd::Zero(kAmbientDimension);
  const VectorXd stop = VectorXd::Ones(kAmbientDimension);

  // Create unit box environment.
  const Box::Ptr box = Box::Create(kAmbientDimension);
  box->SetBounds(VectorXd::Zero(kAmbientDimension),
                 VectorXd::Ones(kAmbientDimension));

  std::vector<size_t> dimensions(kAmbientDimension);
  std::iota(dimensions.begin(), dimensions.end(), 0);

  // Create a nullptr for the ValueFunction.
  const ValueFunction::ConstPtr null_value(NULL);

  // Plan.
  const OmplPlanner<og::RRTConnect> planner(
    null_value, box, dimensions, kVelocity);
  const Trajectory traj = planner.Plan(start, stop);

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
