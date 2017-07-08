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
// Defines the ValueFunction class.
//
///////////////////////////////////////////////////////////////////////////////

#include <meta_planner/value_function.h>

// Factory method. Use this instead of the constructor.
// Note that this class is const-only, which means that once it is
// instantiated it can never be changed.
ValueFunction::ConstPtr ValueFunction::Create(const std::string& file_name) {
  ValueFunction::ConstPtr ptr(new ValueFunction(file_name));
  return ptr;
}

// Constructor. Don't use this. Use the factory method instead.
ValueFunction::ValueFunction(const std::string& file_name) {
  initialized_ = Load(file_name);
}

// Linearly interpolate to get the value at a particular state.
double ValueFunction::Value(const VectorXd& state) const {
  // TODO!
  return 0.0;
}

// Linearly interpolate to get the gradient at a particular state.
VectorXd ValueFunction::Gradient(const VectorXd& state) const {
  // TODO!
  return VectorXd::Zero(1);
}

// Load from file. Returns whether or not it was successful.
bool ValueFunction::Load(const std::string& file_name) {
  // Open the file.
  mat_t* matfp = Mat_Open(file_name.c_str(), MAT_ACC_RDONLY);
  if (matfp == NULL) {
    ROS_ERROR("Could not open file: %s.", file_name);
    return false;
  }

  // Read the specified variable from this file.
  const std::string grid_min = "grid_min";
  matvar_t* grid_min_mat = Mat_VarRead(matfp, grid_min.c_str());
  if (grid_min_mat == NULL) {
    ROS_ERROR("Could not read variable: %s.", grid_min);
    return false;
  }

  const std::string grid_max = "grid_max";
  matvar_t* grid_max_mat = Mat_VarRead(matfp, grid_max.c_str());
  if (grid_max_mat == NULL) {
    ROS_ERROR("Could not read variable: %s.", grid_max);
    return false;
  }

  const std::string grid_N = "grid_N";
  matvar_t* grid_N_mat = Mat_VarRead(matfp, grid_N.c_str());
  if (grid_N_mat == NULL) {
    ROS_ERROR("Could not read variable: %s.", grid_N);
    return false;
  }

  const std::string data = "data";
  matvar_t* data_mat = Mat_VarRead(matfp, data.c_str());
  if (data_mat == NULL) {
    ROS_ERROR("Could not read variable: %s.", data);
    return false;
  }


  // Populate class variables.
  if (grid_min_mat->data_type != MAT_T_DOUBLE) {
    ROS_ERROR("%s: Wrong type of data.",grid_min);
    return false;
  }

  size_t num_elements = grid_min_mat->nbytes/grid_min_mat->data_size; 
  for (size_t ii = 0; ii < num_elements; ii++){
    lower_.push_back(static_cast<double*>(grid_min_mat->data)[ii]);
  }



  if (grid_max_mat->data_type != MAT_T_DOUBLE) {
    ROS_ERROR("%s: Wrong type of data.",grid_max);
    return false;
  }

  num_elements = grid_max_mat->nbytes/grid_max_mat->data_size; 
  for (size_t ii = 0; ii < num_elements; ii++){
    upper_.push_back(static_cast<double*>(grid_max_mat->data)[ii]);
  }



  if (grid_N_mat->data_type != MAT_T_UINT64) {
    ROS_ERROR("%s: Wrong type of data.",grid_N);
    return false;
  }

  num_elements = grid_N_mat->nbytes/grid_N_mat->data_size; 
  for (size_t ii = 0; ii < num_elements; ii++){
    num_voxels_.push_back(static_cast<size_t*>(grid_N_mat->data)[ii]);
  }



  if (data_mat->data_type != MAT_T_DOUBLE) {
    ROS_ERROR("%s: Wrong type of data.",data);
    return false;
  }

  num_elements = data_mat->nbytes/data_mat->data_size; 
  for (size_t ii = 0; ii < num_elements; ii++){
    num_voxels_.push_back(static_cast<double*>(data_mat->data)[ii]);
  }


  //Free memory and close file
  Mat_VarFree(grid_min_mat);
  Mat_VarFree(grid_max_mat);
  Mat_VarFree(grid_N_mat);
  Mat_VarFree(data_mat);
  Mat_Close(matfp);

  return true;
}
