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

#ifndef META_PLANNER_BOX_H
#define META_PLANNER_BOX_H

#include <meta_planner/environment.h>

#include <ros/ros.h>

class Box : public Environment {
public:
  virtual ~Box() {}
  Box(size_t dimension);

  // Inherited from Environment, but can be overriden by child classes.
  virtual VectorXd Sample();

  // Inherited from Environment, but can be overriden by child classes.
  // Returns true if the state is a valid configuration.
  virtual bool IsValid(const VectorXd& state) const;

  // Set bounds in each dimension.
  void SetBounds(const VectorXd& lower, const VectorXd& upper);

  // Get the dimension and upper/lower bounds as const references.
  size_t Dimension() const { return dimension_; }
  const VectorXd& LowerBounds() const { return lower_; }
  const VectorXd& UpperBounds() const { return upper_; }

private:
  const size_t dimension_;
  VectorXd lower_;
  VectorXd upper_;
};

#endif
