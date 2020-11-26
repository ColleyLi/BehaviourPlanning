/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/

#pragma once

#include <qpOASES.hpp>

#include <memory>
#include <vector>

#include "modules/planning/math/smoothing_spline/spline_2d.h"
#include "modules/planning/math/smoothing_spline/spline_2d_constraint.h"
#include "modules/planning/math/smoothing_spline/spline_2d_kernel.h"
#include "modules/planning/math/smoothing_spline/spline_2d_solver.h"

namespace apollo {
namespace planning {

class ActiveSetSpline2dSolver final : public Spline2dSolver {
 public:
  ActiveSetSpline2dSolver(const std::vector<double>& t_knots,
                          const uint32_t order);

  void Reset(const std::vector<double>& t_knots, const uint32_t order) override;

  // customize setup
  Spline2dConstraint* mutable_constraint() override;
  Spline2dKernel* mutable_kernel() override;
  Spline2d* mutable_spline() override;

  // solve
  bool Solve() override;

  // extract
  const Spline2d& spline() const override;

 private:
  std::unique_ptr<::qpOASES::SQProblem> sqp_solver_;

  int last_num_constraint_ = 0;
  int last_num_param_ = 0;
  bool last_problem_success_ = false;
};

}  // namespace planning
}  // namespace apollo
