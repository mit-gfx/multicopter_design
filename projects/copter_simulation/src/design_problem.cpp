// @ Copyright 2016 Massachusetts Institute of Technology.
// 
// This program is free software; you can redistribute it and / or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
// MA 02110-1301, USA.
#include "design_problem.h"
#include <assert.h>

namespace copter_simulation {

DesignProblem::DesignProblem()
  : A_(0, 0), C_(0, 0), b_(0), d_(0),
  E_(0, 0), E_inv_(0, 0), F_(0, 0), x0_(0), g_(0),
  bounded_parameter_num_(0), free_parameter_num_(0) {}

void DesignProblem::Initialize(const Eigen::MatrixXd& A,
  const Eigen::VectorXd& b, const Eigen::MatrixXd& C,
  const Eigen::VectorXd& d, const Eigen::VectorXd& x0) {
  A_ = A; b_ = b;
  C_ = C; d_ = d; x0_ = x0;
  assert(IsFullParameterFeasible(x0));

  Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(A_);
  E_ = lu_decomp.kernel();
  F_ = C_ * E_;
  g_ = d_ - C_ * x0_;
  // SVD on F.
  Eigen::JacobiSVD<Eigen::MatrixXd> svd;
  svd.compute(F_, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::MatrixXd U = svd.matrixU(), V = svd.matrixV();
  Eigen::VectorXd scale = svd.singularValues();
  const int F_rank = static_cast<int>(svd.rank());
  // x = x0 + Ey, U\Sigma V'y <= g.
  E_ = E_ * V;
  // x = x0 + Ey, U\Sigma y <= g.
  bounded_parameter_num_ = F_rank;
  free_parameter_num_ = static_cast<int>(E_.cols()) - F_rank;
  F_ = U.leftCols(F_rank);
  const Eigen::MatrixXd E_left = E_.leftCols(F_rank);
  E_.leftCols(F_rank) = E_left
    * scale.head(F_rank).cwiseInverse().asDiagonal();
  // Suppress all tiny elements.
  // TODO: better ways to increase numerical stabilities?
  const double eps = 1e-6;
  for (auto i = 0; i < E_.rows(); ++i)
    for (auto j = 0; j < E_.cols(); ++j) {
      if (std::abs(E_(i, j)) < eps) E_(i, j) = 0;
    }
  for (auto i = 0; i < F_.rows(); ++i)
    for (auto j = 0; j < F_.cols(); ++j) {
      if (std::abs(F_(i, j)) < eps) F_(i, j) = 0;
    }
  for (auto i = 0; i < g_.size(); ++i) {
    if (std::abs(g_(i)) < eps) g_(i) = 0;
  }

  // x = x0 + Ey, Fy <= g.
  assert(IsReducedParameterFeasible(
    Eigen::VectorXd::Zero(NumOfReducedParameter())));
  // Now compute the pseudo-inverse of E.
  svd.compute(E_, Eigen::ComputeThinU | Eigen::ComputeThinV);
  U = svd.matrixU(); scale = svd.singularValues(); V = svd.matrixV();
  // U'(x - x0) = \sigma V' y
  const int E_rank = static_cast<int>(svd.rank());
  assert(E_rank == static_cast<int>(E_.cols()));
  E_inv_ = V * scale.cwiseInverse().asDiagonal()
    * U.leftCols(E_rank).transpose();
  for (auto i = 0; i < E_inv_.rows(); ++i)
    for (auto j = 0; j < E_inv_.cols(); ++j) {
      if (std::abs(E_inv_(i, j)) < eps) E_inv_(i, j) = 0;
    }
}

const Eigen::VectorXd DesignProblem::ToFullParameter(
  const Eigen::VectorXd& reduced) const {
  return x0_ + E_ * reduced;
}

const Eigen::VectorXd DesignProblem::ToReducedParameter(
  const Eigen::VectorXd& full) const {
  return E_inv_ * (full - x0_);
}

const bool DesignProblem::IsFullParameterFeasible(
  const Eigen::VectorXd& full) const {
  assert(full.size() == x0_.size());
  return (A_ * full - b_).cwiseAbs().maxCoeff() < 1e-6
    && (C_ * full - d_).maxCoeff() < 1e-6;
}

const bool DesignProblem::IsReducedParameterFeasible(
  const Eigen::VectorXd& reduced) const {
  assert(reduced.size() == E_.cols());
  return ((F_ * reduced.head(bounded_parameter_num_) - g_).maxCoeff() < 1e-6);
}

} // copter_simulation