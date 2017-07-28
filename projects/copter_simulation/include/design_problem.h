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
#ifndef _COPTER_SIMULATION_DESIGN_PROBLEM_H_
#define _COPTER_SIMULATION_DESIGN_PROBLEM_H_

#include "Eigen/Dense"

namespace copter_simulation {

// This struct encodes the constraints:
//   Ax = b, Cx <= d.
// or equivalently, let x = x0 + Ey where E spans the null space of A:
//   Fy <= g.
// where F = CE, g = d - Cx0.
// Now consider the SVD of F: F = U\SigmaV' = CE.
//   U\Sigma V'y <= g.
// So if we define z := V'y, we can have:
//   x = x0 + EVz.
//   U\Sigma z <= g.
// Next we split z into z = [z1; z2] where z1 are the entries with nonzero
// singular values, so z2 is completely free:
//   x = x0 + EV[z1; z2].
//   U_1\Sigma_1 z1 <= g.
// We further define w1 = \Sigma_1 z1, so
//   x = x0 + EV[\Sigma_1^{-1}w1; z2].
//   U_1w1 <= g.
// So to conclude, our reduced parameters are represented as:
//   x = x0 + Ey.
//   Fy_1 <= g.
// where columns in F are orthogonal.

class DesignProblem {
public:
  DesignProblem();

  void Initialize(const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
    const Eigen::MatrixXd& C, const Eigen::VectorXd& d,
    const Eigen::VectorXd& x0);

  const Eigen::VectorXd ToFullParameter(const Eigen::VectorXd& reduced) const;
  const Eigen::VectorXd ToReducedParameter(const Eigen::VectorXd& full) const;

  const bool IsFullParameterFeasible(const Eigen::VectorXd& full) const;
  const bool IsReducedParameterFeasible(const Eigen::VectorXd& reduced) const;

  const Eigen::MatrixXd A() const { return A_; }
  const Eigen::VectorXd b() const { return b_; }
  const Eigen::MatrixXd C() const { return C_; }
  const Eigen::VectorXd d() const { return d_; }
  const Eigen::MatrixXd E() const { return E_; }
  const Eigen::MatrixXd F() const { return F_; }
  const Eigen::VectorXd x0() const { return x0_; }
  const Eigen::VectorXd g() const { return g_; }

  const int bounded_parameter_num() const { return bounded_parameter_num_; }
  const int free_parameter_num() const { return free_parameter_num_; }
  const int NumOfFullParameter() const { return static_cast<int>(x0_.size()); }
  const int NumOfReducedParameter() const {
    return bounded_parameter_num() + free_parameter_num();
  }

private:
  // Matrices and vectors for intuitive parameters.
  // Ax = b, Cx <= d.
  Eigen::MatrixXd A_, C_;
  Eigen::VectorXd b_, d_;

  // Matrices and vectors for reduced parameters.
  Eigen::MatrixXd E_, E_inv_, F_;
  Eigen::VectorXd x0_, g_;
  int bounded_parameter_num_, free_parameter_num_;
};

} // copter_simulation

#endif  // _COPTER_SIMULATION_DESIGN_PROBLEM_H_