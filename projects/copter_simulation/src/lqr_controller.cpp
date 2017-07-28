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
#include "lqr_controller.h"
#include <iostream>

namespace copter_simulation {

// Borrowed from Drake. Based on the Matrix Sign Function method outlined in
// this paper:
// http://www.engr.iupui.edu/~skoskie/ECE684/Riccati_algorithms.pdf
static bool lqr(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
  const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R,
  Eigen::MatrixXd& K, Eigen::MatrixXd& S) {
  const int n = static_cast<int>(A.rows());
  Eigen::LLT<Eigen::MatrixXd> R_cholesky(R);

  Eigen::MatrixXd H(2 * n, 2 * n);
  H << A, B * R_cholesky.solve(B.transpose()), Q, -A.transpose();

  Eigen::MatrixXd Z = H;
  Eigen::MatrixXd Z_old;

  // These could be options.
  const double tolerance = 1e-9;
  const double max_iterations = 100;

  double relative_norm;
  int iteration = 0;

  const double p = -1.0 / static_cast<double>(Z.rows());
  do {
    Z_old = Z;
    // R. Byers. Solving the algebraic Riccati equation with the matrix sign
    // function. Linear Algebra Appl., 85:267–279, 1987.
    // Added determinant scaling to improve convergence (converges in rough half
    // the iterations with this).
    Z *= std::pow(std::abs(Z.determinant()), p);
    Z = Z - 0.5 * (Z - Z.inverse());
    relative_norm = (Z - Z_old).norm();
    ++iteration;
  } while (iteration < max_iterations && relative_norm > tolerance);
  if (std::isnan(relative_norm) || relative_norm > tolerance) return false;

  Eigen::MatrixXd W11 = Z.block(0, 0, n, n);
  Eigen::MatrixXd W12 = Z.block(0, n, n, n);
  Eigen::MatrixXd W21 = Z.block(n, 0, n, n);
  Eigen::MatrixXd W22 = Z.block(n, n, n, n);

  Eigen::MatrixXd lhs(2 * n, n);
  Eigen::MatrixXd rhs(2 * n, n);
  Eigen::MatrixXd eye = Eigen::MatrixXd::Identity(n, n);
  lhs << W12, W22 + eye;
  rhs << W11 + eye, W21;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(lhs,
    Eigen::ComputeThinU | Eigen::ComputeThinV);
  S = svd.solve(rhs);
  K = R_cholesky.solve(B.transpose() * S);
  return true;
}

LqrController::LqrController()
  : Controller(), K_(Eigen::MatrixXd::Zero(0, 0)),
  x0_(Eigen::VectorXd::Zero(0)),
  u0_(Eigen::VectorXd::Zero(0)) {}

void LqrController::Initialize(const Eigen::MatrixXd& A,
  const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R,
  const Eigen::VectorXd& x0, const Eigen::VectorXd& u0) {
  // Check the dimensions:
  // A, Q and R should be square.
  assert(A.rows() == A.cols());
  assert(Q.rows() == Q.cols());
  assert(R.rows() == R.cols());
  // A, B and Q should have the same rows.
  assert(A.rows() == B.rows());
  assert(A.rows() == Q.rows());
  assert(A.cols() == x0.size());
  // B and R should have the same columns.
  assert(B.cols() == R.cols());
  assert(B.cols() == u0.size());

  const int input_dim = static_cast<int>(x0.size());
  const int output_dim = static_cast<int>(u0.size());
  Controller::Initialize(input_dim, output_dim);

  x0_ = x0;
  u0_ = u0;
  // Compute K from A, B, Q, R.
  Eigen::MatrixXd dummy_S;
  if (!lqr(A, B, Q, R, K_, dummy_S)) {
    std::cout << "Error: Failed to solve lqr." << std::endl;
    exit(0);
  }
}

const Eigen::VectorXd LqrController::Output(const Eigen::VectorXd& input,
  const Eigen::VectorXd& target, const double dt) {
  assert(input.size() == target.size());
  assert(static_cast<int>(input.size()) == input_dim());
  return -K_ * (input - x0_) + u0_;
}

} // copter_simulation
