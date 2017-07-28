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
#include "nlopt_linear_function.h"

namespace copter_simulation {

NloptLinearFunction::NloptLinearFunction()
  : NloptFunction(), A_(0, 0), b_(0) {}

void NloptLinearFunction::Initialize(const std::vector<std::string>& x_name,
  const Eigen::VectorXd& lower_bound, const Eigen::VectorXd& upper_bound,
  const Eigen::MatrixXd& A, const Eigen::VectorXd& b) {
  NloptFunction::Initialize(x_name, lower_bound, upper_bound);
  assert(A.rows() == lower_bound.size());
  assert(A.cols() == x_name.size());
  assert(A.rows() == b.size());
  A_ = A; b_ = b;
}

void NloptLinearFunction::Initialize(const std::string& x_name, const int num,
  const Eigen::VectorXd& lower_bound, const Eigen::VectorXd& upper_bound,
  const Eigen::MatrixXd& A, const Eigen::VectorXd& b) {
  std::vector<std::string> x_names(num, x_name + "_");
  for (int i = 0; i < num; ++i) {
    x_names[i] += std::to_string(i);
  }
  Initialize(x_names, lower_bound, upper_bound, A, b);
}

const Eigen::VectorXd NloptLinearFunction::Evaluate(const Eigen::VectorXd& x,
  Eigen::MatrixXd& jacobian) {
  assert(x.size() == A_.cols());
  if (jacobian.size()) {
    assert(A_.rows() == jacobian.rows());
    assert(A_.cols() == jacobian.cols());
    jacobian = A_;
  }
  return A_ * x + b_;
}

const double NloptLinearFunction::Evaluate(const int row_num,
  const Eigen::VectorXd& x, Eigen::VectorXd& gradient) {
  assert(row_num >= 0 && row_num < static_cast<int>(A_.rows()));
  assert(x.size() == A_.cols());
  if (gradient.size()) {
    assert(gradient.size() == x.size());
    gradient = A_.row(row_num);
  }
  return A_.row(row_num).dot(x) + b_(row_num);
}

} // copter_simulation