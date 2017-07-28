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
#include "nlopt_function.h"

namespace copter_simulation {

NloptFunction::NloptFunction()
  : x_name_(0), x_dim_(0), f_dim_(0),
  lower_bound_(Eigen::VectorXd::Zero(0)),
  upper_bound_(Eigen::VectorXd::Zero(0)) {}

void NloptFunction::Initialize(const std::vector<std::string>& x_name,
  const Eigen::VectorXd& lower_bound, const Eigen::VectorXd& upper_bound) {
  assert(lower_bound.size() == upper_bound.size());
  x_name_ = x_name;
  x_dim_ = static_cast<int>(x_name.size());
  f_dim_ = static_cast<int>(lower_bound.size());
  lower_bound_ = lower_bound;
  upper_bound_ = upper_bound;
}

void NloptFunction::Initialize(const std::vector<std::string>& x_name,
  const double lower_bound, const double upper_bound) {
  Initialize(x_name, Eigen::VectorXd::Constant(1, lower_bound),
    Eigen::VectorXd::Constant(1, upper_bound));
}

const double NloptFunction::Evaluate(const int row_num,
  const Eigen::VectorXd& x, Eigen::VectorXd& gradient) {
  assert(row_num >= 0 && row_num < f_dim_);
  assert(static_cast<int>(x.size()) == x_dim_);
  Eigen::MatrixXd jacobian;
  if (gradient.size()) {
    assert(gradient.size() == x.size());
    jacobian = Eigen::MatrixXd::Zero(f_dim_, x_dim_);
  } else {
    jacobian = Eigen::MatrixXd::Zero(0, 0);
  }
  const Eigen::VectorXd value = Evaluate(x, jacobian);
  if (gradient.size()) gradient = jacobian.row(row_num);
  return value(row_num);
}

const bool NloptFunction::HasFiniteLowerBound(const int row_num) {
  assert(row_num >= 0 && row_num < f_dim_);
  return lower_bound_[row_num] != -Infinity();
}

const bool NloptFunction::HasFiniteUpperBound(const int row_num) {
  assert(row_num >= 0 && row_num < f_dim_);
  return upper_bound_[row_num] != Infinity();
}

const bool NloptFunction::HasEqualBound(const int row_num) {
  assert(row_num >= 0 && row_num < f_dim_);
  return lower_bound_[row_num] == upper_bound_[row_num];
}

const std::string NloptFunction::x_name(const int i) const {
  assert(i >= 0 && i < x_dim_);
  return x_name_[i];
}

} // copter_simulation