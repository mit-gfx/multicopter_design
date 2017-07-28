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
#ifndef _COPTER_SIMULATION_NLOPT_FUNCTION_H_
#define _COPTER_SIMULATION_NLOPT_FUNCTION_H_

#include <math.h>
#include <vector>
#include "Eigen/Dense"

namespace copter_simulation {

class NloptFunction {
public:
  NloptFunction();
  virtual ~NloptFunction() {}

  void Initialize(const std::vector<std::string>& x_name,
    const Eigen::VectorXd& lower_bound,
    const Eigen::VectorXd& upper_bound);
  void Initialize(const std::vector<std::string>& x_name,
    const double lower_bound, const double upper_bound);

  // Given x, return the function value and fill out the jacobian.
  virtual const Eigen::VectorXd Evaluate(const Eigen::VectorXd& x,
    Eigen::MatrixXd& jacobian) = 0;
  virtual const double Evaluate(const int row_num, const Eigen::VectorXd& x,
    Eigen::VectorXd& gradient);
  const bool HasFiniteLowerBound(const int row_num);
  const bool HasFiniteUpperBound(const int row_num);
  const bool HasEqualBound(const int row_num);

  const std::vector<std::string>& x_name() const { return x_name_; }
  const std::string x_name(const int i) const;
  const int x_dim() const { return x_dim_; }
  const int f_dim() const { return f_dim_; }
  const Eigen::VectorXd lower_bound() const { return lower_bound_; }
  const Eigen::VectorXd upper_bound() const { return upper_bound_; }

  static const double Infinity() { return HUGE_VAL; }

private:
  std::vector<std::string> x_name_;
  int x_dim_, f_dim_;
  Eigen::VectorXd lower_bound_, upper_bound_;
};

} // copter_simulation

#endif  // _COPTER_SIMULATION_NLOPT_FUNCTION_H_