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
#ifndef _COPTER_SIMULATION_NLOPT_LINEAR_FUNCTION_H_
#define _COPTER_SIMULATION_NLOPT_LINEAR_FUNCTION_H_

#include "nlopt_function.h"

namespace copter_simulation {

// Linear constraints: lower <= Ax + b <= upper 
class NloptLinearFunction : public NloptFunction {
public:
  NloptLinearFunction();

  void Initialize(const std::vector<std::string>& x_name,
    const Eigen::VectorXd& lower_bound, const Eigen::VectorXd& upper_bound,
    const Eigen::MatrixXd& A, const Eigen::VectorXd& b);
  void Initialize(const std::string& x_name, const int num,
    const Eigen::VectorXd& lower_bound, const Eigen::VectorXd& upper_bound,
    const Eigen::MatrixXd& A, const Eigen::VectorXd& b);

  const Eigen::VectorXd Evaluate(const Eigen::VectorXd& x,
    Eigen::MatrixXd& jacobian);
  const double Evaluate(const int row_num, const Eigen::VectorXd& x,
    Eigen::VectorXd& gradient);

private:
  Eigen::MatrixXd A_;
  Eigen::VectorXd b_;
};

} // copter_simulation

#endif  // _COPTER_SIMULATION_NLOPT_LINEAR_FUNCTION_H_