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
#include "pid_controller.h"

namespace copter_simulation {

PidController::PidController()
  : Controller(), p_(0.0), i_(0.0), d_(0.0),
  last_error_(Eigen::VectorXd::Zero(0)), sum_(Eigen::VectorXd::Zero(0)) {}

void PidController::Initialize(const int input_dim, const int output_dim,
  const double p, const double i, const double d) {
  Controller::Initialize(input_dim, output_dim);
  p_ = p; i_ = i; d_ = d;
  last_error_ = sum_ = Eigen::VectorXd::Zero(input_dim);
}

const Eigen::VectorXd PidController::Output(const Eigen::VectorXd& input,
  const Eigen::VectorXd& target, const double dt) {
  // Check inputs.
  assert(input.size() == target.size());
  assert(static_cast<int>(input.size()) == input_dim());

  const Eigen::VectorXd error = target - input;
  Eigen::VectorXd output = p_ * error;
  // I term.
  sum_ += error * dt;
  output += i_ * sum_;
  // D term.
  if (dt > 0.0) {
    output += d_ * (error - last_error_) / dt;
  }
  last_error_ = error;

  // Check output.
  assert(output.size() == output_dim());
  return output;
}

} // copter_simulation