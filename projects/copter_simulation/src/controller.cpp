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
#include "controller.h"

namespace copter_simulation {

Controller::Controller()
  : input_dim_(0), output_dim_(0) {}

void Controller::Initialize(const int input_dim, const int output_dim) {
  input_dim_ = input_dim;
  output_dim_ = output_dim;
}

const Eigen::VectorXd DoubleToVectorXd(const double value) {
  return Eigen::VectorXd::Constant(1, value);
}

const double VectorXdToDouble(const Eigen::VectorXd& value) {
  return value(0);
}

} // copter_simulation