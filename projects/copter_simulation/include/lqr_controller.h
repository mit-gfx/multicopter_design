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
#ifndef _COPTER_SIMULATION_LQR_CONTROLLER_H_
#define _COPTER_SIMULATION_LQR_CONTROLLER_H_

#include "controller.h"

namespace copter_simulation {

class LqrController : public Controller {
public:
  LqrController();

  void Initialize(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
    const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R,
    const Eigen::VectorXd& x0, const Eigen::VectorXd& u0);

  // Ignore target because it is set in Initialize.
  const Eigen::VectorXd Output(const Eigen::VectorXd& input,
    const Eigen::VectorXd& target, const double dt);

  const Eigen::VectorXd x0() const { return x0_; }

private:
  Eigen::MatrixXd K_;
  Eigen::VectorXd x0_, u0_;
};

} // copter_simulation.

#endif  // _COPTER_SIMULATION_LQR_CONTROLLER_H_