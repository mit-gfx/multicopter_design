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
#include "rotor.h"
#include <fstream>
// Expose PI.
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <math.h>
#include <vector>

namespace copter_simulation {

Rotor::Rotor() : torque_to_thrust_ratio_(0.0),
  torque_to_current_ratio_(0.0),
  propeller_thrust_coefficient_(1e-5),
  max_thrust_(5.0),
  propeller_height_(0.0),
  propeller_phase_(0.0),
  propeller_speed_(0.0),
  position_(Eigen::Vector3d::Zero()),
  direction_(-Eigen::Vector3d::UnitZ()),
  is_ccw_(false),
  flip_propeller_(false) {}

void Rotor::Initialize(const double torque_to_thrust_ratio,
  const double torque_to_current_ratio,
  const double propeller_thrust_coefficient,
  const double max_thrust, const double propeller_height,
  const Eigen::Vector3d& position, const Eigen::Vector3d& direction,
  const bool is_ccw, const bool flip_propeller) {
  torque_to_thrust_ratio_ = torque_to_thrust_ratio;
  torque_to_current_ratio_ = torque_to_current_ratio;
  propeller_thrust_coefficient_ = propeller_thrust_coefficient;
  max_thrust_ = max_thrust;
  propeller_height_ = propeller_height;
  propeller_phase_ = 0.0;
  position_ = position;
  direction_ = direction.normalized();
  is_ccw_ = is_ccw;
  flip_propeller_ = flip_propeller;
}

void Rotor::Initialize(const std::string& motor_measurement,
  const double propeller_height, const Eigen::Vector3d& position,
  const Eigen::Vector3d& direction, const bool is_ccw,
  const bool flip_propeller) {
  // Full name of the file.
  const std::string project_source_folder = PROJECT_SOURCE_DIR;
  std::ifstream input(project_source_folder + "/resources/measurement/"
    + motor_measurement);
  // There are four columns in this file: PWM, torque, force, RPM.
  std::vector<double> torque_samples(0), force_samples(0), speed_samples(0),
    current_samples(0);
  while (input.good()) {
    double dummy, torque, force, speed, current;
    input >> dummy >> torque >> force >> speed >> current;
    // Convert RMP to radian/second.
    speed *= M_PI / 30.0;
    torque_samples.push_back(std::abs(torque));
    force_samples.push_back(std::abs(force));
    speed_samples.push_back(std::abs(speed));
    current_samples.push_back(std::abs(current));
  }
  const int sample_num = static_cast<int>(force_samples.size());
  const Eigen::VectorXd force = Eigen::Map<Eigen::VectorXd>(
    force_samples.data(), sample_num),
    torque = Eigen::Map<Eigen::VectorXd>(torque_samples.data(), sample_num),
    speed = Eigen::Map<Eigen::VectorXd>(speed_samples.data(), sample_num),
    current = Eigen::Map<Eigen::VectorXd>(current_samples.data(), sample_num);
  const Eigen::VectorXd speed_sq = speed.array().square();

  // Estimate the parameters.
  const double torque_to_thrust_ratio = force.dot(torque) / force.squaredNorm();
  const double torque_to_current_ratio =
    current.dot(torque) / current.squaredNorm();
  const double propeller_thrust_coefficient =
    speed_sq.dot(force) / speed_sq.squaredNorm();
  Initialize(torque_to_thrust_ratio, torque_to_current_ratio,
    propeller_thrust_coefficient, force.maxCoeff(), propeller_height, position,
    direction, is_ccw, flip_propeller);
}

void Rotor::Advance(const double thrust, const double dt) {
  // Clamp between 0 and max_thrust.
  const double clamped_thrust = (thrust < 0 ? 0
    : (thrust > max_thrust_ ? max_thrust_ : thrust));
  propeller_speed_ = (is_ccw_ ? 1.0 : -1.0)
    * std::sqrt(clamped_thrust / propeller_thrust_coefficient_);
  propeller_phase_ += propeller_speed_ * dt;
}

const Eigen::Vector3d Rotor::Thrust(const double thrust) const {
  const double clamped_thrust = (thrust < 0 ? 0
    : (thrust > max_thrust_ ? max_thrust_ : thrust));
  return clamped_thrust * direction_ * (flip_propeller_ ? -1.0 : 1.0);
}

const Eigen::Vector3d Rotor::Torque(const double thrust) const {
  return ThrustTorque(thrust) + SpinningTorque(thrust);
}

const Eigen::Vector3d Rotor::ThrustTorque(const double thrust) const {
  const Eigen::Vector3d thrust_force = Thrust(thrust);
  return position_.cross(thrust_force);
}

const Eigen::Vector3d Rotor::SpinningTorque(const double thrust) const {
  const Eigen::Vector3d thrust_force = Thrust(thrust);
  return torque_to_thrust_ratio_ * thrust_force
    * (is_ccw_ == flip_propeller_ ? 1.0 : -1.0);
}

} // copter_simulation
