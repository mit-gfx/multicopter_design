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
#ifndef _COPTER_SIMULATION_ROTOR_H_
#define _COPTER_SIMULATION_ROTOR_H_

#include "Eigen/Dense"

namespace copter_simulation {

class Rotor {
public:
  // Both position and direction are defined in the body frame of the copter.
  Rotor();

  void Initialize(const double torque_to_thrust_ratio,
    const double torque_to_current_ratio,
    const double propeller_thrust_coefficient,
    const double max_thrust, const double propeller_height,
    const Eigen::Vector3d& position, const Eigen::Vector3d& direction,
    const bool is_ccw, const bool flip_propeller);
  void Initialize(const std::string& motor_measurement,
    const double propeller_height, const Eigen::Vector3d& position,
    const Eigen::Vector3d& direction, const bool is_ccw,
    const bool flip_propeller);

  // Update the phase of the propeller.
  void Advance(const double thrust, const double dt);

  // Return the thrust and torque in the body frame.
  const Eigen::Vector3d Thrust(const double thrust) const;
  // Torque = ThrustTorque + SpinningTorque.
  const Eigen::Vector3d Torque(const double thrust) const;
  const Eigen::Vector3d ThrustTorque(const double thrust) const;
  const Eigen::Vector3d SpinningTorque(const double thrust) const;

  const double torque_to_thrust_ratio() const {
    return torque_to_thrust_ratio_;
  }
  const double torque_to_current_ratio() const {
    return torque_to_current_ratio_;
  }
  const double propeller_thrust_coefficient() const {
    return propeller_thrust_coefficient_;
  }
  const double propeller_height() const {
    return propeller_height_;
  }
  const double propeller_phase() const {
    return propeller_phase_;
  }
  const double propeller_speed() const {
    return propeller_speed_;
  }
  const Eigen::Vector3d position() const { return position_; }
  void set_position(const Eigen::Vector3d& position) {
    position_ = position;
  }
  const Eigen::Vector3d direction() const { return direction_; }
  void set_direction(const Eigen::Vector3d& direction) {
    direction_ = direction;
  }
  const bool is_ccw() const { return is_ccw_; }
  void set_is_ccw(const bool is_ccw) {
    is_ccw_ = is_ccw;
  }
  const bool flip_propeller() const { return flip_propeller_; }

private:
  // torque / thrust. Usually in the order of 0.01 ~ 0.1.
  double torque_to_thrust_ratio_;
  // Ideally, torque is proportional to current.
  double torque_to_current_ratio_;
  // It is known that thrust = C * w^2, where w is the spinning rate (in rad/s)
  // and C is a coefficient depends on propeller geometry. We will call C
  // propeller_thrust_coefficient_. Our experiment shows C is around 1e-5.
  double propeller_thrust_coefficient_;
  // The maximal thrust this rotor can provide.
  double max_thrust_;

  // Propeller related members.
  // The position of the propeller = position_ + direction_ * propeller_height.
  double propeller_height_;
  // In radians.
  double propeller_phase_;
  double propeller_speed_;

  // Both position and direction are defined in the body frame. direction is
  // always the up direction of the motor, no matter whether it is installed
  // upsidedown.
  Eigen::Vector3d position_;
  Eigen::Vector3d direction_;
  // Clockwise or counterclockwise, determined by righthand rule and direction.
  bool is_ccw_;
  // flip_propeller is mainly used by Y6, where the bottom propellers are
  // installed differently.
  bool flip_propeller_;
};

} // copter_simulation

#endif  // _COPTER_SIMULATION_ROTOR_H_