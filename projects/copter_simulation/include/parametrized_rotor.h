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
#ifndef _COPTER_SIMULATION_PARAMETRIZED_ROTOR_H_
#define _COPTER_SIMULATION_PARAMETRIZED_ROTOR_H_

#include "parametrized_shape.h"
#include "rotor.h"

namespace copter_simulation {

class ParametrizedConnector;

class ParametrizedRotor : public ParametrizedShape {
public:
  ParametrizedRotor();

  void Initialize(const TriRigidBody& reference_rigid_body,
    const Eigen::VectorXd& reference_parameter,
    const ParametrizedConnector* parent_connector,
    const std::string& measurement_file,
    const double prop_height, const bool is_ccw,
    const bool flip_propeller);

  void AnalyzeConstraint() const;
  const TriRigidBody operator()(const Eigen::VectorXd& parameter) const;
  const Eigen::Matrix4d Transform(const Eigen::VectorXd& parameter) const;
  const Rotor NewRotor(const Eigen::VectorXd& parameter) const;
  const Rotor& reference_rotor() const { return reference_rotor_; }

  static const int NumOfParametrizedRotor() { return next_rotor_id_; }

private:
  const Eigen::Vector3d RotorPosition(const Eigen::VectorXd& parameter) const;
  const Eigen::Vector3d RotorDirection(const Eigen::VectorXd& parameter) const;

  const ParametrizedConnector* parent_connector_;

  Rotor reference_rotor_;

  static int next_rotor_id_;
};

} // copter_simulation

#endif  // _COPTER_SIMULATION_PARAMETRIZED_ROTOR_H_