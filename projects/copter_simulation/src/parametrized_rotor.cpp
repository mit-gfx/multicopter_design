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
#include "parametrized_rotor.h"
#include "geometry.h"
#include "parametrized_connector.h"

namespace copter_simulation {

int ParametrizedRotor::next_rotor_id_ = 0;

ParametrizedRotor::ParametrizedRotor()
  : parent_connector_(NULL), reference_rotor_() {}

void ParametrizedRotor::Initialize(const TriRigidBody& reference_rigid_body,
  const Eigen::VectorXd& reference_parameter,
  const ParametrizedConnector* parent_connector,
  const std::string& measurement_file,
  const double prop_height, const bool is_ccw,
  const bool flip_propeller) {
  // Parameters: x, y, z, dir.x, dir.y, dir.z.
  // Currently dir.x, dir.y, dir.z are all fixed.
  assert(reference_parameter.size() == 6u);
  ParametrizedShape::Initialize("rotor_" + std::to_string(next_rotor_id_),
    reference_rigid_body, reference_parameter);
  ++next_rotor_id_;
  
  parent_connector_ = parent_connector;
  reference_rotor_.Initialize(measurement_file, prop_height,
    RotorPosition(reference_parameter),
    RotorDirection(reference_parameter), is_ccw, flip_propeller);
}

void ParametrizedRotor::AnalyzeConstraint() const {
  const std::string& rotor_name = name();
  const Eigen::Vector3d rotor_ref_dir = RotorDirection(reference_parameter());
  // Fix the direction.
  for (int i = 0; i < 3; ++i) {
    Constraint dir_constraint;
    dir_constraint.Initialize(rotor_ref_dir(i), rotor_ref_dir(i));
    dir_constraint.AddVariable(rotor_name, 3 + i, 1.0);
    AddConstraint(dir_constraint);
  }

  // Fix the relative location between the motor and connector.
  assert(parent_connector_);
  const std::string& connector_name = parent_connector_->name();
  // Since fixed constraints are symmetric, we add it only if neighbor_name >
  // connector_name.
  if (rotor_name.compare(connector_name) > 0) return;
  // Fix the relative location between rotor and connector.
  const Eigen::Vector3d rotor_ref_pos = RotorPosition(reference_parameter());
  const Eigen::Vector3d connector_ref_pos =
    parent_connector_->reference_parameter();
  const Eigen::Vector3d offset = connector_ref_pos - rotor_ref_pos;
  for (int i = 0; i < 3; ++i) {
    Constraint fixed_constraint;
    fixed_constraint.Initialize(offset(i), offset(i));
    fixed_constraint.AddVariable(connector_name, i, 1.0);
    fixed_constraint.AddVariable(rotor_name, i, -1.0);
    AddConstraint(fixed_constraint);
  }
}

const TriRigidBody ParametrizedRotor::operator()(
  const Eigen::VectorXd& parameter) const {
  assert(static_cast<int>(parameter.size()) == NumOfParameters());
  const Eigen::Vector3d pos = RotorPosition(parameter);
  const Eigen::Vector3d dir = RotorDirection(parameter);
  TriRigidBody new_shape = reference_rigid_body();
  new_shape.Translate(-RotorPosition(reference_parameter()));
  Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(
    RotorDirection(reference_parameter()),
    RotorDirection(parameter));
  new_shape.Rotate(q.matrix());
  new_shape.Translate(RotorPosition(parameter));
  return new_shape;
}

const Eigen::Matrix4d ParametrizedRotor::Transform(
  const Eigen::VectorXd& parameter) const {
  assert(static_cast<int>(parameter.size()) == NumOfParameters());
  const Eigen::Vector3f ref_pos = RotorPosition(
    reference_parameter()).cast<float>();
  const Eigen::Vector3f ref_dir = RotorDirection(
    reference_parameter()).cast<float>();
  const Eigen::Vector3f new_pos = RotorPosition(parameter).cast<float>();
  const Eigen::Vector3f new_dir = RotorDirection(parameter).cast<float>();
  const Eigen::Quaternionf q = Eigen::Quaternionf::FromTwoVectors(
    ref_dir, new_dir);
  return (opengl_viewer::Translate(new_pos) *
    opengl_viewer::Rotate(q) *
    opengl_viewer::Translate(-ref_pos)).cast<double>();
}

const Rotor ParametrizedRotor::NewRotor(
  const Eigen::VectorXd& parameter) const {
  Rotor new_rotor = reference_rotor_;
  new_rotor.set_position(RotorPosition(parameter));
  new_rotor.set_direction(RotorDirection(parameter));
  return new_rotor;
}

const Eigen::Vector3d ParametrizedRotor::RotorPosition(
  const Eigen::VectorXd& parameter) const {
  assert(static_cast<int>(parameter.size()) == NumOfParameters());
  return parameter.head(3);
}

const Eigen::Vector3d ParametrizedRotor::RotorDirection(
  const Eigen::VectorXd& parameter) const {
  assert(static_cast<int>(parameter.size()) == NumOfParameters());
  return parameter.tail(3);
}

} // copter_simulation