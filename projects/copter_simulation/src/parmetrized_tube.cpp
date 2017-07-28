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
#include "parametrized_tube.h"
#include "geometry.h"
#include "inertial_sensor.h"
#include "parametrized_connector.h"

namespace copter_simulation {

int ParametrizedTube::next_tube_id_ = 0;

ParametrizedTube::ParametrizedTube()
  : ParametrizedShape(), dir_(Eigen::Vector3d::Zero()), connectors_(0) {}

void ParametrizedTube::Initialize(const TriRigidBody& reference_rigid_body,
  const Eigen::VectorXd& reference_parameter,
  const Eigen::Vector3d& dir,
  const std::vector<ParametrizedConnector*>& connectors) {
  // Position: x, y, z.
  // Length: s.
  assert(reference_parameter.size() == 4u);
  // We request each tube should have at least one connector.
  assert(connectors.size() >= 1u);
  ParametrizedShape::Initialize("tube_" + std::to_string(next_tube_id_),
    reference_rigid_body, reference_parameter);
  ++next_tube_id_;
  dir_ = dir.normalized();
  connectors_ = connectors;
}

void ParametrizedTube::AnalyzeConstraint() const {
  // Build constraints.
  const std::string& tube_name = name();
  const Eigen::Vector3d tube_center = reference_parameter().head(3);
  const double tube_length = reference_parameter()(3);
  // The length should be positive.
  Constraint length_constraint;
  length_constraint.Initialize(0.0, MaxTubeLength());
  length_constraint.AddVariable(tube_name, 3, 1.0);
  AddConstraint(length_constraint);

  // Now we add constraints between length and connectors.
  // First, all connectors should be aligned with the tube.
  const int connector_num = static_cast<int>(connectors_.size());
  const Eigen::Matrix3d D = SkewMatrix(dir_);
  for (const auto& connector : connectors_) {
    const std::string& connector_name = connector->name();
    // (tube_pos - connector_pos).cross(dir) = 0.
    for (int j = 0; j < 3; ++j) {
      Constraint fixed_constraint;
      fixed_constraint.Initialize(0, 0);
      for (int k = 0; k < 3; ++k) {
        if (D(j, k) == 0) continue;
        fixed_constraint.AddVariable(tube_name, k, D(j, k));
        fixed_constraint.AddVariable(connector_name, k, -D(j, k));
      }
      AddConstraint(fixed_constraint);
    }
  }
  // Next, the length of the tube limits how far a connector can be from the
  // center of mass of the tube.
  for (const auto& connector : connectors_) {
    const std::string& connector_name = connector->name();
    const Eigen::Vector3d connector_pos = connector->reference_parameter();
    // -s/2 <= (connector_pos - tube_pos).dot(dir_) <= s/2.
    Constraint left_constraint, right_constraint;
    left_constraint.Initialize(0, Constraint::Infinity());
    right_constraint.Initialize(-Constraint::Infinity(), 0);
    left_constraint.AddVariable(tube_name, 3, 0.5);
    right_constraint.AddVariable(tube_name, 3, -0.5);
    for (int i = 0; i < 3; ++i) {
      left_constraint.AddVariable(tube_name, i, -dir_(i));
      left_constraint.AddVariable(connector_name, i, dir_(i));
      right_constraint.AddVariable(tube_name, i, -dir_(i));
      right_constraint.AddVariable(connector_name, i, dir_(i));
    }
    AddConstraint(left_constraint);
    AddConstraint(right_constraint);
  }
}

const TriRigidBody ParametrizedTube::operator()(
  const Eigen::VectorXd& parameter) const {
  assert(static_cast<int>(parameter.size()) == NumOfParameters());
  const Eigen::Vector3d ref_pos = reference_parameter().head(3),
    new_pos = parameter.head(3);
  const double ref_length = reference_parameter()(3),
    new_length = parameter(3);
  const Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(
    dir_, Eigen::Vector3d::UnitX());

  TriRigidBody new_shape = reference_rigid_body();
  new_shape.Translate(-ref_pos);
  new_shape.Rotate(q.matrix());
  new_shape.Scale(new_length / ref_length, 1.0, 1.0);
  new_shape.Rotate(q.conjugate().matrix());
  new_shape.Translate(new_pos);

  return new_shape;
}

const Eigen::Matrix4d ParametrizedTube::Transform(
  const Eigen::VectorXd& parameter) const {
  assert(static_cast<int>(parameter.size()) == NumOfParameters());
  const Eigen::Vector4f ref_param = reference_parameter().cast<float>();
  const Eigen::Vector4f param = parameter.cast<float>();
  const Eigen::Vector3f ref_pos = ref_param.head(3), new_pos = param.head(3);
  const float ref_length = ref_param(3), new_length = param(3);
  const Eigen::Quaternionf q = Eigen::Quaternionf::FromTwoVectors(
    dir_.cast<float>(), Eigen::Vector3f::UnitX());

  return (opengl_viewer::Translate(new_pos) *
    opengl_viewer::Rotate(q.conjugate()) *
    opengl_viewer::Scale(new_length / ref_length, 1.0f, 1.0f) *
    opengl_viewer::Rotate(q) *
    opengl_viewer::Translate(-ref_pos)).cast<double>();
}

} // copter_simulation