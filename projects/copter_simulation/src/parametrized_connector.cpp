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
#include "parametrized_connector.h"
#include "geometry.h"
#include "parametrized_rotor.h"

namespace copter_simulation {

int ParametrizedConnector::next_connector_id_ = 0;

ParametrizedConnector::ParametrizedConnector()
  : ParametrizedShape(), dir_(Eigen::Vector3d::Zero()),
  parent_tube_(NULL), neighbors_(0) {}

void ParametrizedConnector::Initialize(
  const TriRigidBody& reference_rigid_body,
  const Eigen::VectorXd& reference_parameter, const Eigen::Vector3d& dir,
  const ParametrizedTube* parent_tube,
  const std::vector<ParametrizedConnectorNeighbor> neighbors) {
  // Position: x, y, z.
  assert(reference_parameter.size() == 3u);
  assert(neighbors.size() == 4u);
  ParametrizedShape::Initialize(
    "connector_" + std::to_string(next_connector_id_), reference_rigid_body,
    reference_parameter);
  ++next_connector_id_;

  dir_ = dir.normalized();
  parent_tube_ = parent_tube;
  neighbors_ = neighbors;
}

void ParametrizedConnector::AnalyzeConstraint() const {
  // If any of its neighbors shows fixed, then we are done.
  for (const auto& neighbor : neighbors_) {
    if (neighbor.type == kNeighborIsPlate) {
      for (int i = 0; i < 3; ++i) {
        Constraint fixed_constraint;
        fixed_constraint.Initialize(reference_parameter()(i),
          reference_parameter()(i));
        fixed_constraint.AddVariable(name(), i, 1.0);
        AddConstraint(fixed_constraint);
      }
      return;
    }
  }

  // If any of its neighbors is another connector or a motor, then their
  // relative location is fixed.
  const std::string& connector_name = name();
  const Eigen::Vector3d pos = reference_parameter();
  for (const auto& neighbor : neighbors_) {
    if (neighbor.type == kFree) continue;
    std::string neighbor_name("");
    Eigen::Vector3d neighbor_pos(0, 0, 0);
    if (neighbor.type == kNeighborIsConnector) {
      const ParametrizedConnector* neighbor_connector =
        reinterpret_cast<const ParametrizedConnector*>(neighbor.neighbor);
      neighbor_name = neighbor_connector->name();
      neighbor_pos = neighbor_connector->reference_parameter();
    } else if (neighbor.type == kNeighborIsRotor) {
      const ParametrizedRotor* neighbor_rotor =
        reinterpret_cast<const ParametrizedRotor*>(neighbor.neighbor);
      neighbor_name = neighbor_rotor->name();
      neighbor_pos = neighbor_rotor->reference_rotor().position();
    } else {
      // Should not happen.
      assert(false);
    }
    // Since fixed constraints are symmetric, we add it only if neighbor_name >
    // connector_name.
    if (neighbor_name.compare(connector_name) <= 0) continue;
    const Eigen::Vector3d diff = pos - neighbor_pos;
    for (int i = 0; i < 3; ++i) {
      Constraint fixed_constraint;
      fixed_constraint.Initialize(diff(i), diff(i));
      fixed_constraint.AddVariable(connector_name, i, 1.0);
      fixed_constraint.AddVariable(neighbor_name, i, -1.0);
      AddConstraint(fixed_constraint);
    }
  }
}

const TriRigidBody ParametrizedConnector::operator()(
  const Eigen::VectorXd& parameter) const {
  assert(static_cast<int>(parameter.size()) == NumOfParameters());
  const Eigen::Vector3d reference_pos = reference_parameter();
  const Eigen::Vector3d new_pos = parameter;
  TriRigidBody new_shape = reference_rigid_body();
  new_shape.Translate(new_pos - reference_pos);
  return new_shape;
}

const Eigen::Matrix4d ParametrizedConnector::Transform(
  const Eigen::VectorXd& parameter) const {
  assert(static_cast<int>(parameter.size()) == NumOfParameters());
  const Eigen::Vector3d reference_pos = reference_parameter();
  const Eigen::Vector3d new_pos = parameter;
  return opengl_viewer::Translate(
    (new_pos - reference_pos).cast<float>()).cast<double>();
}

} // copter_simulation