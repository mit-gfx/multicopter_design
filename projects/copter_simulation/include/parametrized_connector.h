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
#ifndef _COPTER_SIMULATION_PARAMETRIZED_CONNECTOR_H_
#define _COPTER_SIMULATION_PARAMETRIZED_CONNECTOR_H_

#include "parametrized_shape.h"

namespace copter_simulation {

enum NeighborLocation { kLeft = 0, kRight, kTop, kBottom };
enum NeighborType { kFree = 0, kNeighborIsConnector, kNeighborIsRotor,
  kNeighborIsPlate };

class ParametrizedConnector;

struct ParametrizedConnectorNeighbor {
public:
  ParametrizedConnectorNeighbor()
    : location(kLeft), type(kFree), neighbor(NULL) {}

  ParametrizedConnectorNeighbor(const NeighborLocation location,
    const NeighborType type, const void* neighbor)
    : location(location), type(type), neighbor(neighbor) {}

  NeighborLocation location;
  NeighborType type;
  const void* neighbor;
};

class ParametrizedTube;

class ParametrizedConnector : public ParametrizedShape {
public:
  ParametrizedConnector();

  void Initialize(const TriRigidBody& reference_rigid_body,
    const Eigen::VectorXd& reference_parameter, const Eigen::Vector3d& dir,
    const ParametrizedTube* parent_tube,
    const std::vector<ParametrizedConnectorNeighbor> neighbors);
  void AnalyzeConstraint() const;

  const TriRigidBody operator()(const Eigen::VectorXd& parameter) const;
  const Eigen::Matrix4d Transform(const Eigen::VectorXd& parameter) const;

  static const int NumOfParametrizedConnector() { return next_connector_id_; }

private:
  Eigen::Vector3d dir_;
  const ParametrizedTube* parent_tube_;
  std::vector<ParametrizedConnectorNeighbor> neighbors_;

  static int next_connector_id_;
};

} // copter_simulation

#endif  // _COPTER_SIMULATION_PARAMETRIZED_CONNECTOR_H_