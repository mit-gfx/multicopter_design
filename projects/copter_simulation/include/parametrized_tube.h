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
//
//
// The parameters in ParametrizedTube are:
// - (x, y, z): the position of the center of mass, in the world frame.
// - s: the length of the tube.
#ifndef _COPTER_SIMULATION_PARAMETRIZED_TUBE_H_
#define _COPTER_SIMULATION_PARAMETRIZED_TUBE_H_

#include "parametrized_shape.h"

namespace copter_simulation {

class ParametrizedConnector;

class ParametrizedTube : public ParametrizedShape {
public:
  ParametrizedTube();

  void Initialize(const TriRigidBody& reference_rigid_body,
    const Eigen::VectorXd& reference_parameter,
    const Eigen::Vector3d& dir,
    const std::vector<ParametrizedConnector*>& connectors);

  void AnalyzeConstraint() const;
  const TriRigidBody operator()(const Eigen::VectorXd& parameter) const;
  const Eigen::Matrix4d Transform(const Eigen::VectorXd& parameter) const;

  static const int NumOfParametrizedTube() { return next_tube_id_; }
  // 5 meters should be long enough?
  static const double MaxTubeLength() { return 5.0; }

private:
  Eigen::Vector3d dir_;
  // It is guaranteed that each tube has at least two connectors, and they are
  // sorted along dir_.
  std::vector<ParametrizedConnector*> connectors_;

  static int next_tube_id_;
};

} // copter_simulation

#endif  // _COPTER_SIMULATION_PARAMETRIZED_TUBE_H_