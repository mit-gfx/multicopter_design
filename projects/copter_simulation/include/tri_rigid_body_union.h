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
#ifndef _COPTER_SIMULATION_TRI_RIGID_BODY_UNION_H_
#define _COPTER_SIMULATION_TRI_RIGID_BODY_UNION_H_

#include <vector>
#include "tri_rigid_body.h"

namespace copter_simulation {

struct ObjectInfo {
public:
  ObjectInfo()
    : mesh(), density(0.0) {}
  ObjectInfo(const TriMesh& mesh, const double density)
    : mesh(mesh), density(density) {}

  TriMesh mesh;
  double density;
};

struct InstanceInfo {
public:
  InstanceInfo()
    : index(0), translation(0, 0, 0), rotation(Eigen::Matrix3d::Identity()),
      scale(1.0, 1.0, 1.0) {}
  InstanceInfo(const int index, const Eigen::Vector3d& translation,
    const Eigen::Matrix3d& rotation, const Eigen::Vector3d& scale)
    : index(index), translation(translation), rotation(rotation),
      scale(scale) {}

  int index;
  Eigen::Vector3d translation;
  Eigen::Matrix3d rotation;
  Eigen::Vector3d scale;
};

class TriRigidBodyUnion : public RigidBody {
public:
  TriRigidBodyUnion();

  void Initialize(const std::vector<double>& densities,
    const std::vector<TriMesh>& world_space_meshes);
  // Initialize with instancing. This is useful when you reuse many objects.
  // It is equivalent to the function above in the following way:
  // std::vector<double> flattened_densities(0);
  // std::vector<TriMesh> world_space_meshes(0);
  // for (auto i = 0; i < index.size(); ++i) {
  //   const int id = instance_info[i].index;
  //   flattened_densities.push_back(model_space_object[id].density);
  //   world_space_meshes.push_back(instance_info[i].rotation
  //     * instance_info[i].scale.asDiagonal()
  //     * model_space_object[id].mesh + instance_info[i].translation);
  // }
  // Initialize(flattened_densities, world_space_meshes);
  void Initialize(const std::vector<ObjectInfo>& model_space_object,
    const std::vector<InstanceInfo>& instance_info);
  void Initialize(const std::vector<TriRigidBody>& world_space_object);

  const int NumOfTriRigidBody() const;
  const TriMesh WorldSpaceMesh(const int i) const;
  const TriMesh BodySpaceMesh(const int i) const;

private:
  std::vector<TriMesh> body_space_meshes_;
};

} // copter_simulation

#endif  // _COPTER_SIMULATION_TRI_RIGID_BODY_UNION_H_