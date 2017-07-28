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
#include "tri_rigid_body_union.h"

namespace copter_simulation {

TriRigidBodyUnion::TriRigidBodyUnion()
  : RigidBody(), body_space_meshes_(0) {}

void TriRigidBodyUnion::Initialize(const std::vector<double>& densities,
  const std::vector<TriMesh>& world_space_meshes) {
  assert(densities.size() == world_space_meshes.size());
  const int mesh_num = static_cast<int>(world_space_meshes.size());
  assert(mesh_num > 0);
  std::vector<ObjectInfo> model_space_object(mesh_num, ObjectInfo());
  for (int i = 0; i < mesh_num; ++i) {
    model_space_object[i].mesh = world_space_meshes[i];
    model_space_object[i].density = densities[i];
  }
  std::vector<InstanceInfo> instance_info(mesh_num, InstanceInfo());
  for (int i = 0; i < mesh_num; ++i) instance_info[i].index = i;
  Initialize(model_space_object, instance_info);
}

void TriRigidBodyUnion::Initialize(
  const std::vector<ObjectInfo>& model_space_object,
  const std::vector<InstanceInfo>& instance_info) {
  // Initialize all unique objects.
  const int unique_mesh_num = static_cast<int>(model_space_object.size());
  std::vector<TriRigidBody> unique_rigid_bodies(
    unique_mesh_num, TriRigidBody());
  for (int i = 0; i < unique_mesh_num; ++i) {
    unique_rigid_bodies[i].Initialize(model_space_object[i].density,
      model_space_object[i].mesh);
  }

  // Instancing.
  const int mesh_num = static_cast<int>(instance_info.size());
  std::vector<TriRigidBody> rigid_bodies(mesh_num, TriRigidBody());
  for (int i = 0; i < mesh_num; ++i) {
    const int id = instance_info[i].index;
    const Eigen::Vector3d v = instance_info[i].translation;
    const Eigen::Matrix3d R = instance_info[i].rotation;
    const Eigen::Vector3d s = instance_info[i].scale;
    rigid_bodies[i] = unique_rigid_bodies[id];
    rigid_bodies[i].Scale(s);
    rigid_bodies[i].Rotate(R);
    rigid_bodies[i].Translate(v);
  }

  Initialize(rigid_bodies);
}

void TriRigidBodyUnion::Initialize(
  const std::vector<TriRigidBody>& world_space_object) {
  velocity_.setZero();
  acceleration_.setZero();
  external_force_.setZero();
  angular_velocity_.setZero();
  external_torque_.setZero();
  orientation_ = Eigen::Quaterniond(Eigen::Matrix3d::Identity());

  // Compute the mass and the center of mass.
  mass_ = 0.0;
  position_.setZero();
  for (const TriRigidBody& tri_rigid_body : world_space_object) {
    mass_ += tri_rigid_body.mass();
    position_ += tri_rigid_body.mass() * tri_rigid_body.position();
  }
  position_ /= mass_;
  mass_inv_ = 1.0 / mass_;

  // Compute the inertia tensor.
  inertia_body_.setZero();
  for (const TriRigidBody& tri_rigid_body : world_space_object) {
    // Use the parallel axis theorem to compute the inertia tensor. See
    // wikipedia for details:
    // https://www.wikiwand.com/en/Parallel_axis_theorem.
    const Eigen::Vector3d offset = position_ - tri_rigid_body.position();
    const Eigen::Matrix3d skew_matrix_offset = (Eigen::Matrix3d()
      << 0, -offset.z(), offset.y(),
      offset.z(), 0, -offset.x(),
      -offset.y(), offset.x(), 0).finished();
    inertia_body_ += tri_rigid_body.inertia_body()
      - tri_rigid_body.mass() * skew_matrix_offset * skew_matrix_offset;
  }
  inertia_body_inv_ = inertia_body_.inverse();

  // Compute the body space mesh.
  body_space_meshes_.clear();
  for (auto i = 0; i < world_space_object.size(); ++i) {
    TriMesh world_space_mesh = world_space_object[i].WorldSpaceMesh();
    TriMesh component;
    component.Initialize(WorldPointToBodyPoint(world_space_mesh.vertex()),
      world_space_mesh.element());
    body_space_meshes_.push_back(component);
  }
}

const int TriRigidBodyUnion::NumOfTriRigidBody() const {
  return static_cast<int>(body_space_meshes_.size());
}

const TriMesh TriRigidBodyUnion::WorldSpaceMesh(const int i) const {
  assert(i >= 0 && i < NumOfTriRigidBody());
  TriMesh component;
  component.Initialize(BodyPointToWorldPoint(body_space_meshes_[i].vertex()),
    body_space_meshes_[i].element());
  return component;
}

const TriMesh TriRigidBodyUnion::BodySpaceMesh(const int i) const {
  assert(i >= 0 && i < NumOfTriRigidBody());
  return body_space_meshes_[i];
}

} // copter_simulation