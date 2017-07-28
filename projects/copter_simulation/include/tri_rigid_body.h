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
// Build a rigid body object from a triangle mesh. The object is assumed to be
// solid, and its surface is represented by a triangle mesh whose vertices are
// defined in the world frame. The origin of the body frame is defined at the
// center of mass of the object, and the axes are parallel to the world frame.
// We require the order of 3 vertices of a triangle is counterclockwise, more
// specifically, this means (v1 - v0).cross(v2 - v1).normalized() gives the
// outward normal of the triangle. We also require the triangle mesh is water-
// tight, otherwise its volume is not well defined.
#ifndef _COPTER_SIMULATION_TRI_RIGID_BODY_H_
#define _COPTER_SIMULATION_TRI_RIGID_BODY_H_

#include "rigid_body.h"
#include "tri_mesh.h"

namespace copter_simulation {

class TriRigidBody : public RigidBody {
public:
  TriRigidBody();
  void Initialize(const double density, const TriMesh& world_space_mesh);

  const TriMesh& body_space_mesh() const {
    return body_space_mesh_;
  }
  const TriMesh WorldSpaceMesh() const {
    TriMesh world_space_mesh;
    world_space_mesh.Initialize(
      BodyPointToWorldPoint(body_space_mesh_.vertex()),
      body_space_mesh_.element());
    return world_space_mesh;
  }

  void Translate(const double x, const double y, const double z);
  void Translate(const Eigen::Vector3d& translation);
  void Rotate(const Eigen::Matrix3d& rotation);
  void Scale(const double s);
  void Scale(const double x, const double y, const double z);
  void Scale(const Eigen::Vector3d& scale);
  // Decompose transform into translate + rotate + scale + rotate, then apply
  // functions above.
  void Transform(const Eigen::Matrix4d& transform);

private:
  TriMesh body_space_mesh_;
};

} // copter_simulation

#endif  // COPTER_SIMULATION_TRI_RIGID_BODY_H_
