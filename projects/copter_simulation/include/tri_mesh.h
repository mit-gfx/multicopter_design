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
// A simple triangle mesh class. Each column in vertex_/face_ represents a
// single point/face. Support obj file formats by using Syoyo Fujita's
// tinyobjloader:
// https://github.com/syoyo/tinyobjloader
#ifndef _COPTER_SIMULATION_TRI_MESH_H_
#define _COPTER_SIMULATION_TRI_MESH_H_

#include <vector>
#include "Eigen/Dense"

namespace copter_simulation {

class TriMesh {
public:
  TriMesh();
  void Initialize(const Eigen::Matrix3Xd& vertex,
    const Eigen::Matrix3Xi& element);
  void Initialize(const std::string& file_name);
  // Combine multiple meshes into a single one.
  void Initialize(const std::vector<TriMesh>& meshes);

  void WriteToFile(const std::string& file_name) const;

  const Eigen::Matrix3Xd& vertex() const { return vertex_; }
  const Eigen::Vector3d vertex(const int index) const;
  const Eigen::Matrix3Xi& element() const { return element_; }
  const Eigen::Vector3i element(const int index) const;

  // Return the three vertices in a triangle face.
  const Eigen::Matrix3d vertex_in_element(const int index) const;
  const int NumOfVertex() const;
  const int NumOfElement() const;

  void Translate(const Eigen::Vector3d& v);
  void Rotate(const Eigen::Quaterniond& R);
  // The angle parameter is in radians.
  void Rotate(const double angle, const Eigen::Vector3d& axis);
  void Rotate(const Eigen::Matrix3d& R);
  void Scale(const Eigen::Vector3d& S);
  // Apply a homogeneous transform.
  void Transform(const Eigen::Matrix4d& H);
  
private:
  void ReadFromFile(const std::string& file_name);

  Eigen::Matrix3Xd vertex_;
  Eigen::Matrix3Xi element_;
};

const TriMesh TriMeshCuboid(const Eigen::Vector3d& pmin,
  const Eigen::Vector3d& size);

} // copter_simulation

#endif // _COPTER_SIMULATION_TRI_MESH_H_