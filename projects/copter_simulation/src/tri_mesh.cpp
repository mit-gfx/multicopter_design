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
#include "tri_mesh.h"
#include <fstream>
#include "opengl_viewer.h"

namespace copter_simulation {

TriMesh::TriMesh()
  : vertex_(Eigen::Matrix3Xd::Zero(3, 0)),
  element_(Eigen::Matrix3Xi::Zero(3, 0)) {}

void TriMesh::Initialize(const Eigen::Matrix3Xd& vertex,
  const Eigen::Matrix3Xi& element) {
  vertex_ = vertex;
  element_ = element;
}

void TriMesh::Initialize(const std::string& file_name) {
  ReadFromFile(file_name);
}

void TriMesh::Initialize(const std::vector<TriMesh>& meshes) {
  const int mesh_num = static_cast<int>(meshes.size());
  if (mesh_num == 0) {
    vertex_ = Eigen::Matrix3Xd::Zero(3, 0);
    element_ = Eigen::Matrix3Xi::Zero(3, 0);
  }
  std::vector<int> vertex_num(mesh_num, 0), element_num(mesh_num, 0);
  for (int i = 1; i < mesh_num; ++i) {
    vertex_num[i] = vertex_num[i - 1] + meshes[i - 1].NumOfVertex();
    element_num[i] = element_num[i - 1] + meshes[i - 1].NumOfElement();
  }
  vertex_ = Eigen::Matrix3Xd::Zero(3,
    vertex_num.back() + meshes.back().NumOfVertex());
  element_ = Eigen::Matrix3Xi::Zero(3,
    element_num.back() + meshes.back().NumOfElement());
  for (int i = 0; i < mesh_num; ++i) {
    vertex_.middleCols(vertex_num[i], meshes[i].NumOfVertex())
      = meshes[i].vertex();
    element_.middleCols(element_num[i], meshes[i].NumOfElement())
      = meshes[i].element().array() + vertex_num[i];
  }
}

const Eigen::Vector3d TriMesh::vertex(const int index) const {
  assert(index >= 0 && index < static_cast<int>(vertex_.cols()));
  return vertex_.col(index);
}

const Eigen::Vector3i TriMesh::element(const int index) const {
  assert(index >= 0 && index < static_cast<int>(element_.cols()));
  return element_.col(index);
}

const Eigen::Matrix3d TriMesh::vertex_in_element(const int index) const {
  assert(index >= 0 && index < static_cast<int>(element_.cols()));
  Eigen::Matrix3d v = Eigen::Matrix3d::Zero();
  for (int i = 0; i < 3; ++i) {
    v.col(i) = vertex_.col(element_(i, index));
  }
  return v;
}

const int TriMesh::NumOfVertex() const {
  return static_cast<int>(vertex_.cols());
}

const int TriMesh::NumOfElement() const {
  return static_cast<int>(element_.cols());
}

void TriMesh::WriteToFile(const std::string& file_name) const {
  std::ofstream fout;
  fout.open(file_name);
  const int vertex_number = NumOfVertex();
  for (int i = 0; i < vertex_number; ++i) {
    fout << "v " << vertex_(0, i) << " "
      << vertex_(1, i) << " "
      << vertex_(2, i) << std::endl;
  }
  const int element_number = NumOfElement();
  for (int i = 0; i < element_number; ++i) {
    fout << "f " << element_(0, i) + 1 << " "
      << element_(1, i) + 1 << " "
      << element_(2, i) + 1 << std::endl;
  }
  fout.close();
}

void TriMesh::ReadFromFile(const std::string& file_name) {
  Eigen::Matrix3Xf vertex;
  Eigen::Matrix2Xf dummy_uv;
  opengl_viewer::ReadFromObjFile(file_name, vertex, element_, dummy_uv);
  vertex_ = vertex.cast<double>();
}

void TriMesh::Translate(const Eigen::Vector3d& v) {
  vertex_.colwise() += v;
}

void TriMesh::Rotate(const Eigen::Quaterniond& R) {
  vertex_ = R.matrix() * vertex_;
}

void TriMesh::Rotate(const double angle, const Eigen::Vector3d& axis) {
  vertex_ = Eigen::AngleAxisd(angle, axis.normalized()).matrix() * vertex_;
}

void TriMesh::Rotate(const Eigen::Matrix3d& R) {
  vertex_ = R * vertex_;
}

void TriMesh::Scale(const Eigen::Vector3d& S) {
  vertex_ = S.asDiagonal() * vertex_;
}

void TriMesh::Transform(const Eigen::Matrix4d& H) {
  Eigen::Matrix4Xd homo_vertex = (H.leftCols(3) * vertex_).colwise()
    + H.col(3);
  vertex_ = homo_vertex.topRows(3).array() /
    homo_vertex.row(3).replicate(3, 1).array();
}

const TriMesh TriMeshCuboid(const Eigen::Vector3d& pmin,
  const Eigen::Vector3d& size) {
  Eigen::MatrixXd vertex(8, 3);
  vertex << 0, 0, 0,
    0, 0, 1,
    0, 1, 0,
    0, 1, 1,
    1, 0, 0,
    1, 0, 1,
    1, 1, 0,
    1, 1, 1;
  Eigen::MatrixXi element(12, 3);
  element << 0, 2, 6,
    0, 6, 4,
    1, 5, 7,
    1, 7, 3,
    7, 5, 4,
    7, 4, 6,
    3, 7, 6,
    3, 6, 2,
    1, 3, 2,
    1, 2, 0,
    1, 0, 4,
    1, 4, 5;
  TriMesh cube;
  cube.Initialize(vertex.transpose(), element.transpose());
  cube.Scale(size);
  cube.Translate(pmin);
  return cube;
}

} // copter_simulation