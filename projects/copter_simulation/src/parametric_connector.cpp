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
#include "parametric_connector.h"
#include <iostream>

namespace copter_simulation {

const TriMesh ParametricTubeMount(const Eigen::Vector3d& normal) {
  Eigen::Vector3d unit_normal = normal.normalized();
  // Flip if necessary.
  if (unit_normal.z() > 0.0) unit_normal *= -1.0;
  // Read mesh.
  const std::string project_source_folder = PROJECT_SOURCE_DIR;
  TriMesh tube_mount;
  tube_mount.Initialize(project_source_folder +
    "/resources/mesh/tube_mount.obj");
  Eigen::Matrix3Xd vertex = tube_mount.vertex();

  // Find the height of the ring we want to change.
  const double largest_y = vertex.row(1).maxCoeff();
  const double eps = 1e-6;
  double ring_height = 1e20;
  const int vertex_num = tube_mount.NumOfVertex();
  for (int i = 0; i < vertex_num; ++i) {
    const Eigen::Vector3d v = vertex.col(i);
    if (v.y() >= largest_y - eps && v.z() <= ring_height) {
      ring_height = v.z();
    }
  }
  // Now collect all vertices on the ring and build the local frame.
  std::vector<int> ring_index(0);
  double ring_min_x = 1e20, ring_max_x = -1e20;
  double ring_min_y = 1e20, ring_max_y = -1e20;
  for (int i = 0; i < vertex_num; ++i) {
    const Eigen::Vector3d v = vertex.col(i);
    if (v.z() <= ring_height + eps && v.z() >= ring_height - eps) {
      ring_index.push_back(i);
      if (v.x() > ring_max_x) ring_max_x = v.x();
      if (v.x() < ring_min_x) ring_min_x = v.x();
      if (v.y() > ring_max_y) ring_max_y = v.y();
      if (v.y() < ring_min_y) ring_min_y = v.y();
    }
  }
  // Collect vertices at the top surface.
  const double top_height = vertex.row(2).minCoeff();
  std::vector<int> top_index(0);
  for (int i = 0; i < vertex_num; ++i) {
    if (vertex(2, i) <= top_height + eps) {
      top_index.push_back(i);
    }
  }
  // Collect vertices in between.
  std::vector<int> mid_index(0);
  for (int i = 0; i < vertex_num; ++i) {
    const double z = vertex(2, i);
    if (z > top_height + eps && z < ring_height - eps) {
      mid_index.push_back(i);
    }
  }
  // First, we rotate along y = ring_max_y/ring_min_y, depending on the sign of
  // the normal.y().
  double y_axis = (unit_normal.y() > 0 ? ring_max_y : ring_min_y);
  double y_rotate_angle = std::acos(-unit_normal.z())
    * (unit_normal.y() > 0 ? 1.0 : -1.0);
  std::vector<int> all_index(0);
  all_index.reserve(ring_index.size() + mid_index.size() + top_index.size());
  all_index.insert(all_index.end(), ring_index.begin(), ring_index.end());
  all_index.insert(all_index.end(), mid_index.begin(), mid_index.end());
  all_index.insert(all_index.end(), top_index.begin(), top_index.end());
  for (const int i : all_index) {
    const Eigen::Vector3d v = vertex.col(i);
    const Eigen::Vector3d anchor(v.x(), y_axis, ring_height);
    vertex.col(i) = Eigen::AngleAxisd(y_rotate_angle, Eigen::Vector3d::UnitX())
      * (v - anchor) + anchor;
  }
  // Next we rotate along the new x axis.
  double x_axis = (unit_normal.x() > 0 ? ring_max_x : ring_min_x);
  const Eigen::Vector3d last_dir(0.0, unit_normal.y(), unit_normal.z());
  double x_rotation_angle = std::acos(unit_normal.dot(last_dir) / last_dir.norm())
    * (unit_normal.x() > 0 ? 1.0 : -1.0);
  const Eigen::Vector3d rot_axis =
    Eigen::AngleAxisd(y_rotate_angle, Eigen::Vector3d::UnitX()) *
    Eigen::Vector3d(0.0, ring_min_y - ring_max_y, 0.0);
  for (const int i : all_index) {
    const Eigen::Vector3d v = vertex.col(i);
    const Eigen::Vector3d anchor(x_axis, v.y(), v.z());
    vertex.col(i) = Eigen::AngleAxisd(x_rotation_angle, rot_axis.normalized())
      * (v - anchor) + anchor;
  }

  TriMesh new_tube_mount;
  new_tube_mount.Initialize(vertex, tube_mount.element());
  return new_tube_mount;
}

} // copter_simulation