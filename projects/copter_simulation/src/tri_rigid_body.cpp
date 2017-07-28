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
#include "tri_rigid_body.h"
#include "geometry.h"

namespace copter_simulation {

TriRigidBody::TriRigidBody()
  : RigidBody(), body_space_mesh_() {}

void TriRigidBody::Initialize(const double density,
  const TriMesh& world_space_mesh) {
  velocity_ = Eigen::Vector3d::Zero();
  acceleration_.setZero();
  external_force_.setZero();

  orientation_ = Eigen::Quaterniond(Eigen::Matrix3d::Identity());
  angular_velocity_ = Eigen::Vector3d::Zero();
  external_torque_.setZero();

  // Using the divergence theorem (Gauss's theorem) to compute the volume,
  // the center of mass, and the inertia tensor.
  // A note on Gauss's theorem: if V is a vector function from R3 to R3:
  // V(x, y, z) = (X(x, y, z), Y(x, y, z), Z(x, y, z)).
  // Let \Omega be a closed volume in R3, and S be its surface, we have:
  // \int_S (V.dot(n))ds = \int_\Omega (dX/dx + dY/dy + dZ/dz)dxdydz
  //
  // The next question is how to integrate a scalar function f(x, y, z) over a
  // triangle in R3. Let v0, v1, v3 be its three vertice, we can parameterize
  // it with u \in [0, 1], v \in [0, 1] and u + v <= 1, such that (x, y, z) =
  // v0 + u * (v1 - v0) + v * (v2 - v0):
  // \int_S f(x, y, z) ds =
  // sin(theta) \int_{v = 0}^{1}\int_{u = 0}^{1 - v} f(u, v) dudv.
  // where theta is the angle between (v1 - v0) and (v2 - v0).
  //
  // Here are some very useful integrations:
  // Define F(f(u, v)) = \int_{v = 0}^{1}\int_{u = 0}^{1 - v} f dudv.
  // F(1) = 1 / 2.
  // F(u) = 1 / 6.
  // F(v) = 1 / 6.
  // F(u^2) = 1 / 12.
  // F(v^2) = 1 / 12.
  // F(uv) = 1 / 24.
  // F(u^3) = 1 / 20.
  // F(v^3) = 1 / 20.
  // F(u^2v) = 1 / 60.
  // F(uv^2) = 1 / 60.
  //
  // For volume, V(x, y, z) = (x, y, z) / 3. f(x, y, z) = v0 * n / 3.
  // So the volume is v0 * n * S / 3.
  // For the center of mass: Vx(x, y, z) = (x2 / 2, 0, 0).
  // f = x2 / 2 * nx = nx / 2 * (v0.x + u * v10.x + v * v20.x)^2.
  body_space_mesh_ = world_space_mesh;
  const int triangle_number = body_space_mesh_.NumOfElement();
  double volume = 0.0;
  position_.setZero();
  // Reuse some data.
  Eigen::Matrix3Xd A, B, C, A2, B2, C2, AB, AC, BC, N;
  A = B = C = A2 = B2 = C2 = AB = AC = BC = N
    = Eigen::Matrix3Xd::Zero(3, triangle_number);
  for (int i = 0; i < triangle_number; ++i) {
    const Eigen::Matrix3d triangle = body_space_mesh_.vertex_in_element(i);
    // Get the outward normal, scaled by the area of the triangle.
    const Eigen::Vector3d a = triangle.col(0), v1 = triangle.col(1),
      v2 = triangle.col(2);
    const Eigen::Vector3d b = v1 - a, c = v2 - a;
    const Eigen::Vector3d n_scaled = b.cross(c);
    // Compute the volume: V(x, y, z) = (x, y, z) / 3.
    volume += n_scaled.dot(a);
    // Cache data.
    A.col(i) = a; B.col(i) = b; C.col(i) = c; N.col(i) = n_scaled;
  }
  // Compute the center of mass:
  // Vx(x, y, z) = (x^2/2, 0, 0).
  // Vy(x, y, z) = (0, y^2/2, 0).
  // Vz(x, y, z) = (0, 0, z^2/2).
  A2 = A.array().square(); B2 = B.array().square(); C2 = C.array().square();
  AB = A.array() * B.array(); AC = A.array() * C.array();
  BC = B.array() * C.array();
  position_ = (A2 / 2 + B2 / 12 + C2 / 12 + AB / 3 + AC / 3
    + BC / 12).cwiseProduct(N).rowwise().sum();
  volume /= 6.0;
  mass_ = volume * density;
  mass_inv_ = 1.0 / mass_;
  position_ /= 2.0 * volume;
  // Shift the mesh so that the center of mass is at the origin.
  body_space_mesh_.Translate(-position_);
  // Update A, A2, AB, AC.
  A = A.colwise() - position_;
  A2 = A.array().square();
  AB = A.array() * B.array(); AC = A.array() * C.array();

  // Compute the inertia tensor.
  // For V = (x^3, 0, 0) / 3: f = x^3 * nx / 3
  // = (v0.x + u * v10.x + v * v20.x)^3 * nx / 3.
  // For V = (x2y / 2, 0, 0): f = x2y * nx / 2 = nx / 2 *
  // (v0.x + u * v10.x + v * v20.x)^2 * (v0.y + u * v10.y + v * v20.y).
  // Compute the diagonal elements.
  const Eigen::Matrix3Xd A3 = A.array().cube(), B3 = B.array().cube(),
    C3 = C.array().cube(), ABC = AB.array() * C.array();
  Eigen::Matrix3Xd d_x2y2z2_integral = A3 + (B3 + C3) / 10.0;
  d_x2y2z2_integral += A.cwiseProduct(B2 + C2) / 2.0;
  d_x2y2z2_integral += (B + C).cwiseProduct(A2 + BC / 10.0) + ABC / 2.0;
  const Eigen::Vector3d x2y2z2_integral =
    N.cwiseProduct(d_x2y2z2_integral).rowwise().sum() / 6.0;

  // Compute the off-diagonal elements.
  Eigen::Matrix3Xd A_next = A, B_next = B, C_next = C;
  A_next.topRows(2) = A.bottomRows(2); A_next.row(2) = A.row(0);
  B_next.topRows(2) = B.bottomRows(2); B_next.row(2) = B.row(0);
  C_next.topRows(2) = C.bottomRows(2); C_next.row(2) = C.row(0);
  const Eigen::Matrix3Xd B2B_next = B2.array() * B_next.array(),
    C2C_next = C2.array() * C_next.array(),
    B2C_next = B2.array() * C_next.array(),
    C2B_next = C2.array() * B_next.array();
  Eigen::Matrix3Xd d_integral =
    (A_next / 2.0 + (B_next + C_next) / 6.0).cwiseProduct(A2);
  d_integral += ((A_next + B_next / 2.0).cwiseProduct(AB)
    + (A_next + C_next / 2.0).cwiseProduct(AC)) / 3.0;
  d_integral += A_next.cwiseProduct(B2 + C2) / 12.0;
  d_integral += (AB.cwiseProduct(C_next) + AC.cwiseProduct(B_next) +
    BC.cwiseProduct(A_next)) / 12.0;
  d_integral += (B2B_next + C2C_next) / 20.0 + (B2C_next + C2B_next) / 60.0;
  d_integral += BC.cwiseProduct(B_next + C_next) / 30.0;
  const Eigen::Vector3d xy_yz_zx_integral =
    N.cwiseProduct(d_integral).rowwise().sum() / 2.0;

  // Assemble the inertia tensor.
  const double x2 = x2y2z2_integral.x(), y2 = x2y2z2_integral.y(),
    z2 = x2y2z2_integral.z();
  const double xy = xy_yz_zx_integral(0), yz = xy_yz_zx_integral(1),
    zx = xy_yz_zx_integral(2);
  inertia_body_ = (Eigen::Matrix3d() <<
    y2 + z2, -xy, -zx,
    -xy, x2 + z2, -yz,
    -zx, -yz, x2 + y2).finished() * density;
  inertia_body_inv_ = inertia_body_.inverse();
}

void TriRigidBody::Translate(const double x, const double y,
  const double z) {
  Translate(Eigen::Vector3d(x, y, z));
}

void TriRigidBody::Translate(const Eigen::Vector3d& translation) {
  position_ += translation;
}

void TriRigidBody::Rotate(const Eigen::Matrix3d& rotation) {
  // Do not allow the rotation matrix to change the handness of the system.
  assert(rotation.determinant() > 0.0);
  position_ = rotation * position_;
  body_space_mesh_.Rotate(rotation);
  inertia_body_ = rotation * inertia_body_ * rotation.transpose();
  inertia_body_inv_ = rotation * inertia_body_inv_ * rotation.transpose();
}

void TriRigidBody::Scale(const double s) {
  Scale(Eigen::Vector3d(s, s, s));
}

void TriRigidBody::Scale(const double x, const double y,
  const double z) {
  Scale(Eigen::Vector3d(x, y, z));
}

void TriRigidBody::Scale(const Eigen::Vector3d& scale) {
  // Do not allow scale factor to change the handness of the system.
  assert(scale.minCoeff() > 0.0);
  mass_ *= scale.cwiseAbs().prod();
  mass_inv_ = 1.0 / mass_;
  position_ = position_.cwiseProduct(scale);
  body_space_mesh_.Scale(scale);

  // Inertia sensor is a little tricky here.
  const Eigen::Vector3d Ix2y2z2 = inertia_body_.diagonal().sum() / 2.0
    - inertia_body_.diagonal().array();
  const Eigen::Vector3d Ix2y2z2_scaled = Ix2y2z2.array()
    * scale.array().square();
  inertia_body_ = scale.asDiagonal() * inertia_body_ * scale.asDiagonal();
  // Update the main diagonal.
  inertia_body_(0, 0) = Ix2y2z2_scaled.y() + Ix2y2z2_scaled.z();
  inertia_body_(1, 1) = Ix2y2z2_scaled.x() + Ix2y2z2_scaled.z();
  inertia_body_(2, 2) = Ix2y2z2_scaled.x() + Ix2y2z2_scaled.y();
  // The volume also changes.
  inertia_body_ *= scale.prod();
  // Finally, update its inverse.
  inertia_body_inv_ = inertia_body_.inverse();
}

void TriRigidBody::Transform(const Eigen::Matrix4d& transform) {
  Eigen::Vector3f translation, scale;
  Eigen::Matrix3f rotation_u, rotation_v;
  opengl_viewer::DecomposeTransform(transform.cast<float>(),
    translation, rotation_u, scale, rotation_v);
  Rotate(rotation_v.cast<double>());
  Scale(scale.cast<double>());
  Rotate(rotation_u.cast<double>());
  Translate(translation.cast<double>());
}

} // copter_simulation
