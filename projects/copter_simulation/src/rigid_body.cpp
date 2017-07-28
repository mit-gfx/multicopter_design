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
#include "rigid_body.h"

namespace copter_simulation {

RigidBody::RigidBody()
  : mass_(1.0),
  mass_inv_(1.0),
  inertia_body_(Eigen::Matrix3d::Identity()),
  inertia_body_inv_(Eigen::Matrix3d::Identity()),
  position_(Eigen::Vector3d::Zero()),
  velocity_(Eigen::Vector3d::Zero()),
  acceleration_(Eigen::Vector3d::Zero()),
  external_force_(Eigen::Vector3d::Zero()),
  orientation_(Eigen::Matrix3d::Identity()),
  angular_velocity_(Eigen::Vector3d::Zero()),
  external_torque_(Eigen::Vector3d::Zero()) {}

void RigidBody::Initialize(const double mass,
  const Eigen::Matrix3d& inertia_body,
  const Eigen::Vector3d& position,
  const Eigen::Vector3d& velocity,
  const Eigen::Quaterniond& orientation,
  const Eigen::Vector3d& angular_velocity) {
  mass_ = mass;
  mass_inv_ = 1.0 / mass;
  inertia_body_ = inertia_body;
  inertia_body_inv_ = inertia_body.inverse();
  position_ = position;
  velocity_ = velocity;
  acceleration_ = Eigen::Vector3d::Zero();
  external_force_ = Eigen::Vector3d::Zero();
  orientation_ = orientation.normalized();
  angular_velocity_ = angular_velocity;
  external_torque_ = Eigen::Vector3d::Zero();
}

const Eigen::Matrix3d RigidBody::WorldInertia(
  const Eigen::Matrix3d& R) const {
  return R * inertia_body_ * R.transpose();
}

const Eigen::Matrix3d RigidBody::WorldInertiaInv(
  const Eigen::Matrix3d& R) const {
  return R * inertia_body_inv_ * R.transpose();
}

void RigidBody::ApplyForce(const Eigen::Vector3d& force,
  const Eigen::Vector3d& position) {
  external_force_ += force;
  external_torque_ += (position - position_).cross(force);
}

void RigidBody::ApplyTorque(const Eigen::Vector3d& torque) {
  external_torque_ += torque;
}

void RigidBody::ClearExternalForce() {
  external_force_.setZero();
}

void RigidBody::ClearExternalTorque() {
  external_torque_.setZero();
}

void RigidBody::Advance(const double dt) {
  // Forward Euler update method.
  // Update linear motion.
  position_ = position_ + velocity_ * dt;
  acceleration_ = external_force_ * mass_inv_;
  velocity_ = velocity_ + acceleration_ * dt;

  // Update angular motion.
  const Eigen::Vector3d angular_momentum =
    WorldInertia(orientation_.matrix()) * angular_velocity_;

  Eigen::Vector3d omega = angular_velocity_ * dt;
  const double half_theta = omega.norm() / 2.0;
  if (half_theta > 1e-10) {
    omega.normalize();
    orientation_ = Eigen::Quaterniond(cos(half_theta),
      omega(0) * sin(half_theta), omega(1) * sin(half_theta),
      omega(2) * sin(half_theta)).normalized() * orientation_;
    orientation_.normalize();
  }
  angular_velocity_ = WorldInertiaInv(orientation_.matrix())
    * (angular_momentum + external_torque_ * dt);

  // Clear external forces and torques.
  external_force_.setZero();
  external_torque_.setZero();
}

const Eigen::Matrix3Xd RigidBody::WorldPointToBodyPoint(
  const Eigen::Matrix3Xd& world_point) const {
  return orientation_.matrix().transpose() * (
    world_point.colwise() - position_);
}
const Eigen::Matrix3Xd RigidBody::BodyPointToWorldPoint(
  const Eigen::Matrix3Xd& body_point) const {
  return (orientation_.matrix() * body_point).colwise() + position_;
}
const Eigen::Matrix3Xd RigidBody::WorldVectorToBodyVector(
  const Eigen::Matrix3Xd& world_vector) const {
  return orientation_.matrix().transpose() * world_vector;
}
const Eigen::Matrix3Xd RigidBody::BodyVectorToWorldVector(
  const Eigen::Matrix3Xd& body_vector) const {
  return orientation_.matrix() * body_vector;
}

const Eigen::Matrix4d RigidBody::BodyToWorldTransform() const {
  return (Eigen::Matrix4d() << orientation_.matrix(), position_,
    0.0, 0.0, 0.0, 1.0).finished();
}

const Eigen::Matrix4d RigidBody::WorldToBodyTransform() const {
  return (Eigen::Matrix4d() << orientation_.matrix().transpose(),
    -orientation_.matrix().transpose() * position_,
    0.0, 0.0, 0.0, 1.0).finished();
}

} // copter_simulation
