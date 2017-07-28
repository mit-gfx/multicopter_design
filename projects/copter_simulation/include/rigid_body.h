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
// All physics quantities are defined in the world space unless explicitly
// mentioned that they are in the body space.
// The status of the rigid body is defined by:
// - x: it's center of mass in the world space (position).
// - v: the velocity of the c.o.m. (velocity).
// - R: the orientation of the rigid body. Imaging there are body axes fixed
// on the rigid body, then the three columns of R represent x, y and z axes in
// the world space.
// - w: the angular velocity represented in the world space. \dot{R} = w x R.
// Another two physics quantities are needed for the governing equations:
// - m: the mass of the rigid body.
// - I: the inertial tensor, defined in the original body space and is constant
// during the simulation.
// The linear motion is easy to update:
// - F = ma: where F is the net external force, a is the acceleration of the
// c.o.m.
// The angular motion bears some discussion. Here is how we do that:
// Consider the angular momentum w.r.t the c.o.m.: H = \int r' x v' dm. Both
// r' and v' are the relative position/velocity between dm and c.o.m. defined
// in the world space. We will use r and v to represent the absolute position
// and velocity in the world space (sorry for the abuse of notation of v here
// since it was already defined previously). Note that \int r' dm = 0, so H is
// also equal to \int r' x v dm.
// Now let's consider \dot{H} = \int r' x a dm + \int v' x v dm. Since we know
// \int r' dm = 0, we have \int v' dm = 0, so \int v' x v dm = \int v' x (v0 +
// v') dm = (\int v' dm) x v0 = 0. As a result, \dot{H} = \int r' x dF =
// \int dM, where the right hand side is the net external torque w.r.t. the
// c.o.m., represented in the world space.
// Next, let's connect the angular momentum with the inertia tensor and the
// angular velocity: H = \int r' x v' dm = \int r' x (w x r') dm =
// \int -r' x (r' x w) dm = (\int -skew(r')skew(r') dm) w, where the integral
// is defined as the inertial tensor.
// Finally, if the rigid body gets rotated, how should we update the inertia
// tensor? Consider y' = Rr', we have:
// \int -skew(y')skew(y') dm = \int -y' x (y' x Identity) dm
// = \int -Rr' x (Rr' x RR^T) dm = \int -Rr' x R(r' x R^T) dm
// = R(\int -r' x (r' x R^T) dm) = R(\int -skew(r')skew(r') dm)R^T.
// So now we have all pieces to update the angular motion:
// - Use external torque to update the angular momentum.
// - Use the angular velocity to update the orientation.
// - Based on the new orientation, update the inertia tensor.
// - Compute the new angular velocity.
#ifndef _COPTER_SIMULATION_RIGID_BODY_H_
#define _COPTER_SIMULATION_RIGID_BODY_H_

#include "Eigen/Dense"

namespace copter_simulation {

class RigidBody {

public:
  void Initialize(const double mass, const Eigen::Matrix3d& inertia_body,
    const Eigen::Vector3d& position, const Eigen::Vector3d& velocity,
    const Eigen::Quaterniond& orientation,
    const Eigen::Vector3d& angular_velocity);
  virtual ~RigidBody() {}

  const double mass() const { return mass_; }
  const double mass_inv() const { return mass_inv_; }
  const Eigen::Matrix3d inertia_body() const { return inertia_body_; }
  const Eigen::Matrix3d inertia_body_inv() const { return inertia_body_inv_; }
  const Eigen::Vector3d position() const { return position_; }
  const Eigen::Vector3d velocity() const { return velocity_; }
  const Eigen::Vector3d acceleration() const { return acceleration_; }
  const Eigen::Quaterniond orientation() const { return orientation_; }
  const Eigen::Vector3d angular_velocity() const { return angular_velocity_; }

  // R is the rotation matrix that defines the transform from body space to
  // world space.
  const Eigen::Matrix3d WorldInertia(const Eigen::Matrix3d& R) const;
  const Eigen::Matrix3d WorldInertiaInv(const Eigen::Matrix3d& R) const;

  void ApplyForce(const Eigen::Vector3d& force, const Eigen::Vector3d& position);
  void ApplyTorque(const Eigen::Vector3d& torque);
  void ClearExternalForce();
  void ClearExternalTorque();
  void Advance(const double dt);

  const Eigen::Matrix3Xd WorldPointToBodyPoint(
    const Eigen::Matrix3Xd& world_point) const;
  const Eigen::Matrix3Xd BodyPointToWorldPoint(
    const Eigen::Matrix3Xd& body_point) const;
  const Eigen::Matrix3Xd WorldVectorToBodyVector(
    const Eigen::Matrix3Xd& world_vector) const;
  const Eigen::Matrix3Xd BodyVectorToWorldVector(
    const Eigen::Matrix3Xd& body_vector) const;
  // Homogeneous transform matrix.
  const Eigen::Matrix4d BodyToWorldTransform() const;
  const Eigen::Matrix4d WorldToBodyTransform() const;

protected:
  RigidBody();

  double mass_;
  double mass_inv_;
  Eigen::Matrix3d inertia_body_;
  Eigen::Matrix3d inertia_body_inv_;

  Eigen::Vector3d position_;
  Eigen::Vector3d velocity_;
  Eigen::Vector3d acceleration_;
  Eigen::Vector3d external_force_;

  Eigen::Quaterniond orientation_;
  Eigen::Vector3d angular_velocity_;
  Eigen::Vector3d external_torque_;
};

} // copter_simulation

#endif  // _COPTER_SIMULATION_RIGID_BODY_H_