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
// An inertial sensor includes a gyroscope and an accelerometer. They provide
// the angular velocities and the accelerations along the three axes. All of
// them are defined in the world frame (NED frame).
//
// Reference:
// Gaberial Hoffmann, 2008. http://ai.stanford.edu/~gabeh/papers/gmhThesis.pdf
#ifndef _COPTER_SIMULATION_INERTIAL_SENSOR_H_
#define _COPTER_SIMULATION_INERTIAL_SENSOR_H_

#include "Eigen/Dense"

namespace copter_simulation {

class InertialSensor {
public:
  InertialSensor();

  void Initialize(const Eigen::Quaterniond& attitude,
    const Eigen::Vector3d& angular_rate, const Eigen::Vector3d& position,
    const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration);

  void Advance(const Eigen::Quaterniond& attitude,
    const Eigen::Vector3d& angular_rate, const Eigen::Vector3d& position,
    const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration);

  // Angular motion.
  const Eigen::Quaterniond attitude() const { return attitude_; }
  const Eigen::Vector3d angular_rate() const { return angular_rate_; }
  const Eigen::Vector3d rpy() const { return rpy_; }
  const Eigen::Vector3d rpy_rate() const { return rpy_rate_; }

  // Linear motion.
  // The world frame is defined as follows (NED frame in meters):
  // x: north; y: east; z: down.
  const Eigen::Vector3d position() const { return position_; }
  const Eigen::Vector3d velocity() const { return velocity_; }
  const Eigen::Vector3d acceleration() const { return acceleration_; }
  // Get the altitude.
  const double GetAltitude() const;

  // Conversion between world and body frames.
  const Eigen::Vector3d WorldVectorToBodyVector(
    const Eigen::Vector3d& world_vector) const;
  const Eigen::Vector3d BodyVectorToWorldVector(
    const Eigen::Vector3d& body_vector) const;

private:
  // Gyro:
  // AHRS information. In real world these values are computed by methods
  // like DCM or EKF based on the gyro and accelerometer. Here we just
  // give InertialSensor the ground truth.
  Eigen::Quaterniond attitude_;
  // roll, pitch and yaw are extracted from attitude_.
  Eigen::Vector3d rpy_;
  // The angular rate defined in the world frame.
  Eigen::Vector3d angular_rate_;
  // The Euler angular rates.
  Eigen::Vector3d rpy_rate_;

  // Accelerometer:
  Eigen::Vector3d position_;
  Eigen::Vector3d velocity_;
  Eigen::Vector3d acceleration_;
};

// Helpful math functions.
const double DegreeToRadian(const double degree);
const Eigen::VectorXd DegreeToRadian(const Eigen::VectorXd& degree);
const double RadianToDegree(const double radian);
const Eigen::VectorXd RadianToDegree(const Eigen::VectorXd& radian);
// Round radian to [-pi, pi].
const double RoundAngle(const double radian);
const double AngularDifference(const double radian_start,
  const double radian_end);
const Eigen::Matrix3d SkewMatrix(const Eigen::Vector3d& w);
const Eigen::Matrix3d RollPitchYawToRotation(const Eigen::Vector3d& rpy);
const Eigen::Matrix3Xd RollPitchYawToRotationMatrixPartialDerivatives(
  const Eigen::Vector3d& rpy);
const Eigen::Vector3d RotationToRollPitchYaw(
  const Eigen::Quaterniond& attitude);
const Eigen::Vector3d EulerRateToBodyAngularRate(const Eigen::Vector3d& rpy,
  const Eigen::Vector3d& rpy_rate);
const Eigen::Vector3d AngularRateToEulerRate(const Eigen::Vector3d& rpy,
  const Eigen::Vector3d& angular_rate);
const Eigen::Matrix3Xd EulerRateToBodyAngularRatePartialDerivatives(
  const Eigen::Vector3d& rpy, const Eigen::Vector3d& rpy_rate);
const Eigen::Matrix3d EulerRateToBodyAngularRateMatrix(
  const Eigen::Vector3d& rpy);
const Eigen::Matrix3Xd EulerRateToBodyAngularRateMatrixPartialDerivatives(
  const Eigen::Vector3d& rpy);
const Eigen::Matrix3d EulerRateToBodyAngularRateMatrixInverse(
  const Eigen::Vector3d& rpy);
const Eigen::Matrix3d EulerRateToBodyAngularRateMatrixDerivative(
  const Eigen::Vector3d& rpy, const Eigen::Vector3d& rpy_rate);
const Eigen::Matrix3Xd
EulerRateToBodyAngularRateMatrixDerivativePartialDerivatives(
  const Eigen::Vector3d& rpy, const Eigen::Vector3d& rpy_rate);

} // copter_simulation

#endif  // _COPTER_SIMULATION_INERTIAL_SENSOR_H_