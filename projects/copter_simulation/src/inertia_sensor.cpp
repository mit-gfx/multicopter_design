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
#include "inertial_sensor.h"
// Expose PI.
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <math.h>

namespace copter_simulation {

InertialSensor::InertialSensor()
  : attitude_(Eigen::Quaterniond(1, 0, 0, 0)),
  rpy_(Eigen::Vector3d::Zero()),
  angular_rate_(Eigen::Vector3d::Zero()),
  rpy_rate_(Eigen::Vector3d::Zero()),
  position_(Eigen::Vector3d::Zero()),
  velocity_(Eigen::Vector3d::Zero()),
  acceleration_(Eigen::Vector3d::Zero()) {}

void InertialSensor::Initialize(const Eigen::Quaterniond& attitude,
  const Eigen::Vector3d& angular_rate, const Eigen::Vector3d& position,
  const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration) {
  Advance(attitude, angular_rate, position, velocity, acceleration);
}

void InertialSensor::Advance(const Eigen::Quaterniond& attitude,
  const Eigen::Vector3d& angular_rate, const Eigen::Vector3d& position,
  const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration) {
  attitude_ = attitude;
  angular_rate_ = angular_rate;
  rpy_ = RotationToRollPitchYaw(attitude);
  rpy_rate_ = AngularRateToEulerRate(rpy_, angular_rate);
  position_ = position;
  velocity_ = velocity;
  acceleration_ = acceleration;
}

// Altitude in NED frame.
const double InertialSensor::GetAltitude() const { return -position_.z(); }

const Eigen::Vector3d InertialSensor::WorldVectorToBodyVector(
  const Eigen::Vector3d& world_vector) const {
  return attitude_.matrix().transpose() * world_vector;
}

const Eigen::Vector3d InertialSensor::BodyVectorToWorldVector(
  const Eigen::Vector3d& body_vector) const {
  return attitude_.matrix() * body_vector;
}

const double DegreeToRadian(const double degree) {
  return degree / 180.0 * M_PI;
}

const Eigen::VectorXd DegreeToRadian(const Eigen::VectorXd& degree) {
  return degree / 180.0 * M_PI;
}

const double RadianToDegree(const double radian) {
  return radian / M_PI * 180.0;
}

const Eigen::VectorXd RadianToDegree(const Eigen::VectorXd& radian) {
  return radian / M_PI * 180.0;
}

const double RoundAngle(const double radian) {
  const double period = 2 * M_PI;
  double round_radian = radian - static_cast<int>(radian / period) * period;
  // round_radian is in (-2pi, 2pi) now.
  if (round_radian < -M_PI) round_radian += period;
  else if (round_radian > M_PI) round_radian -= period;
  // Clamp in case we have numerical errors.
  if (round_radian > M_PI) round_radian = M_PI;
  if (round_radian < -M_PI) round_radian = -M_PI;
  return round_radian;
}

const double AngularDifference(const double radian_start,
  const double radian_end) {
  const double round_start = RoundAngle(radian_start),
    round_end = RoundAngle(radian_end);
  const double ccw_diff = round_end - round_start;
  if (std::abs(ccw_diff) <= M_PI) {
    return ccw_diff;
  } else {
    // Should rotate in the other way.
    return ccw_diff + 2 * M_PI * (ccw_diff > 0.0 ? -1.0 : 1.0);
  }
}

// Helpful math functions.
const Eigen::Matrix3d SkewMatrix(const Eigen::Vector3d& w) {
  return (Eigen::Matrix3d() << 0, -w(2), w(1),
    w(2), 0, -w(0),
    -w(1), w(0), 0).finished();
}

const Eigen::Matrix3d RollPitchYawToRotation(const Eigen::Vector3d& rpy) {
  const double roll = rpy(0), pitch = rpy(1), yaw = rpy(2);
  return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).matrix();
}

const Eigen::Matrix3Xd RollPitchYawToRotationMatrixPartialDerivatives(
  const Eigen::Vector3d& rpy) {
  const double roll = rpy(0), pitch = rpy(1), yaw = rpy(2);
  Eigen::Matrix3Xd A = Eigen::MatrixXd::Zero(3, 9);
  const Eigen::Vector3d X = Eigen::Vector3d::UnitX(),
    Y = Eigen::Vector3d::UnitY(), Z = Eigen::Vector3d::UnitZ();
  const Eigen::Matrix3d R_roll = Eigen::AngleAxisd(roll, X).matrix(),
    R_pitch = Eigen::AngleAxisd(pitch, Y).matrix(),
    R_yaw = Eigen::AngleAxisd(yaw, Z).matrix();
  Eigen::Matrix3d dR_roll = (Eigen::Matrix3d() <<
    1, 0, 0,
    0, 0, -1,
    0, 1, 0).finished() * R_roll;
  dR_roll(0, 0) = 0;
  Eigen::Matrix3d dR_pitch = (Eigen::Matrix3d() <<
    0, 0, 1,
    0, 1, 0,
    -1, 0, 0).finished() * R_pitch;
  dR_pitch(1, 1) = 0;
  Eigen::Matrix3d dR_yaw = (Eigen::Matrix3d() <<
    0, -1, 0,
    1, 0, 0,
    0, 0, 1).finished() * R_yaw;
  dR_yaw(2, 2) = 0;

  // d/d_roll.
  A.leftCols(3) = R_yaw * R_pitch * dR_roll;
  // d/d_pitch.
  A.middleCols(3, 3) = R_yaw * dR_pitch * R_roll;
  // d/d_yaw.
  A.rightCols(3) = dR_yaw * R_pitch * R_roll;
  return A;
}

const Eigen::Vector3d RotationToRollPitchYaw(
  const Eigen::Quaterniond& attitude) {
  // The three angles are computed based on the slides here:
  // http://www.princeton.edu/~stengel/MAE331Lecture9.pdf, page 3.
  double roll, pitch, yaw;
  const Eigen::Matrix3d R = attitude.matrix();
  const Eigen::Vector3d XI = Eigen::Vector3d::UnitX();
  const Eigen::Vector3d YI = Eigen::Vector3d::UnitY();
  const Eigen::Vector3d ZI = Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d XB = R.col(0), YB = R.col(1), ZB = R.col(2);
  // Let's first rotate along XB to compute Y2.
  const Eigen::Vector3d Y2 = ZI.cross(XB).normalized();
  double cos_roll = Y2.dot(YB);
  // Clamp cosRoll (if necessary).
  if (cos_roll > 1.0) cos_roll = 1.0;
  if (cos_roll < -1.0) cos_roll = -1.0;
  roll = acos(cos_roll);
  // Check to see whether we need to swap the sign of roll.
  if (Y2.dot(ZB) > 0.0) roll = -roll;

  // Next let's rotate along Y2 so that X1 falls into XOY plane.
  const Eigen::Vector3d X1 = Y2.cross(ZI).normalized();
  double cos_pitch = X1.dot(XB);
  // Clamp cosPitch.
  if (cos_pitch > 1.0) cos_pitch = 1.0;
  if (cos_pitch < -1.0) cos_pitch = -1.0;
  pitch = acos(cos_pitch);
  // Check to see whether we need to swap the sign of pitch.
  if (XB(2) > 0.0) pitch = -pitch;

  // We finally need to rotate along ZI to compute yaw.
  double cos_yaw = X1.dot(XI);
  // Clamp cosYaw.
  if (cos_yaw > 1.0) cos_yaw = 1.0;
  if (cos_yaw < -1.0) cos_yaw = -1.0;
  yaw = acos(cos_yaw);
  if (X1(1) < 0) yaw = -yaw;
  return Eigen::Vector3d(roll, pitch, yaw);
  // Sanity check:
  // const Matrix3 R1 = AngleAxis(yaw, Vector3::UnitZ())
  //  * AngleAxis(pitch, Vector3::UnitY())
  //  * AngleAxis(roll, Vector3::UnitX()).matrix();
  // const Matrix3 R2 = AngleAxis(roll, XB)
  //   * AngleAxis(pitch, Y2)
  //   * AngleAxis(yaw, ZI).matrix();
  // std::cout << "Diff between R and R1 = " << (R - R1).norm() << std::endl;
  // std::cout << "Diff between R and R2 = " << (R - R2).norm() << std::endl;
}

const Eigen::Vector3d EulerRateToBodyAngularRate(const Eigen::Vector3d& rpy,
  const Eigen::Vector3d& rpy_rate) {
  return EulerRateToBodyAngularRateMatrix(rpy) * rpy_rate;
}

// Compute Euler rates from attitude and angular rate.
// Reference:
// http://www.princeton.edu/~stengel/MAE331Lecture9.pdf
const Eigen::Vector3d AngularRateToEulerRate(const Eigen::Vector3d& rpy,
  const Eigen::Vector3d& angular_rate) {
  return EulerRateToBodyAngularRateMatrixInverse(rpy) *
    (RollPitchYawToRotation(rpy).transpose() * angular_rate);
}

const Eigen::Matrix3Xd EulerRateToBodyAngularRatePartialDerivatives(
  const Eigen::Vector3d& rpy, const Eigen::Vector3d& rpy_rate) {
  Eigen::Matrix3Xd A = Eigen::MatrixXd::Zero(3, 6);
  const Eigen::Matrix3Xd B =
    EulerRateToBodyAngularRateMatrixPartialDerivatives(rpy);
  A.col(0) = B.leftCols(3) * rpy_rate;
  A.col(1) = B.middleCols(3, 3) * rpy_rate;
  A.col(2) = B.rightCols(3) * rpy_rate;
  A.rightCols(3) = EulerRateToBodyAngularRateMatrix(rpy);
  return A;
}

const Eigen::Matrix3d EulerRateToBodyAngularRateMatrix(
  const Eigen::Vector3d& rpy) {
  const double roll = rpy(0), pitch = rpy(1);
  const double s_roll = sin(roll), c_roll = cos(roll),
    s_pitch = sin(pitch), c_pitch = cos(pitch);
  return (Eigen::Matrix3d() << 1, 0, -s_pitch,
    0, c_roll, s_roll * c_pitch,
    0, -s_roll, c_roll * c_pitch).finished();
}

const Eigen::Matrix3Xd EulerRateToBodyAngularRateMatrixPartialDerivatives(
  const Eigen::Vector3d& rpy) {
  const double roll = rpy(0), pitch = rpy(1);
  const double s_roll = sin(roll), c_roll = cos(roll),
    s_pitch = sin(pitch), c_pitch = cos(pitch);
  Eigen::Matrix3Xd A = Eigen::MatrixXd::Zero(3, 9);
  A.leftCols(3) = (Eigen::Matrix3d() << 0, 0, 0,
    0, -s_roll, c_roll * c_pitch,
    0, -c_roll, -s_roll * c_pitch).finished();
  A.middleCols(3, 3) = (Eigen::Matrix3d() << 0, 0, -c_pitch,
    0, 0, -s_roll * s_pitch,
    0, 0, -c_roll * s_pitch).finished();
  return A;
}

const Eigen::Matrix3d EulerRateToBodyAngularRateMatrixInverse(
  const Eigen::Vector3d& rpy) {
  // Reference:
  // http://www.princeton.edu/~stengel/MAE331Lecture9.pdf.
  const double roll = rpy(0), pitch = rpy(1), yaw = rpy(2);
  const double s_roll = sin(roll), c_roll = cos(roll),
    s_pitch = sin(pitch), c_pitch = cos(pitch), t_pitch = tan(pitch);
  return (Eigen::Matrix3d() << 1, s_roll * t_pitch, c_roll * t_pitch,
    0, c_roll, -s_roll,
    0, s_roll / c_pitch, c_roll / c_pitch).finished();
}

const Eigen::Matrix3d EulerRateToBodyAngularRateMatrixDerivative(
  const Eigen::Vector3d& rpy, const Eigen::Vector3d& rpy_rate) {
  const double roll = rpy(0), pitch = rpy(1);
  const double s_roll = sin(roll), c_roll = cos(roll),
    s_pitch = sin(pitch), c_pitch = cos(pitch);
  const double roll_dot = rpy_rate(0), pitch_dot = rpy_rate(1);
  return (Eigen::Matrix3d() << 0, 0, -c_pitch * pitch_dot,
    0, -s_roll * roll_dot,
    c_roll * c_pitch * roll_dot - s_roll * s_pitch * pitch_dot,
    0, -c_roll * roll_dot,
    -s_roll * c_pitch * roll_dot - c_roll * s_pitch * pitch_dot).finished();
}

const Eigen::Matrix3Xd
EulerRateToBodyAngularRateMatrixDerivativePartialDerivatives(
  const Eigen::Vector3d& rpy, const Eigen::Vector3d& rpy_rate) {
  Eigen::Matrix3Xd A = Eigen::Matrix3Xd::Zero(3, 18);
  const double roll = rpy(0), pitch = rpy(1);
  const double s_roll = sin(roll), c_roll = cos(roll),
    s_pitch = sin(pitch), c_pitch = cos(pitch);
  const double roll_dot = rpy_rate(0), pitch_dot = rpy_rate(1);
  // d_roll.
  A.leftCols(3) = (Eigen::Matrix3d() << 0, 0, 0,
    0, -c_roll * roll_dot,
    -s_roll * c_pitch * roll_dot - c_roll * s_pitch * pitch_dot,
    0, s_roll * roll_dot,
    -c_roll * c_pitch * roll_dot + s_roll * s_pitch * pitch_dot).finished();
  // d_pitch.
  A.middleCols(3, 3) = (
    Eigen::Matrix3d() << 0, 0, s_pitch * pitch_dot,
    0, 0, c_roll * (-s_pitch) * roll_dot - s_roll * c_pitch * pitch_dot,
    0, 0, -s_roll * (-s_pitch) * roll_dot - c_roll * c_pitch * pitch_dot
  ).finished();
  A.rightCols(9) = EulerRateToBodyAngularRateMatrixPartialDerivatives(rpy);
  return A;
}

} // copter_simulation