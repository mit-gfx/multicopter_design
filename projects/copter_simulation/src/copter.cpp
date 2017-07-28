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
#include "copter.h"
#include <iostream>
#include "geometry.h"
#include "nlopt_linear_function.h"
#include "nlopt_wrapper.h"

namespace copter_simulation {

class NloptClosestParemeterObjective : public NloptFunction {
public:
  NloptClosestParemeterObjective()
    : NloptFunction(), A_(0, 0), b_(0) {}

  // (Ax - b)^2.
  void Initialize(const std::string& x_name, const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b) {
    const int num = static_cast<int>(A.cols());
    std::vector<std::string> x_names(num, x_name + "_");
    for (int i = 0; i < num; ++i) {
      x_names[i] += std::to_string(i);
    }
    const double inf = NloptFunction::Infinity();
    NloptFunction::Initialize(x_names, -inf, inf);
    assert(A.rows() == b.size());
    A_ = A;
    b_ = b;
  }

  const Eigen::VectorXd Evaluate(const Eigen::VectorXd& x,
    Eigen::MatrixXd& jacobian) {
    assert(x.size() == A_.cols());
    if (jacobian.size()) {
      assert(jacobian.rows() == 1);
      assert(jacobian.cols() == A_.cols());
      jacobian = 2 * (A_ * x - b_).transpose() * A_;
    }
    return DoubleToVectorXd((A_ * x - b_).squaredNorm());
  }

private:
  Eigen::MatrixXd A_;
  Eigen::VectorXd b_;
};

Copter::Copter() : designer_(),
  battery_(),
  propeller_(),
  inertial_sensor_(),
  rotors_(0),
  copter_body_(),
  rotor_effective_matrix_(Eigen::MatrixXd::Zero(6, 0)),
  pid_climb_rate_controller_(),
  pid_roll_controller_(),
  pid_pitch_controller_(),
  pid_x_rate_controller_(),
  pid_y_rate_controller_(),
  lqr_position_controller_(),
  current_flight_mode_(kPosition),
  current_flight_mode_target_(Eigen::VectorXd::Zero(0)) {}

void Copter::Initialize(const std::string& xml_name) {
  designer_.Initialize(xml_name);
  SyncCopterFromDesign();
  InitializeController();
}

const Eigen::VectorXd Copter::GetFullDesignParameter() const {
  return designer_.GetCurrentFullDesignParameter();
}

const Eigen::VectorXd Copter::GetReducedDesignParameter() const {
  return designer_.GetCurrentReducedDesignParameter();
}

void Copter::SyncCopterFromDesign() {
  battery_ = designer_.battery();
  propeller_ = designer_.PropellerInModelSpace();
  rotors_ = designer_.GetCurrentRotorDesign();
  copter_body_ = designer_.GetCurrentCopterDesign();
  // Initialize the inertial sensor.
  inertial_sensor_.Initialize(copter_body_.orientation(),
    copter_body_.angular_velocity(), copter_body_.position(),
    copter_body_.velocity(), copter_body_.acceleration());
}

void Copter::SetReducedDesignParameter(const Eigen::VectorXd& parameter) {
  designer_.SetReducedDesignParameter(parameter);
  SyncCopterFromDesign();
}

void Copter::SetFullDesignParameter(const Eigen::VectorXd& parameter) {
  static Eigen::VectorXd last_parameter(0);
  if (last_parameter.size() == parameter.size() &&
    (last_parameter - parameter).cwiseAbs().maxCoeff() == 0.0) return;
  last_parameter = parameter;

  // Find the closest feasible reduced parameter by solving a linear
  // programming problem:
  // min \|(x0 + Ey) - x*\|.
  //      Fy <= g.
  NloptWrapper lp;
  opengl_viewer::Option option;
  // Terminate after 100 iterations so that users can explore the design in
  // real time.
  option.SetIntOption("max eval", 100);
  lp.Initialize(option);

  // Add variables.
  const DesignProblem& problem = designer_.design_constraints();
  const int full_dof = problem.NumOfFullParameter();
  const int reduced_dof = problem.NumOfReducedParameter();
  assert(full_dof == static_cast<int>(parameter.size()));
  lp.AddVariable("y", Eigen::VectorXd::Zero(reduced_dof));

  // Add objectives.
  NloptClosestParemeterObjective objective;
  objective.Initialize("y", problem.E(), parameter - problem.x0());
  lp.AddObjective(&objective);

  // Add constraints.
  const Eigen::MatrixXd F = problem.F();
  const Eigen::VectorXd g = problem.g();
  NloptLinearFunction Fyg;
  const double inf = NloptFunction::Infinity();
  const int bounded_num = static_cast<int>(F.rows());
  Fyg.Initialize("y", problem.bounded_parameter_num(),
    Eigen::VectorXd::Constant(bounded_num, -inf), g, F,
    Eigen::VectorXd::Zero(bounded_num));
  lp.AddConstraint(&Fyg);

  // Solve.
  lp.Solve();

  // Get solution. This is guaranteed to be feasible.
  SetReducedDesignParameter(lp.x("y", reduced_dof));
}

void Copter::InitializeController() {
  // Initialize rotor effective matrix.
  const int rotor_num = designer_.NumOfRotors();
  rotor_effective_matrix_ = Eigen::MatrixXd::Zero(6, rotor_num);
  for (int i = 0; i < rotor_num; ++i) {
    rotor_effective_matrix_.col(i).head(3) = rotors_[i].Thrust(1.0);
    rotor_effective_matrix_.col(i).tail(3) = rotors_[i].Torque(1.0);
  }
  // Initialize the controller.
  current_flight_mode_ = kPosition;
  // Position target: x, y, z, yaw (in radians).
  const double yaw_target = RoundAngle(0.0);
  current_flight_mode_target_ = Eigen::Vector4d(0.0, 0.0, -0.75, yaw_target);

  // Find the trim condition.
  Eigen::VectorXd x0, u0;
  if (!FindTrimCondition(x0, u0)) {
    std::cout << "Error: Failed to find a trim condition." << std::endl;
    exit(0);
  }
  // Update x0 with the target position.
  x0.head(3) = current_flight_mode_target_.head(3);
  x0(5) = current_flight_mode_target_(3);
  Eigen::MatrixXd A, B;
  LinearizeStateSpaceModel(x0, u0, A, B);
  const Eigen::MatrixXd Q = Eigen::VectorXd::Constant(12, 10.0).asDiagonal();
  const Eigen::MatrixXd R = Eigen::VectorXd::Constant(
    rotor_num, 1.0).asDiagonal();
  lqr_position_controller_.Initialize(A, B, Q, R, x0, u0);

  // For velocity mode. The parameters below are manually tweaked and only
  // works for quadcopter.xml.
  pid_climb_rate_controller_.Initialize(1, 1, 1.0, 0.0, 0.0);
  pid_roll_controller_.Initialize(1, 1, 10.0, 0.0, 1.0);
  pid_pitch_controller_.Initialize(1, 1, 10.0, 0.0, 1.0);
  pid_x_rate_controller_.Initialize(1, 1, -0.15, 0.0, 0.0);
  pid_y_rate_controller_.Initialize(1, 1, 0.07, 0.0, 0.0);
}

void Copter::Advance(const double dt) {
  const int rotor_num = static_cast<int>(rotors_.size());
  Eigen::VectorXd thrust = Eigen::VectorXd::Zero(rotor_num);
  switch (current_flight_mode_) {
    case kPosition: {
      thrust = PositionControllerOutput(dt);
      break;
    }
    case kVelocity: {
      thrust = VelocityControllerOutput(dt);
      break;
    }
    default: {
      std::cout << "Error: Unknown flight mode." << std::endl;
      exit(0);
    }
  }

  // - Use thrusts to update rigid body simulation and rotor phase.
  const double gravity = copter_body_.mass() * 9.81;
  double total_current = 0.0;
  for (int i = 0; i < rotor_num; ++i) {
    Rotor& rotor = rotors_[i];
    rotor.Advance(thrust(i), dt);
    const Eigen::Vector3d spinning_torque = rotor.SpinningTorque(thrust(i));
    total_current += spinning_torque.norm() / rotor.torque_to_current_ratio();

    copter_body_.ApplyForce(
      copter_body_.BodyVectorToWorldVector(rotor.Thrust(thrust(i))),
      copter_body_.BodyPointToWorldPoint(rotor.position())
    );
    copter_body_.ApplyTorque(
      copter_body_.BodyVectorToWorldVector(spinning_torque));
  }
  battery_.Advance(total_current, dt);
  copter_body_.ApplyForce(gravity * Eigen::Vector3d::UnitZ(),
    copter_body_.position());
  copter_body_.Advance(dt);

  // - Update sensor.
  inertial_sensor_.Advance(copter_body_.orientation(),
    copter_body_.angular_velocity(), copter_body_.position(),
    copter_body_.velocity(), copter_body_.acceleration());
}

const Eigen::VectorXd Copter::GetCurrentState() const {
  // To make it more realistic, we should use data from the inertial sensor
  // instead of copter_body, although they are the same when the sensor is
  // perfectly clean.
  Eigen::VectorXd state(12);
  // xyz.
  state.head(3) = inertial_sensor_.position();

  // rpy.
  const Eigen::Vector3d rpy = inertial_sensor_.rpy();
  state.segment(3, 3) = rpy;

  // uvw.
  const Eigen::Matrix3d RT = RollPitchYawToRotation(rpy).transpose();
  state.segment(6, 3) = RT * inertial_sensor_.velocity();

  // rpy_dot.
  state.tail(3) = AngularRateToEulerRate(rpy, inertial_sensor_.angular_rate());

  return state;
}

const Eigen::VectorXd Copter::StateSpaceModel(const Eigen::VectorXd& x,
  const Eigen::VectorXd& u) const {
  assert(x.size() == 12u);
  assert(static_cast<int>(u.size()) == static_cast<int>(rotors_.size()));
  const Eigen::Vector3d xyz = x.head(3);
  const Eigen::Vector3d rpy = x.segment(3, 3);
  const Eigen::Vector3d uvw = x.segment(6, 3);
  const Eigen::Vector3d rpy_dot = x.tail(3);

  Eigen::VectorXd x_dot(12); x_dot.setZero();
  // Compute xyz_dot.
  const Eigen::Matrix3d R = RollPitchYawToRotation(rpy);
  const Eigen::Vector3d xyz_dot = R * uvw;
  x_dot.head(3) = xyz_dot;

  // rpy_dot is given.
  x_dot.segment(3, 3) = rpy_dot;

  // Compute uvw_dot.
  // Recall that xyz_dot = R * uvw.
  // => R' * xyz_dot = uvw.
  // => \dot{R}' * xyz_dot + R' * xyz_ddot = uvw_dot.
  // => -R'[w] * xyz_dot + R' * xyz_ddot = uvw_dot.
  // (Furthermore, Rw_B = w => [w] = R[w_B]R'.)
  // => -[w_B]uvw + R' * xyz_ddot = uvw_dot.
  const Eigen::Vector3d body_w = EulerRateToBodyAngularRate(rpy, rpy_dot);
  const double m = copter_body_.mass();
  // Compute the force in the body frame.
  Eigen::Vector3d body_force = Eigen::Vector3d::Zero();
  // Gravity.
  const double g = 9.81;
  const Eigen::Matrix3d RT = R.transpose();
  body_force += RT * (m * g * Eigen::Vector3d::UnitZ());
  // Thrusts.
  const Eigen::VectorXd body_thrust_and_torque = rotor_effective_matrix_ * u;
  body_force += body_thrust_and_torque.head(3);
  const Eigen::Vector3d uvw_dot = uvw.cross(body_w) + body_force / m;
  x_dot.segment(6, 3) = uvw_dot;

  // Compute rpy_ddot. Use Euler's rotation equations.
  const Eigen::Vector3d body_torque = body_thrust_and_torque.tail(3);
  const Eigen::Vector3d body_w_dot = copter_body_.inertia_body_inv() *
    (body_torque - body_w.cross(copter_body_.inertia_body() * body_w));
  const Eigen::Matrix3d L = EulerRateToBodyAngularRateMatrix(rpy);
  const Eigen::Matrix3d L_dot = EulerRateToBodyAngularRateMatrixDerivative(
    rpy, rpy_dot);
  const Eigen::Vector3d rpy_ddot = L.inverse()
    * (body_w_dot - L_dot * rpy_dot);
  x_dot.tail(3) = rpy_ddot;

  return x_dot;
}

void Copter::LinearizeStateSpaceModel(const Eigen::VectorXd& x0,
  const Eigen::VectorXd& u0, Eigen::MatrixXd& A, Eigen::MatrixXd& B) const {
  assert(x0.size() == 12u);
  assert(static_cast<int>(u0.size()) == static_cast<int>(rotors_.size()));
  A = Eigen::MatrixXd::Zero(12, 12);
  const int rotor_num = static_cast<int>(rotors_.size());
  B = Eigen::MatrixXd::Zero(12, rotor_num);

  const Eigen::Vector3d xyz = x0.head(3);
  const Eigen::Vector3d rpy = x0.segment(3, 3);
  const Eigen::Vector3d uvw = x0.segment(6, 3);
  const Eigen::Vector3d rpy_dot = x0.tail(3);

  const Eigen::Matrix3d R = RollPitchYawToRotation(rpy);
  // Compute dxyz_dot/drpy.
  const Eigen::Matrix3Xd dR_drpy =
    RollPitchYawToRotationMatrixPartialDerivatives(rpy);
  A.topRows(3).col(3) = dR_drpy.leftCols(3) * uvw;
  A.topRows(3).col(4) = dR_drpy.middleCols(3, 3) * uvw;
  A.topRows(3).col(5) = dR_drpy.rightCols(3) * uvw;
  // Compute dxyz_dot/duvw.
  A.topRows(3).middleCols(6, 3) = R;

  // drpy_dot/drpy_dot.
  A.middleRows(3, 3).rightCols(3) = Eigen::Matrix3d::Identity();

  // Compute uvw_dot.
  // Recall that xyz_dot = R * uvw.
  // => R' * xyz_dot = uvw.
  // => \dot{R}' * xyz_dot + R' * xyz_ddot = uvw_dot.
  // => -R'[w] * xyz_dot + R' * xyz_ddot = uvw_dot.
  // (Furthermore, Rw_B = w => [w] = R[w_B]R'.)
  // => -[w_B]uvw + R' * xyz_ddot = uvw_dot.
  const Eigen::Vector3d body_w = EulerRateToBodyAngularRate(rpy, rpy_dot);
  const Eigen::Matrix3Xd d_body_w =
    EulerRateToBodyAngularRatePartialDerivatives(rpy, rpy_dot);
  // drpy.
  A.block(6, 3, 3, 3) += SkewMatrix(uvw) * d_body_w.leftCols(3);
  // drpy_dot.
  A.block(6, 9, 3, 3) += SkewMatrix(uvw) * d_body_w.middleCols(3, 3);
  // duvw.
  A.middleRows(6, 3).middleCols(6, 3) += -SkewMatrix(body_w);
  // body_force / m: gravity.
  const double m = copter_body_.mass();
  const double g = 9.81;
  const Eigen::Vector3d Z = Eigen::Vector3d::UnitZ();
  A.middleRows(6, 3).col(3) += dR_drpy.leftCols(3).transpose() * g * Z;
  A.middleRows(6, 3).col(4) += dR_drpy.middleCols(3, 3).transpose() * g * Z;
  A.middleRows(6, 3).col(5) += dR_drpy.rightCols(3).transpose() * g * Z;
  // Thrusts.
  B.middleRows(6, 3) += rotor_effective_matrix_.topRows(3) / m;

  // Compute rpy_ddot. Use Euler's rotation equations.
  const Eigen::Matrix3Xd H = rotor_effective_matrix_.bottomRows(3);
  const Eigen::Matrix3d I0 = copter_body_.inertia_body();
  const Eigen::Matrix3d I0_inv = copter_body_.inertia_body_inv();
  const Eigen::Matrix3Xd dL_drpy =
    EulerRateToBodyAngularRateMatrixPartialDerivatives(rpy);
  const Eigen::Matrix3d L_inv = EulerRateToBodyAngularRateMatrixInverse(rpy);
  Eigen::Matrix3Xd dL_inv_drpy = Eigen::Matrix3Xd::Zero(3, 9);
  for (int i = 0; i < 3; ++i) {
    dL_inv_drpy.middleCols(i * 3, 3) =
      -L_inv * dL_drpy.middleCols(i * 3, 3) * L_inv;
  }
  const Eigen::Matrix3d L_dot =
    EulerRateToBodyAngularRateMatrixDerivative(rpy, rpy_dot);
  const Eigen::Matrix3Xd dL_dot =
    EulerRateToBodyAngularRateMatrixDerivativePartialDerivatives(rpy, rpy_dot);
  B.bottomRows(3) = L_inv * I0_inv * H;
  // duvw.
  const Eigen::Vector3d body_torque = H * u0;
  const Eigen::Vector3d body_w_dot = I0_inv
    * (body_torque - body_w.cross(I0 * body_w));
  const Eigen::Matrix3d d_body_w_dot_d_rpy = I0_inv *
    (SkewMatrix(I0 * body_w) - SkewMatrix(body_w) * I0) * d_body_w.leftCols(3);
  for (int i = 0; i < 3; ++i) {
    A.bottomRows(3).col(3 + i) += dL_inv_drpy.middleCols(3 * i, 3)
      * (body_w_dot - L_dot * rpy_dot) + L_inv * (d_body_w_dot_d_rpy.col(i)
        - dL_dot.middleCols(3 * i, 3) * rpy_dot);
  }
  // drpy_dot.
  const Eigen::Matrix3d d_body_w_dot_d_rpy_dot = I0_inv *
    (SkewMatrix(I0 * body_w) - SkewMatrix(body_w) * I0) *
    d_body_w.rightCols(3);
  A.bottomRows(3).rightCols(3) += L_inv * (d_body_w_dot_d_rpy_dot - L_dot);
  for (int i = 0; i < 3; ++i) {
    A.bottomRows(3).col(9 + i) +=
      -L_inv * dL_dot.middleCols(9 + 3 * i, 3) * rpy_dot;
  }
}

bool Copter::FindTrimCondition(Eigen::VectorXd& x, Eigen::VectorXd& u) const {
  // Recall that x = [X, Y, Z,
  //  roll, pitch, yaw,
  //  U, V, W,
  //  roll_dot, pitch_dot, yaw_dot].
  // Note that the dynamic does not depend on the choice of X, Y, Z, yaw, and
  // f(x, u) = 0, we can simplify x so that it has much fewer parameters:
  // x = [0, 0, 0, roll, pitch, 0, 0, 0, 0, 0, 0, 0].
  const int rotor_num = static_cast<int>(rotors_.size());
  // If the dimension does not match, replace them with zero.
  if (x.size() != 12u) { x = Eigen::VectorXd::Zero(12); }
  if (static_cast<int>(u.size()) != rotor_num) {
    u = Eigen::VectorXd::Zero(rotor_num);
  }

  // Zero out x except for roll and pitch.
  const double init_roll = x(3), init_pitch = x(4);
  x.setZero(); x(3) = init_roll; x(4) = init_pitch;

  // Newton's method.
  // unknowns: [roll, pitch, u0, u1, u2, ...].
  Eigen::VectorXd y = Eigen::VectorXd::Zero(2 + rotor_num);
  y(0) = x(3); y(1) = x(4);
  y.tail(rotor_num) = u;
  int iter = 0;
  const int max_iter = 1000;
  const double eps = 1e-8;
  while (true) {
    // Compute dy:
    // f(x + dx, u + du) \approx f(x, u) + Adx + Bdu = 0.
    Eigen::VectorXd xi = Eigen::VectorXd::Zero(12);
    xi(3) = y(0); xi(4) = y(1);
    const Eigen::VectorXd ui = y.tail(rotor_num);
    const Eigen::VectorXd b = -StateSpaceModel(xi, ui);
    Eigen::MatrixXd Ai, Bi;
    LinearizeStateSpaceModel(xi, ui, Ai, Bi);
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(12, 2 + rotor_num);
    A.col(0) = Ai.col(3); A.col(1) = Ai.col(4);
    A.rightCols(rotor_num) = Bi;

    // Now solve A * dy = b.
    const Eigen::VectorXd dy =
      A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

    // Determine if we should terminate.
    if (iter >= max_iter || dy.norm() <= eps) break;

    // Update y.
    y += dy;
    ++iter;
  }

  // Update x and u.
  x(3) = y(0); x(4) = y(1);
  u = y.tail(rotor_num);
  // Check if we succeed.
  return StateSpaceModel(x, u).norm() <= eps;
}

const Eigen::Matrix4d Copter::TransformPropellerToSimulationSpace(
  const int i) const {
  assert(i >= 0 && i < NumOfPropellers());
  const Rotor& r = rotors_[i];
  const Eigen::Quaterniond spin(Eigen::AngleAxisd(r.propeller_phase(),
    -Eigen::Vector3d::UnitZ()));
  const Eigen::Quaterniond align = Eigen::Quaterniond::FromTwoVectors(
    -Eigen::Vector3d::UnitZ(), r.direction());
  const Eigen::Vector3d translate = r.position() +
    r.propeller_height() * r.direction();
  return copter_body_.BodyToWorldTransform() *
    opengl_viewer::Translate(translate.cast<float>()).cast<double>() *
    opengl_viewer::Rotate((align * spin).cast<float>()).cast<double>();
}

const Eigen::VectorXd Copter::PositionControllerOutput(const double dt) {
  assert(current_flight_mode_ == kPosition);
  // The original point where we initialize our LQR controller.
  const Eigen::VectorXd x0 = lqr_position_controller_.x0();
  const Eigen::Vector3d pos0 = x0.head(3);
  const double heading0 = x0(5);

  // Get the current state.
  Eigen::VectorXd current_state = GetCurrentState();
  // Since the dynamic is invariant to the position, we translate the origin
  // to get the equivalent "state" when target_state overlaps x0.
  const Eigen::Vector3d pos_target = current_flight_mode_target_.head(3);
  const double heading_target = current_flight_mode_target_(3);
  current_state(5) = AngularDifference(
    heading_target, current_state(5)) + heading0;
  current_state.head(3) =
    Eigen::AngleAxisd(heading_target - heading0, -Eigen::Vector3d::UnitZ()) *
    (current_state.head(3) - pos_target) + pos0;

  // For LQR the target_state is unused (because we use x0 instead), so we
  // simply pass a dummy target state here.
  return lqr_position_controller_.Output(current_state,
    Eigen::VectorXd::Zero(12), dt);
}

const Eigen::VectorXd Copter::VelocityControllerOutput(const double dt) {
  assert(current_flight_mode_ == kVelocity);
  // - Collect sensor data.
  const double climb_rate = -inertial_sensor_.velocity().z();
  const double roll = inertial_sensor_.rpy()(0);
  const double pitch = inertial_sensor_.rpy()(1);
  const double x_rate = inertial_sensor_.velocity().x();
  const double y_rate = inertial_sensor_.velocity().y();

  // - Determine the current state.
  const Eigen::VectorXd climb_input = DoubleToVectorXd(climb_rate);
  const Eigen::VectorXd roll_input = DoubleToVectorXd(roll);
  const Eigen::VectorXd pitch_input = DoubleToVectorXd(pitch);
  const Eigen::VectorXd x_rate_input = DoubleToVectorXd(x_rate);
  const Eigen::VectorXd y_rate_input = DoubleToVectorXd(y_rate);

  // - Feed controller with the current state and target.
  const Eigen::VectorXd climb_target = DoubleToVectorXd(
    -current_flight_mode_target_.z());
  const double climb_thrust = VectorXdToDouble(
    pid_climb_rate_controller_.Output(climb_input, climb_target, dt));

  const Eigen::VectorXd x_rate_target = DoubleToVectorXd(0);
  const Eigen::VectorXd pitch_target =
    pid_x_rate_controller_.Output(x_rate_input, x_rate_target, dt);

  const Eigen::VectorXd y_rate_target = DoubleToVectorXd(0);
  const Eigen::VectorXd roll_target =
    pid_y_rate_controller_.Output(y_rate_input, y_rate_target, dt);

  const double roll_thrust = VectorXdToDouble(
    pid_roll_controller_.Output(roll_input, roll_target, dt));

  const double pitch_thrust = VectorXdToDouble(
    pid_pitch_controller_.Output(pitch_input, pitch_target, dt));

  // - Controller returns rotor thrusts.
  const int rotor_num = static_cast<int>(rotors_.size());
  const double gravity = copter_body_.mass() * 9.81;
  const double base_thrust = (climb_thrust + gravity) / rotor_num;
  return (Eigen::Vector4d() <<
    base_thrust + pitch_thrust, base_thrust - roll_thrust,
    base_thrust - pitch_thrust, base_thrust + roll_thrust).finished();
}

} // copter_simulation