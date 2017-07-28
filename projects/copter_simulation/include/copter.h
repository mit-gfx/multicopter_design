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
#ifndef _COPTER_SIMULATION_COPTER_H_
#define _COPTER_SIMULATION_COPTER_H_

#include <unordered_map>
#include "copter_designer.h"
#include "inertial_sensor.h"
#include "lqr_controller.h"
#include "pid_controller.h"

namespace copter_simulation {

enum FlightMode {
  kPosition = 0, kVelocity
};

class Copter {
public:
  Copter();

  void Initialize(const std::string& file_name);
  // In the design phase, use the following functions to explore the design
  // space.
  const Eigen::VectorXd GetFullDesignParameter() const;
  const Eigen::VectorXd GetReducedDesignParameter() const;
  // Keep the existing design if parameter is infeasible.
  void SetReducedDesignParameter(const Eigen::VectorXd& parameter);
  // Given a full parameter, we will figure out the closest feasible reduced
  // parameter, and then forward it to SetReducedDesignParameter.
  void SetFullDesignParameter(const Eigen::VectorXd& parameter);
  // Once your design is fixed, call this function to initialize controllers.
  void InitializeController();
  const CopterDesigner& designer() const { return designer_; }

  // Simulation.
  void Advance(const double dt);

  const Battery& battery() const { return battery_; }
  const InertialSensor& inertial_sensor() const { return inertial_sensor_; }
  const TriRigidBodyUnion& copter_body() const { return copter_body_; }
  const Rotor& rotor(const int i) const { return rotors_[i]; }
  const int NumOfRigidComponents() const {
    return designer_.NumOfCopterRigidComponent();
  }
  const int NumOfRotors() const { return designer_.NumOfRotors(); }
  const int NumOfPropellers() const { return designer_.NumOfPropellers(); }

  // State space model. Define the copter state x as a 12d vector:
  // x = [X, Y, Z, roll, pitch, yaw, U, V, W, roll_dot, pitch_dot, yaw_dot].
  // This is inspired from Dr. Andy Barry's thesis:
  // http://abarry.org/Barry16.pdf.
  // Define u as a Nd vector where N is the number of rotors.
  // The dynamic can be compactly represented as x_dot = f(x, u).

  // Get x from the current state.
  const Eigen::VectorXd GetCurrentState() const;
  // Implement x_dot = f(x, u).
  const Eigen::VectorXd StateSpaceModel(const Eigen::VectorXd& x,
    const Eigen::VectorXd& u) const;
  // Linearize the state space model at (x0, u0):
  // f(x, u) \approx f(x0, u0) + A(x - x0) + B(u - u0).
  // This function has passed my numerical tests (Test code not included).
  void LinearizeStateSpaceModel(const Eigen::VectorXd& x0,
    const Eigen::VectorXd& u0, Eigen::MatrixXd& A, Eigen::MatrixXd& B) const;
  // Find x and u such that f(x, u) = 0. Initial guesses of x and u are
  // requested and the final solutions are written back to x and u.
  bool FindTrimCondition(Eigen::VectorXd& x, Eigen::VectorXd& u) const;

  // Visualization.
  const TriMesh ReferenceRigidMesh(const int i) const {
    return designer_.ReferenceRigidComponent(i)
      .reference_rigid_body.body_space_mesh();
  }
  const CopterRigidComponentName RigidMeshType(const int i) const {
    return designer_.ReferenceRigidComponent(i).type;
  }
  const TriMesh PropellerInModelSpace() const {
    return designer_.PropellerInModelSpace();
  }
  const Eigen::Matrix4d TransformRigidToDesignSpace(const int i) const {
    return designer_.TransformRigidToDesignSpace(i);
  }
  const Eigen::Matrix4d TransformPropellerToDesignSpace(const int i) const {
    return designer_.TransformPropellerToDesignSpace(i);
  }
  const Eigen::Matrix4d TransformRigidToSimulationSpace(const int i) const {
    return copter_body_.BodyToWorldTransform()
      * designer_.TransformRigidToDesignBodySpace(i);
  }
  const Eigen::Matrix4d TransformPropellerToSimulationSpace(const int i) const;

  // Interaction.
  const FlightMode current_flight_mode() const { return current_flight_mode_; }
  void set_current_flight_mode(const FlightMode mode) {
    current_flight_mode_ = mode;
  }
  const Eigen::VectorXd current_flight_mode_target() const {
    return current_flight_mode_target_;
  }
  void set_current_flight_mode_target(const Eigen::VectorXd& target) {
    current_flight_mode_target_ = target;
  }

private:
  // Compute the output thrust in a particular mode.
  const Eigen::VectorXd PositionControllerOutput(const double dt);
  const Eigen::VectorXd VelocityControllerOutput(const double dt);

  void SyncCopterFromDesign();

  // Designer.
  CopterDesigner designer_;

  // Simulation. We will sync them with designer whenever the design parameters
  // are changed.
  Battery battery_;
  TriMesh propeller_;
  InertialSensor inertial_sensor_;
  std::vector<Rotor> rotors_;
  TriRigidBodyUnion copter_body_;

  // Controller. Once the flight mode and target are specified (hard coded now)
  // the controller will try to steer the copter to satisfy the target. In the
  // Advance function the program looks like this:
  // - Collect sensor data.
  // - Determine the current state.
  // - Feed controller with the current state and target.
  // - Controller returns rotor thrusts.
  // - Use thrusts to update rigid body simulation and rotor phase.
  // - Update sensor.
  //
  // This matrix maps rotor inputs to body thrusts and torques.
  Eigen::MatrixXd rotor_effective_matrix_;
  PidController pid_climb_rate_controller_;
  PidController pid_roll_controller_, pid_pitch_controller_;
  PidController pid_x_rate_controller_, pid_y_rate_controller_;
  LqrController lqr_position_controller_;
  FlightMode current_flight_mode_;
  Eigen::VectorXd current_flight_mode_target_;
};

} // copter_simulation

#endif  // _COPTER_SIMULATION_COPTER_H_