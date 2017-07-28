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
#ifndef _COPTER_SIMULATION_PARAMETRIZED_SHAPE_H_
#define _COPTER_SIMULATION_PARAMETRIZED_SHAPE_H_

#include <iostream>
#include <unordered_map>
#include "constraint.h"
#include "design_problem.h"
#include "tri_rigid_body.h"

namespace copter_simulation {

class ParametrizedShape {
public:
  ParametrizedShape() : name_(), reference_rigid_body_(),
    reference_parameter_(Eigen::VectorXd::Zero(0)) {}
  virtual ~ParametrizedShape() {}

  void Initialize(const std::string& name,
    const TriRigidBody& reference_rigid_body,
    const Eigen::VectorXd& reference_parameter) {
    name_ = name;
    reference_rigid_body_ = reference_rigid_body;
    reference_parameter_ = reference_parameter;
    // Add variables.
    if (all_reference_parameters_.find(name)
      != all_reference_parameters_.end()) {
      std::cout << "Warning: Multiple shapes are sharing the same name: "
        << name << std::endl;
    }
    all_reference_parameters_[name] = reference_parameter;
  }
  const std::string& name() const { return name_; }
  const int NumOfParameters() const {
    return static_cast<int>(reference_parameter_.size());
  }
  const TriRigidBody& reference_rigid_body() const {
    return reference_rigid_body_;
  }
  const Eigen::VectorXd reference_parameter() const {
    return reference_parameter_;
  }

  // Build the constraints. This should be done after all of the parametrized
  // shapes are initialized and all constraints are added.
  // If we flatten all_variables_ into a single vector x, then this function
  // returns A, b, C and d such that the constraints on x are:
  //   Ax = b, Cx <= d.
  // We can further simplify them into a 'reduced' parameter y, defined as:
  //   x := x0 + Ey.
  // where Ax0 = b and E spans the null space of A. In this way we can ignore
  // the equality constraints:
  //   Cx = Cx0 + CEy <= d.
  //   CEy <= d - Cx0.
  static const DesignProblem BuildConstraint();
  // Given a flattened vector x0, return the segment that corresponds to the
  // component name.
  static const Eigen::VectorXd ExtractParameter(const std::string& name,
    const Eigen::VectorXd& x0);
  static const std::unordered_map<std::string, Eigen::VectorXd>&
    all_reference_parameters() {
    return all_reference_parameters_;
  }

  virtual void AnalyzeConstraint() const = 0;
  // The default operator() decomposes Transform into rotation, scaling and
  // translation then apply them. In some cases this may be slow (e.g., your
  // transform is a translation, but decomposition still tries to do SVD and
  // find the rotation component), and you are encouraged to override it.
  virtual const TriRigidBody operator()(
    const Eigen::VectorXd& parameter) const;
  virtual const Eigen::Matrix4d Transform(
    const Eigen::VectorXd& parameter) const = 0;

protected:
  void AddConstraint(const Constraint& constraint) const {
    all_constraints_.push_back(constraint);
  }

private:
  std::string name_;

  // The reference parameter and rigid body are always consistent.
  TriRigidBody reference_rigid_body_;
  Eigen::VectorXd reference_parameter_;

  static std::unordered_map<std::string, Eigen::VectorXd>
    all_reference_parameters_;
  static std::unordered_map<std::string, int> reference_parameter_offsets_;
  static std::vector<Constraint> all_constraints_;
};

} // copter_simulation

#endif  // _COPTER_SIMULATION_PARAMETRIZED_SHAPE_H_