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
#include "parametrized_shape.h"

namespace copter_simulation {

std::unordered_map<std::string, Eigen::VectorXd>
ParametrizedShape::all_reference_parameters_ =
  std::unordered_map<std::string, Eigen::VectorXd>();

std::unordered_map<std::string, int>
ParametrizedShape::reference_parameter_offsets_ =
  std::unordered_map<std::string, int>();

std::vector<Constraint> ParametrizedShape::all_constraints_
  = std::vector<Constraint>(0);

const DesignProblem ParametrizedShape::BuildConstraint() {
  DesignProblem problem;
  // Figure out offsets first.
  int offset = 0;
  reference_parameter_offsets_.clear();
  for (const auto& pair : all_reference_parameters_) {
    reference_parameter_offsets_[pair.first] = offset;
    offset += static_cast<int>(pair.second.size());
  }
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(offset);
  // We assume reference parameters are feasible.
  for (const auto& pair : all_reference_parameters_) {
    x0.segment(reference_parameter_offsets_[pair.first],
      pair.second.size()) = pair.second;
  }
  // Now loop over all constraints.
  std::vector<Eigen::VectorXd> equality_constraints(0),
    inequality_constraints(0);
  for (const auto& constraint : all_constraints_) {
    // Skip if this is an unbounded constraint.
    if (constraint.type() == kUnbound) continue;
    // Collect the row information first. The last element is used for the rhs.
    Eigen::VectorXd row = Eigen::VectorXd::Zero(offset + 1);
    const std::unordered_set<std::string> all_names
      = constraint.AllComponentNames();
    for (const auto& name : all_names) {
      const std::unordered_map<int, double> all_coeff =
        constraint.AllCoefficients(name);
      for (const auto& pair : all_coeff) {
        row(reference_parameter_offsets_[name] + pair.first) = pair.second;
      }
    }
    // Now based on the type of the constraint, determine if we should put them
    // in equality or inequality constraints.
    switch (constraint.type()) {
      case kEqual: {
        row(offset) = constraint.lower_bound();
        equality_constraints.push_back(row);
        break;
      }
      case kLower: {
        row(offset) = constraint.lower_bound();
        row *= -1.0;
        inequality_constraints.push_back(row);
        break;
      }
      case kUpper: {
        row(offset) = constraint.upper_bound();
        inequality_constraints.push_back(row);
        break;
      }
      case kBound: {
        row(offset) = constraint.upper_bound();
        inequality_constraints.push_back(row);
        row(offset) = constraint.lower_bound();
        row *= -1.0;
        inequality_constraints.push_back(row);
        break;
      }
      default: {
        // This should not happen.
        assert(false);
      }
    }
  }
  // Build A, b, C, d.
  const int equality_num = static_cast<int>(equality_constraints.size());
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(equality_num, offset);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(equality_num);
  for (int i = 0; i < equality_num; ++i) {
    const Eigen::VectorXd& row = equality_constraints[i];
    A.row(i) = row.head(offset);
    b(i) = row(offset);
  }
  const int inequality_num = static_cast<int>(inequality_constraints.size());
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(inequality_num, offset);
  Eigen::VectorXd d = Eigen::VectorXd::Zero(inequality_num);
  for (int i = 0; i < inequality_num; ++i) {
    const Eigen::VectorXd& row = inequality_constraints[i];
    C.row(i) = row.head(offset);
    d(i) = row(offset);
  }

  problem.Initialize(A, b, C, d, x0);
  return problem;
}

const Eigen::VectorXd ParametrizedShape::ExtractParameter(
  const std::string& name, const Eigen::VectorXd& x0) {
  return x0.segment(reference_parameter_offsets_[name],
    all_reference_parameters_[name].size());
}

const TriRigidBody ParametrizedShape::operator()(
  const Eigen::VectorXd& parameter) const {
  assert(parameter.size() == reference_parameter_.size());
  TriRigidBody new_shape = reference_rigid_body_;
  new_shape.Transform(Transform(parameter));
  return new_shape;
}

} // copter_simulation