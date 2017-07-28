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
#include "nlopt_wrapper.h"
#include <iostream>
#include "nlopt.h"

namespace copter_simulation {

enum NloptBoundType { kFree = 0, kLower, kUpper };

struct NloptFunctionInfo {
public:
  NloptFunctionInfo()
    : function(NULL), parent(NULL), row_num(0), bound_type(kFree) {}
  NloptFunctionInfo(NloptFunction* function, NloptWrapper* parent,
    const int row_num, const NloptBoundType bound_type)
    : function(function), parent(parent), row_num(row_num),
    bound_type(bound_type) {}

  NloptFunction* function;
  NloptWrapper* parent;
  int row_num;
  NloptBoundType bound_type;
};

static double NloptFunctionWrapper(unsigned int n, const double* x,
  double* gradient, void* data) {
  NloptFunctionInfo* info = reinterpret_cast<NloptFunctionInfo*>(data);
  // Translate x.
  const NloptWrapper* parent = info->parent;
  std::vector<double> x_segment(0);
  for (const auto& name : info->function->x_name()) {
    x_segment.push_back(x[parent->x_name_to_index(name)]);
  }
  const Eigen::VectorXd x_val = Eigen::Map<const Eigen::VectorXd>(
    x_segment.data(), x_segment.size());

  // Translate gradient.
  Eigen::VectorXd gradient_val = gradient ? Eigen::VectorXd::Zero(x_val.size())
    : Eigen::VectorXd::Zero(0);
  double value = info->function->Evaluate(info->row_num, x_val, gradient_val);
  switch (info->bound_type) {
    case kFree: {
      // Do nothing.
      break;
    }
    case kLower: {
      value = info->function->lower_bound()(info->row_num) - value;
      gradient_val *= -1;
      break;
    }
    case kUpper: {
      value -= info->function->upper_bound()(info->row_num);
      break;
    }
    default: {
      assert(false);
      break;
    }
  }
  // Collect gradient info.
  if (gradient) {
    // Clear gradients.
    std::memset(gradient, 0, sizeof(double) * n);
    const int x_num = info->function->x_dim();
    for (int i = 0; i < x_num; ++i) {
      gradient[parent->x_name_to_index(info->function->x_name(i))]
        = gradient_val(i);
    }
  }
  return value;
}

NloptWrapper::NloptWrapper()
  : options_(),
  x_(0), x_lower_(0), x_upper_(0),
  x_index_to_name_(0), x_name_to_index_(),
  objective_(NULL),
  constraints_(0) {
  // Default values.
  options_.SetIntOption("algorithm", NLOPT_LD_SLSQP);
  options_.SetIntOption("max eval", -1);
  options_.SetStringOption("problem type", "min");
  options_.SetFloatOption("tolerance", 1e-6f);
}

void NloptWrapper::Initialize(const opengl_viewer::Option& options) {
  // Int options.
  for (const auto& name : options.GetAllIntOptionNames()) {
    options_.SetIntOption(name, options.GetIntOption(name));
  }
  // String options.
  for (const auto& name : options.GetAllStringOptionNames()) {
    options_.SetStringOption(name, options.GetStringOption(name));
  }
  // Float options.
  for (const auto& name : options.GetAllFloatOptionNames()) {
    options_.SetFloatOption(name, options.GetFloatOption(name));
  }
}

void NloptWrapper::AddVariable(const std::string& name,
  const double init_value, const double lower, const double upper) {
  assert(x_name_to_index_.find(name) == x_name_to_index_.end());
  const int next_id = static_cast<int>(x_.size());
  x_.push_back(init_value);
  x_lower_.push_back(lower);
  x_upper_.push_back(upper);
  x_index_to_name_.push_back(name);
  x_name_to_index_[name] = next_id;
}

void NloptWrapper::AddVariable(const std::string& name,
  const Eigen::VectorXd& init_value, const double lower,
  const double upper) {
  AddVariable(name, init_value,
    Eigen::VectorXd::Constant(init_value.size(), lower),
    Eigen::VectorXd::Constant(init_value.size(), upper));
}

void NloptWrapper::AddVariable(const std::string& name,
  const Eigen::VectorXd& init_value, const Eigen::VectorXd& lower,
  const Eigen::VectorXd& upper) {
  assert(init_value.size() == lower.size());
  assert(init_value.size() == upper.size());
  const int num = static_cast<int>(init_value.size());
  for (int i = 0; i < num; ++i) {
    AddVariable(name + "_" + std::to_string(i), init_value(i),
      lower(i), upper(i));
  }
}

const double NloptWrapper::x(const int i) const {
  assert(i >= 0 && i < NumOfVariables());
  return x_[i];
}

const double NloptWrapper::x(const std::string& name) const {
  return x(x_name_to_index_.at(name));
}

const Eigen::VectorXd NloptWrapper::x(
  const std::string& name, const int num) const {
  Eigen::VectorXd value = Eigen::VectorXd::Zero(num);
  for (int i = 0; i < num; ++i) {
    value(i) = x(name + "_" + std::to_string(i));
  }
  return value;
}

const int NloptWrapper::x_name_to_index(const std::string& name) const {
  return x_name_to_index_.at(name);
}

void NloptWrapper::Solve() {
  // Set algorithm.
  nlopt_opt solver = nlopt_create(static_cast<nlopt_algorithm>(
    options_.GetIntOption("algorithm")), NumOfVariables());

  // Set variables.
  nlopt_set_lower_bounds(solver, x_lower_.data());
  nlopt_set_upper_bounds(solver, x_upper_.data());

  // Set the objective function.
  const std::string type = options_.GetStringOption("problem type");
  NloptFunctionInfo objective_info{ objective_, this, 0, kFree };
  if (type == "min") {
    nlopt_set_min_objective(solver, NloptFunctionWrapper, &objective_info);
  } else if (type == "max") {
    nlopt_set_max_objective(solver, NloptFunctionWrapper, &objective_info);
  } else {
    assert(false);
  }

  // Set constraints for the objective function.
  const double tol = options_.GetFloatOption("tolerance");
  const bool objective_has_lower_bound = objective_->HasFiniteLowerBound(0),
    objective_has_upper_bound = objective_->HasFiniteUpperBound(0),
    objective_has_equal_bound = objective_->HasEqualBound(0);
  NloptFunctionInfo objective_bound_info{ objective_, this, 0, kFree };
  if (objective_has_equal_bound) {
    objective_bound_info.bound_type = kUpper;
    nlopt_add_equality_constraint(
      solver, NloptFunctionWrapper, &objective_bound_info, tol);
  } else if (objective_has_lower_bound) {
    objective_bound_info.bound_type = kLower;
    nlopt_add_inequality_constraint(
      solver, NloptFunctionWrapper, &objective_bound_info, tol);
  } else if (objective_has_upper_bound) {
    objective_bound_info.bound_type = kUpper;
    nlopt_add_inequality_constraint(
      solver, NloptFunctionWrapper, &objective_bound_info, tol);
  }

  // Set constraints.
  const int constraint_num = static_cast<int>(constraints_.size());
  std::vector<std::vector<NloptFunctionInfo>> infos(constraint_num);
  for (int i = 0; i < constraint_num; ++i) {
    NloptFunction* constraint = constraints_[i];
    const int num = constraint->f_dim();
    infos[i].clear();
    infos[i].reserve(2 * num);
    for (int j = 0; j < num; ++j) {
      if (constraint->HasFiniteLowerBound(j) || constraint->HasEqualBound(j)) {
        infos[i].push_back({ constraint, this, j, kLower });
        nlopt_add_inequality_constraint(solver, NloptFunctionWrapper,
          &infos[i].back(), tol);
      }
      if (constraint->HasFiniteUpperBound(j) || constraint->HasEqualBound(j)) {
        infos[i].push_back({ constraint, this, j, kUpper });
        nlopt_add_inequality_constraint(solver, NloptFunctionWrapper,
          &infos[i].back(), tol);
      }
    }
  }

  // Set stopping criteria.
  nlopt_set_xtol_rel(solver, tol);
  nlopt_set_maxeval(solver, options_.GetIntOption("max eval"));

  // Optimization.
  double f_val = 0.0;
  nlopt_result result = nlopt_optimize(solver, x_.data(), &f_val);
  nlopt_destroy(solver);

  if (result < 0) {
    std::cout << "Warning: NLOPT does not find a solution." << std::endl;
    if (result != NLOPT_ROUNDOFF_LIMITED) exit(0);
  }
}

} // copter_simulation