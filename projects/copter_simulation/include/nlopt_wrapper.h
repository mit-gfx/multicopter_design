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
// Usage:
// - Create an Option instance to record all options. Valid options include:
//  "algorithm" (int): NLOPT_LD_SLSQP by default.
//  "max eval" (int): -1 by default, which means this check is disabled.
//  "problem type" (string): "min" or "max". "min" by default.
//  "tolerance" (float): 1e-6 by default.
// - Call Initialize(option);
// - Call AddVariable, AddObjective, and AddConstraint.
// - Call Solve.
// - Call x() to extract the solution.
#ifndef _COPTER_SIMULATION_NLOPT_WRAPPER_H_
#define _COPTER_SIMULATION_NLOPT_WRAPPER_H_

#include <string>
#include <unordered_map>
#include <vector>
#include "option.h"
#include "nlopt_function.h"

namespace copter_simulation {

class NloptWrapper {
public:
  NloptWrapper();

  void Initialize(const opengl_viewer::Option& options =
    opengl_viewer::Option());

  void AddVariable(const std::string& name, const double init_value = 0.0,
    const double lower = -NloptFunction::Infinity(),
    const double upper = NloptFunction::Infinity());
  void AddVariable(const std::string& name,
    const Eigen::VectorXd& init_value,
    const double lower = -NloptFunction::Infinity(),
    const double upper = NloptFunction::Infinity());
  void AddVariable(const std::string& name,
    const Eigen::VectorXd& init_value, const Eigen::VectorXd& lower,
    const Eigen::VectorXd& upper);
  const int NumOfVariables() const { return static_cast<int>(x_.size()); }
  const double x(const int i) const;
  const double x(const std::string& name) const;
  const Eigen::VectorXd x(const std::string& name, const int num) const;
  const int x_name_to_index(const std::string& name) const;

  void AddObjective(NloptFunction* objective) {
    objective_ = objective;
  }
  void AddConstraint(NloptFunction* constraint) {
    constraints_.push_back(constraint);
  }

  void Solve();

private:
  opengl_viewer::Option options_;

  std::vector<double> x_, x_lower_, x_upper_;
  std::vector<std::string> x_index_to_name_;
  std::unordered_map<std::string, int> x_name_to_index_;

  NloptFunction* objective_;
  std::vector<NloptFunction*> constraints_;
};

} // copter_simulation

#endif  // _COPTER_SIMULATION_NLOPT_WRAPPER_H_