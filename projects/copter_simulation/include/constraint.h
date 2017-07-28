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
#ifndef _COPTER_SIMULATION_CONSTRAINT_H_
#define _COPTER_SIMULATION_CONSTRAINT_H_

#include <string>
#include <unordered_map>
#include <unordered_set>

namespace copter_simulation {

enum ConstraintType { kEqual = 0, kLower, kUpper, kBound, kUnbound };

class Constraint {
public:
  Constraint();

  void Initialize(const double lower_bound, const double upper_bound);
  void AddVariable(const std::string& name, const int index,
    const double coefficient);

  const ConstraintType type() const { return type_; }
  const double lower_bound() const { return lower_bound_; }
  const double upper_bound() const { return upper_bound_; }
  const std::unordered_set<std::string> AllComponentNames() const;
  const std::unordered_map<int, double> AllCoefficients(
    const std::string& name) const;

  static const double Infinity() { return 1e20; }
private:
  double lower_bound_, upper_bound_;
  // type_ = kEqual if lower_bound_ == upper_bound_.
  // type_ = kLower if upper_bound_ == inf && lower_bound_ > -inf.
  // type_ = kUpper if lower_bound_ == -inf && upper_bound_ < inf.
  // type_ = kBound if lower_bound_ > -inf && upper_bound_ < inf.
  // type_ = kUnbound if lower_bound_ == -inf && upper_bound_ == inf.
  ConstraintType type_;

  // info_[component_name][index] = coefficient.
  std::unordered_map<std::string, std::unordered_map<int, double>> info_;
};

} // copter_simulation

#endif  // _COPTER_SIMULATION_CONSTRAINT_H_
