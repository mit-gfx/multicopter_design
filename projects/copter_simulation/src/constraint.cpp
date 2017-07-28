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
#include "constraint.h"
#include <assert.h>

namespace copter_simulation {

Constraint::Constraint()
  : lower_bound_(-Constraint::Infinity()),
  upper_bound_(Constraint::Infinity()),
  type_(ConstraintType::kUnbound),
  info_() {}

void Constraint::Initialize(const double lower_bound,
  const double upper_bound) {
  // Clamp lower_bound and upper_bound.
  lower_bound_ = lower_bound;
  upper_bound_ = upper_bound;
  const double inf = Constraint::Infinity();
  if (lower_bound_ < -inf) lower_bound_ = -inf;
  if (upper_bound_ > inf) upper_bound_ = inf;
  assert(lower_bound_ <= upper_bound_);

  // Determine type based on the values of lower_bound and upper_bound.
  if (lower_bound_ == upper_bound_) {
    assert(lower_bound_ > -inf && upper_bound_ < inf);
    type_ = kEqual;
  } else if (lower_bound_ == -inf && upper_bound_ < inf) {
    type_ = kUpper;
  } else if (lower_bound_ == -inf && upper_bound_ == inf) {
    type_ = kUnbound;
  } else if (lower_bound_ > -inf && upper_bound_ < inf) {
    type_ = kBound;
  } else if (lower_bound_ > -inf && upper_bound_ == inf) {
    type_ = kLower;
  } else {
    // This should never happen.
    assert(false);
  }

  // Clear the data structure.
  info_.clear();
}

void Constraint::AddVariable(const std::string& name, const int index,
  const double coefficient) {
  info_[name][index] = coefficient;
}

const std::unordered_set<std::string> Constraint::AllComponentNames() const {
  std::unordered_set<std::string> all_names; all_names.clear();
  for (const auto& pair : info_) {
    all_names.insert(pair.first);
  }
  return all_names;
}

const std::unordered_map<int, double> Constraint::AllCoefficients(
  const std::string& name) const {
  return info_.at(name);
}

} // copter_simulation