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
#ifndef _COPTER_SIMULATION_BATTERY_H_
#define _COPTER_SIMULATION_BATTERY_H_

namespace copter_simulation {

class Battery {
public:
  Battery() : total_capacity_(0.0), current_capacity_(0.0), last_current_(0.0)
    {}

  // Unit: mAh.
  void Initialize(const double total_capacity) {
    total_capacity_ = total_capacity;
    current_capacity_ = total_capacity;
    last_current_ = 0.0;
  }

  // current in A, dt in second.
  void Advance(const double current, const double dt) {
    current_capacity_ -= current * dt / 3.6;  // As -> mAh.
    last_current_ = current * 1000.0;
    if (current_capacity_ < 0.0) current_capacity_ = 0.0;
  }

  const double total_capacity() const { return total_capacity_; }
  const double current_capacity() const { return current_capacity_; }
  const double Percent() const { return current_capacity_ / total_capacity_; }
  // Return the estimated time in minutes. Negative numbers means N/A.
  const double EstimateRemainingTime() const {
    return last_current_ > 0.0 ?
      current_capacity_ / last_current_ * 60.0 : -1.0;
  }

private:
  double total_capacity_; // in mAh.
  double current_capacity_; // in mAh.
  double last_current_; // Used to estimate the remaining active time, in mA.
};

} // copter_simulation.

#endif  // _COPTER_SIMULATION_BATTERY_H_