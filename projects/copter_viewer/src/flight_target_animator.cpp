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
#include "flight_target_animator.h"
#include "copter_viewer.h"

namespace copter_viewer {

FlightTargetAnimator::FlightTargetAnimator()
  : parent_viewer_(NULL) {}

void FlightTargetAnimator::Initialize(const CopterViewer* parent_viewer) {
  parent_viewer_ = parent_viewer;
}

const Eigen::Matrix4f FlightTargetAnimator::AnimatedModelMatrix(
  const float t) {
  // Get the current flight target.
  // TODO: this only works for the position mode.
  const Eigen::Vector4f target =
    parent_viewer_->copter().current_flight_mode_target().cast<float>();
  // Get position.
  const Eigen::Vector3f pos = target.head(3);
  const float heading = opengl_viewer::RadianToDegree(target(3));
  return opengl_viewer::Translate(pos) *
    opengl_viewer::Rotate(heading, 0.0f, 0.0f, 1.0f);
}

} // copter_viewer