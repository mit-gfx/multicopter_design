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
#include "light_animator.h"
#include "copter_viewer.h"

namespace copter_viewer {

LightAnimator::LightAnimator()
  : Animator(), initial_position_(0.0f, 0.0f, 0.0f), parent_viewer_(NULL) {}

void LightAnimator::Initialize(const Eigen::Vector3f& initial_position,
  const CopterViewer* parent_viewer) {
  initial_position_ = initial_position;
  parent_viewer_ = parent_viewer;
}

const Eigen::Matrix4f LightAnimator::AnimatedModelMatrix(const float t) {
  // Fix the height, only move around the XoY plane.
  Eigen::Vector3f copter_center =
    parent_viewer_->copter().copter_body().position().cast<float>();
  copter_center.z() = 0.0f;
  return opengl_viewer::Translate(initial_position_ + copter_center);
}

} // copter_viewer