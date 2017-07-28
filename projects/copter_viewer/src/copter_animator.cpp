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
#include "copter_animator.h"
#include "copter_viewer.h"

namespace copter_viewer {

CopterAnimator::CopterAnimator()
  : Animator(), id_(-1), is_propeller_(false), parent_viewer_(NULL) {}

void CopterAnimator::Initialize(const int component_id,
  const CopterViewer* parent_viewer) {
  const int rigid_part_num = parent_viewer->copter().NumOfRigidComponents();
  is_propeller_ = (component_id >= rigid_part_num);
  id_ = is_propeller_ ? (component_id - rigid_part_num) : component_id;
  parent_viewer_ = parent_viewer;
}

const Eigen::Matrix4f CopterAnimator::AnimatedModelMatrix(
  const float t) {
  const copter_simulation::Copter& copter = parent_viewer_->copter();
  // Which phase are we in?
  if (parent_viewer_->phase() == kDesignPhase) {
    return is_propeller_ ?
      copter.TransformPropellerToDesignSpace(id_).cast<float>() :
      copter.TransformRigidToDesignSpace(id_).cast<float>();
  } else {
    // Simulation phase.
    return is_propeller_ ?
      copter.TransformPropellerToSimulationSpace(id_).cast<float>() :
      copter.TransformRigidToSimulationSpace(id_).cast<float>();
  }
}

} // copter_viewer