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
#ifndef _COPTER_VIEWER_COPTER_ANIMATOR_H_
#define _COPTER_VIEWER_COPTER_ANIMATOR_H_

#include "animator.h"

namespace copter_viewer {

class CopterViewer;

class CopterAnimator : public opengl_viewer::Animator {
public:
  CopterAnimator();

  void Initialize(const int component_id, const CopterViewer* parent_viewer);

  const Eigen::Matrix4f AnimatedModelMatrix(const float t);

private:
  int id_;
  bool is_propeller_;
  const CopterViewer* parent_viewer_;
};

} // copter_viewer

#endif  // _COPTER_VIEWER_COPTER_ANIMATOR_H_