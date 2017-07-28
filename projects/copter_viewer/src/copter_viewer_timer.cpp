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
#include "copter_viewer_timer.h"
#include "copter_viewer.h"

namespace copter_viewer {

CopterViewerTimer::CopterViewerTimer()
  : fps_(25), dt_(0.04f), current_frame_(0), parent_viewer_(NULL) {}

void CopterViewerTimer::Initialize(const int fps,
  CopterViewer* parent_viewer) {
  fps_ = fps;
  dt_ = 1.0f / fps;
  current_frame_ = 0;
  parent_viewer_ = parent_viewer;
}

const float CopterViewerTimer::CurrentTime() {
  // Pause forever if we are in the design phase.
  if (parent_viewer_->phase() == kDesignPhase) {
    return 0.0;
  }
  // Otherwise, simulate forward.
  const float current_time = current_frame_ * dt_;
  if (!parent_viewer_->keyboard_handler().paused()) {
    // Advance.
    parent_viewer_->copter().Advance(dt_);
    ++current_frame_;
  }
  return current_time;
}

} // copter_viewer