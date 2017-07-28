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
#ifndef _COPTER_VIEWER_COPTER_VIEWER_H_
#define _COPTER_VIEWER_COPTER_VIEWER_H_

#include <vector>
#include "copter_animator.h"
#include "copter_imgui_wrapper.h"
#include "copter_simulation.h"
#include "copter_viewer_keyboard_handler.h"
#include "copter_viewer_timer.h"
#include "flight_target_animator.h"
#include "light_animator.h"
#include "opengl_viewer.h"

namespace copter_viewer {

enum CopterViewerPhase { kDesignPhase = 0, kSimulationPhase };

class CopterViewer {
public:
  CopterViewer();

  void Initialize(const int fps, const std::string& copter_file_name);
  void Run();
  void Cleanup();

  const CopterViewerPhase phase() const { return phase_; }
  void set_phase(const CopterViewerPhase phase) { phase_ = phase; }
  const copter_simulation::Copter& copter() const { return copter_; }
  copter_simulation::Copter& copter() { return copter_; }

  const CopterViewerKeyboardHandler& keyboard_handler() const {
    return keyboard_handler_;
  }

private:
  void AddGround();
  void AddAxis();
  void AddCopter(const std::string& copter_file_name);
  void AddFlightTarget();
  void AddLight();

  CopterViewerPhase phase_;
  copter_simulation::Copter copter_;

  // Viewer related members.
  opengl_viewer::Viewer& viewer_;
  std::vector<CopterAnimator> copter_animators_;
  FlightTargetAnimator flight_target_animator_;
  LightAnimator light_animator_;
  CopterViewerTimer timer_;
  CopterImGuiWrapper imgui_wrapper_;
  // User interaction.
  CopterViewerKeyboardHandler keyboard_handler_;
};

}

#endif // _COPTER_VIEWER_COPTER_VIEWER_H_