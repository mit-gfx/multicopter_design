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
#include "copter_viewer_keyboard_handler.h"
#include "copter_viewer.h"

namespace copter_viewer {

CopterViewerKeyboardHandler::CopterViewerKeyboardHandler()
  : paused_(false), parent_viewer_(NULL) {}

void CopterViewerKeyboardHandler::Initialize(
  const bool paused, CopterViewer* parent_viewer) {
  paused_ = paused;
  parent_viewer_ = parent_viewer;
}

void CopterViewerKeyboardHandler::KeyCallback(
  const int key, const int action) {
  // Pause.
  if (key == GLFW_KEY_P && action == GLFW_PRESS) paused_ = !paused_;

  // Adjust the current target of the copter.
  // TODO: make this code flight-mode dependent in the future.
  if (action == GLFW_PRESS || action == GLFW_REPEAT) {
    Eigen::VectorXd target =
      parent_viewer_->copter().current_flight_mode_target();
    switch (key) {
      case GLFW_KEY_LEFT: {
        target.y() -= 0.1;
        break;
      }
      case GLFW_KEY_RIGHT: {
        target.y() += 0.1;
        break;
      }
      case GLFW_KEY_UP: {
        target.x() += 0.1;
        break;
      }
      case GLFW_KEY_DOWN: {
        target.x() -= 0.1;
        break;
      }
      // Yaw needs to be clamped to [-pi, pi].
      case GLFW_KEY_A: {
        target.w() -= 0.1;
        target.w() = copter_simulation::RoundAngle(target.w());
        break;
      }
      case GLFW_KEY_D: {
        target.w() += 0.1;
        target.w() = copter_simulation::RoundAngle(target.w());
        break;
      }
      case GLFW_KEY_W: {
        target.z() -= 0.1;
        break;
      }
      case GLFW_KEY_S: {
        target.z() += 0.1;
        break;
      }
      default:
        break;
    }
    parent_viewer_->copter().set_current_flight_mode_target(target);
  }
}

} // copter_viewer