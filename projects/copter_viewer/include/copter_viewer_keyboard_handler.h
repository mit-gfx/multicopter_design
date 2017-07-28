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
#ifndef _COPTER_VIEWER_COPTER_VIEWER_KEYBOARD_HANDLER_H_
#define _COPTER_VIEWER_COPTER_VIEWER_KEYBOARD_HANDLER_H_

#include "keyboard_handler.h"

namespace copter_viewer {

class CopterViewer;

class CopterViewerKeyboardHandler : public opengl_viewer::KeyboardHandler {
public:
  CopterViewerKeyboardHandler();

  void Initialize(const bool paused, CopterViewer* parent_viewer);

  const bool paused() const { return paused_; }

  void KeyCallback(const int key, const int action);

private:
  bool paused_;
  CopterViewer* parent_viewer_;
};

} // copter_viewer

#endif  // _COPTER_VIEWER_COPTER_VIEWER_KEYBOARD_HANDLER_H_