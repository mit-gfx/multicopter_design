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
#include "copter_viewer.h"

// Usage: ./copter_viewer [xml file name].
int main(int argc, char* argv[]) {
  std::string copter_xml_file_name = "";
  if (argc < 2) {
    copter_xml_file_name = "pentacopter.xml";
  } else {
    copter_xml_file_name = std::string(argv[1]);
  }

  copter_viewer::CopterViewer viewer;
  const int fps = 30;
  viewer.Initialize(fps, copter_xml_file_name);
  viewer.Run();
  viewer.Cleanup();
  return 0;
}