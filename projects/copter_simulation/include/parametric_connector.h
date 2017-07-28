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
#ifndef _COPTER_SIMULATION_PARAMETRIC_CONNECTOR_H_
#define _COPTER_SIMULATION_PARAMETRIC_CONNECTOR_H_

#include "tri_mesh.h"

namespace copter_simulation {

// The following function works ONLY for tube_mount.obj I provided in
// resources/mesh/. In practice, we actually used SolidWorks to parametrize the
// tube mount and we wrote a short SolidWorks configuration file to change the
// normal of the tube mount and exported it to a printable STL file. However,
// as I don't want to add dependencies on SolidWorks in this project, I wrote
// this C++ function to manually adjust the points in the tube mount mesh, and
// the returned mesh is not tested in a read 3D printer. If you want to
// fabricate your own tube mount model, consider using SolidWorks/AutoCAD/
// OnShape, etc.
//
// You should specify the normal of the new tube mount. I will normalize the
// input argument so you don't have to pass in a unit vector. Normal =
// (0, 0, -1) corresponds to the unchanged mesh in tube_mount.obj.
const TriMesh ParametricTubeMount(const Eigen::Vector3d& normal);

} // copter_simulation

#endif  // _COPTER_SIMULATION_PARAMETRIC_CONNECTOR_H_