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

namespace copter_viewer {

CopterViewer::CopterViewer() : phase_(kDesignPhase), copter_(),
  viewer_(opengl_viewer::Viewer::GetViewer()),
  copter_animators_(0), flight_target_animator_(),
  light_animator_(), timer_(), imgui_wrapper_(), keyboard_handler_() {}

void CopterViewer::Initialize(const int fps,
  const std::string& copter_file_name) {
  opengl_viewer::Option option;
  // Set parameters.
  option.SetIntOption("height", 1000);
  option.SetIntOption("width", 1600);
  option.SetVectorOption("background color", 0.86f, 0.88f, 0.90f, 1.0f);
  option.SetFloatOption("camera aspect ratio", 1600.0f / 1000.0f);
  option.SetVectorOption("camera pos", -1.6f, 0.8f, -1.6f);
  option.SetVectorOption("camera look at", 0.8f, -0.5f, 0.0f);
  option.SetVectorOption("camera up", 0.0f, 0.0f, -1.0f);
  option.SetFloatOption("camera pan speed", 0.004f);
  option.SetFloatOption("camera field of view", 45.0f);
  option.SetFloatOption("shadow acne bias", 0.005f);
  option.SetFloatOption("shadow sampling angle", 0.2f);
  option.SetIntOption("shadow sampling number", 2);
  // Initialize pointer options before we add them.
  timer_.Initialize(fps, this);
  option.SetPointerOption("timer", static_cast<void*>(&timer_));
  keyboard_handler_.Initialize(false, this);
  option.SetPointerOption("keyboard handler",
    static_cast<void*>(&keyboard_handler_));
  // Initialize ImGui.
  imgui_wrapper_.Initialize(this);
  option.SetPointerOption("imgui wrapper",
    static_cast<void*>(&imgui_wrapper_));
  viewer_.Initialize(option);
  
  // Now add static objects. First, the background.
  AddGround();
  // Then the axis at the origin.
  AddAxis();
  // Add the copter.
  AddCopter(copter_file_name);
  // Add the flight target.
  AddFlightTarget();

  // Add light.
  AddLight();
}

void CopterViewer::Run() {
  viewer_.Run();
}

void CopterViewer::Cleanup() {
  viewer_.Cleanup();
}

void CopterViewer::AddGround() {
  // Generate the ground texture.
  const int texture_size = 2, square_size = 1;
  std::vector<std::vector<Eigen::Vector3f>> texture_image(texture_size,
    std::vector<Eigen::Vector3f>(texture_size, Eigen::Vector3f::Zero()));
  for (int i = 0; i < texture_size; ++i) {
    for (int j = 0; j < texture_size; ++j) {
      // Determine the color of the checker.
      if ((i / square_size - j / square_size) % 2) {
        texture_image[i][j] = Eigen::Vector3f(157.0f, 150.0f, 143.0f) / 255.0f;
      } else {
        texture_image[i][j] = Eigen::Vector3f(216.0f, 208.0f, 197.0f) / 255.0f;
      }
    }
  }
  opengl_viewer::Image checker;
  checker.Initialize(texture_image);

  Eigen::Matrix3Xf ground_vertex = Eigen::Matrix3Xf::Zero(3, 4);
  ground_vertex << -1.0f, -1.0f, 1.0f, 1.0f,
    -1.0f, 1.0f, -1.0f, 1.0f,
    0.0f, 0.0f, 0.0f, 0.0f;
  Eigen::Matrix3Xi ground_face = Eigen::Matrix3Xi::Zero(3, 2);
  ground_face << 0, 2, 1, 1, 2, 3;
  const float scale = 1000.0f;
  Eigen::Matrix2Xf ground_uv = ground_vertex.topRows(2) / 2.0f * scale;
  opengl_viewer::Option ground_option;
  ground_option.SetMatrixOption("model matrix",
    opengl_viewer::Scale(scale, scale, 1.0f));
  ground_option.SetVectorOption("ambient", 0.75f, 0.77f, 0.72f);
  ground_option.SetVectorOption("diffuse", 0.84f, 0.80f, 0.81f);
  ground_option.SetIntOption("texture row num", texture_size);
  ground_option.SetIntOption("texture col num", texture_size);
  ground_option.SetMatrixOption("texture", checker.rgb_data());
  ground_option.SetMatrixOption("uv", ground_uv);
  ground_option.SetStringOption("texture mag filter", "nearest");
  viewer_.AddStaticObject(ground_vertex, ground_face, ground_option);
}

void CopterViewer::AddAxis() {
  std::vector<Eigen::Matrix3Xf> axis_vertices(2);
  std::vector<Eigen::Matrix3Xi> axis_faces(2);
  opengl_viewer::Cylinder(0.01f, 0.01f, 0.9f, 16, 1,
    axis_vertices[0], axis_faces[0]);
  opengl_viewer::Cone(0.02f, 0.15f, 16, 1, axis_vertices[1], axis_faces[1]);
  opengl_viewer::Translate(0.0f, 0.0f, 0.85f, axis_vertices[1]);
  // Merge them into a single arrow.
  Eigen::Matrix3Xf axis_vertex;
  Eigen::Matrix3Xi axis_face;
  opengl_viewer::MergeMeshes(axis_vertices, axis_faces,
    axis_vertex, axis_face);
  // Now add x axis.
  opengl_viewer::Option axis_option;
  axis_option.SetMatrixOption("model matrix",
    opengl_viewer::Rotate(90.0f, 0.0f, 1.0f, 0.0f));
  axis_option.SetVectorOption("ambient", 1.0f, 0.0f, 0.0f);
  axis_option.SetVectorOption("diffuse", 1.0f, 0.0f, 0.0f);
  axis_option.SetVectorOption("specular", 1.0f, 0.0f, 0.0f);
  viewer_.AddStaticObject(axis_vertex, axis_face, axis_option);
  // Then y axis.
  axis_option.SetMatrixOption("model matrix",
    opengl_viewer::Rotate(90.0f, -1.0f, 0.0f, 0.0f));
  axis_option.SetVectorOption("ambient", 0.0f, 1.0f, 0.0f);
  axis_option.SetVectorOption("diffuse", 0.0f, 1.0f, 0.0f);
  axis_option.SetVectorOption("specular", 0.0f, 1.0f, 0.0f);
  viewer_.AddStaticObject(axis_vertex, axis_face, axis_option);
}

void CopterViewer::AddCopter(const std::string& copter_file_name) {
  copter_.Initialize(copter_file_name);
  // Rigid components.
  const int rigid_part_num = copter_.NumOfRigidComponents();
  const int prop_num = copter_.NumOfPropellers();
  copter_animators_ = std::vector<CopterAnimator>(
    rigid_part_num + prop_num, CopterAnimator());

  const int unique_rigid_type_num =
    copter_simulation::kCopterRigidComponentNameNum;
  const int unique_type_num = unique_rigid_type_num + 1;  // +1 for props.
  Eigen::Matrix3Xf component_ambient, component_diffuse, component_specular;
  component_ambient = component_diffuse = component_specular
    = Eigen::Matrix3Xf::Zero(3, unique_type_num);
  component_ambient <<
    0.001f, 0.809f, 0.350f, 0.747f, 0.711f, 0.015f, 0.147f,
    0.564f, 0.585f, 0.896f, 0.174f, 0.514f, 0.091f, 0.166f,
    0.193f, 0.480f, 0.823f, 0.859f, 0.304f, 0.364f, 0.989f;
  component_diffuse <<
    0.446f, 0.009f, 0.571f, 0.166f, 0.352f, 0.783f, 0.302f,
    0.119f, 0.378f, 0.602f, 0.663f, 0.057f, 0.803f, 0.876f,
    0.005f, 0.532f, 0.607f, 0.451f, 0.608f, 0.520f, 0.727f;
  component_specular <<
    0.956f, 0.142f, 0.862f, 0.844f, 0.611f, 0.297f, 0.376f,
    0.926f, 0.462f, 0.210f, 0.997f, 0.392f, 0.840f, 0.093f,
    0.539f, 0.235f, 0.780f, 1.000f, 0.266f, 0.024f, 0.677f;

  // Rigid components first.
  for (int i = 0; i < rigid_part_num; ++i) {
    opengl_viewer::Option shape_option;
    copter_simulation::CopterRigidComponentName name =
      copter_.RigidMeshType(i);
    shape_option.SetVectorOption("ambient", component_ambient.col(name));
    shape_option.SetVectorOption("diffuse", component_diffuse.col(name));
    shape_option.SetVectorOption("specular", component_specular.col(name));
    if (name == copter_simulation::kPlate ||
      name == copter_simulation::kBattery ||
      name == copter_simulation::kElectronics ||
      name == copter_simulation::kRoundConnector) {
      shape_option.SetBoolOption("smooth normal", false);
    }
    const copter_simulation::TriMesh mesh =
      copter_.ReferenceRigidMesh(i);
    copter_animators_[i].Initialize(i, this);
    viewer_.AddDynamicObject(mesh.vertex().cast<float>(),
      mesh.element(), &copter_animators_[i], shape_option);
  }

  // Then propellers.
  for (int i = 0; i < prop_num; ++i) {
    opengl_viewer::Option shape_option;
    shape_option.SetVectorOption("ambient", component_ambient.rightCols(1));
    shape_option.SetVectorOption("diffuse", component_diffuse.rightCols(1));
    shape_option.SetVectorOption("specular", component_specular.rightCols(1));
    const copter_simulation::TriMesh mesh = copter_.PropellerInModelSpace();
    const int id = rigid_part_num + i;
    copter_animators_[id].Initialize(id, this);
    viewer_.AddDynamicObject(mesh.vertex().cast<float>(),
      mesh.element(), &copter_animators_[id], shape_option);
  }
}

void CopterViewer::AddFlightTarget() {
  // Use a half unit length blue arrow.
  std::vector<Eigen::Matrix3Xf> target_vertices(2);
  std::vector<Eigen::Matrix3Xi> target_faces(2);
  opengl_viewer::Cylinder(0.01f, 0.01f, 0.4f, 16, 1,
    target_vertices[0], target_faces[0]);
  opengl_viewer::Cone(0.02f, 0.15f, 16, 1,
    target_vertices[1], target_faces[1]);
  opengl_viewer::Translate(0.0f, 0.0f, 0.35f, target_vertices[1]);
  // Merge.
  Eigen::Matrix3Xf target_vertex;
  Eigen::Matrix3Xi target_face;
  opengl_viewer::MergeMeshes(target_vertices, target_faces,
    target_vertex, target_face);

  opengl_viewer::Option flight_target_option;
  opengl_viewer::Rotate(90.0f, 0.0f, 1.0f, 0.0f, target_vertex);
  flight_target_option.SetVectorOption("ambient", 0.0f, 0.0f, 1.0f);
  flight_target_option.SetVectorOption("diffuse", 0.0f, 0.0f, 1.0f);
  flight_target_option.SetVectorOption("specular", 0.0f, 0.0f, 1.0f);
  flight_target_animator_.Initialize(this);
  viewer_.AddDynamicObject(target_vertex, target_face,
    &flight_target_animator_, flight_target_option);
}

void CopterViewer::AddLight() {
  opengl_viewer::Option light_option;
  light_option.SetVectorOption("ambient", 0.54f, 0.56f, 0.51f);
  light_option.SetVectorOption("diffuse", 0.55f, 0.59f, 0.56f);
  light_option.SetVectorOption("specular", 0.21f, 0.26f, 0.24f);
  // Animate the light.
  const Eigen::Vector3f light_position(0.2f, -0.3f, -1.5f);
  light_animator_.Initialize(light_position, this);
  viewer_.AddDynamicPointLight(&light_animator_, light_option);
}

} // copter_viewer