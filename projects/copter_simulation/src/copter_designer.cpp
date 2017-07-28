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
#include "copter_designer.h"
#include <iostream>
#include "tinyxml2.h"
#include "geometry.h"
#include "inertial_sensor.h"

namespace copter_simulation {

template<typename T>
static void ParseElement(const tinyxml2::XMLElement* node,
  const std::string& name, T& value) {
  if (!node) {
    std::cout << "Error: Empty node." << std::endl;
    exit(0);
  }
  const char* attr = node->Attribute(name.c_str());
  if (attr) {
    std::stringstream str(attr);
    str >> value;
  }
}

template<int N>
static void ParseDoubles(const tinyxml2::XMLElement* node,
  const std::string& name, Eigen::Matrix<double, N, 1>& value) {
  if (!node) {
    std::cout << "Error: Empty node." << std::endl;
    exit(0);
  }
  const char* attr = node->Attribute(name.c_str());
  if (attr) {
    std::stringstream str(attr);
    for (auto i = 0; i < N; ++i)
      str >> value(i);
  }
}

CopterDesigner::CopterDesigner()
  : reference_rigid_components_(0),
  rigid_component_offsets_(0),
  reference_propeller_(),
  battery_(),
  reference_tubes_(0),
  reference_round_connectors_(0),
  reference_rotors_(0),
  design_constraints_(),
  reduced_design_parameters_(Eigen::VectorXd::Zero(0)),
  reference_to_design_transforms_(0),
  copter_body_(),
  rotors_(0) {}

void CopterDesigner::Initialize(const std::string& xml_file) {
  const std::string project_source_folder = PROJECT_SOURCE_DIR;
  tinyxml2::XMLDocument xml_doc;
  xml_doc.LoadFile(
    (project_source_folder + "/resources/copter/" + xml_file).c_str());
  if (xml_doc.ErrorID()) {
    std::cout << "Error: Failed to parse XML: " << xml_file << std::endl;
    exit(0);
  }
  const tinyxml2::XMLElement* root = xml_doc.FirstChildElement("copter");
  if (!root) {
    std::cout << "Error: No root node." << std::endl;
    exit(0);
  }

  // Plates.
  const tinyxml2::XMLElement* subnode = root->FirstChildElement("plate");
  std::string file_name("");
  double density = 0.0;
  std::vector<Eigen::Vector2d> size(0);
  std::vector<Eigen::Vector3d> xyz(0);
  std::vector<Eigen::Matrix3d> R(0);
  ParsePlate(subnode, file_name, density, xyz, R, size);
  const std::string mesh_folder = project_source_folder + "/resources/mesh/";
  TriMesh plate_mesh; plate_mesh.Initialize(mesh_folder + file_name);
  TriRigidBody plate_instance; plate_instance.Initialize(density, plate_mesh);
  // Add them into components_.
  reference_rigid_components_.clear();
  rigid_component_offsets_ = std::vector<int>(
    kCopterRigidComponentNameNum + 1, 0);
  for (auto i = 0; i < xyz.size(); ++i) {
    reference_rigid_components_.push_back({ plate_instance,
      xyz[i], R[i], Eigen::Vector3d(size[i].x(), size[i].y(), 1.0), kPlate,
      static_cast<int>(reference_rigid_components_.size())});
  }
  rigid_component_offsets_[kPlate + 1] = static_cast<int>(xyz.size());
  file_name = "", density = 0.0, xyz.clear(); R.clear();

  // Battery.
  subnode = root->FirstChildElement("battery");
  xyz.push_back(Eigen::Vector3d(0, 0, 0));
  R.push_back(Eigen::Matrix3d::Identity());
  double capacity = 0.0;
  ParseBattery(subnode, file_name, density, xyz[0], R[0], capacity);
  TriMesh battery_mesh; battery_mesh.Initialize(mesh_folder + file_name);
  TriRigidBody battery; battery.Initialize(density, battery_mesh);
  reference_rigid_components_.push_back({ battery, xyz[0], R[0],
    Eigen::Vector3d(1, 1, 1), kBattery,
    static_cast<int>(reference_rigid_components_.size()) });
  rigid_component_offsets_[kBattery + 1] =
    rigid_component_offsets_[kBattery] + 1;
  battery_.Initialize(density);
  file_name = "", density = 0.0, xyz.clear(); R.clear();

  // Electronics.
  subnode = root->FirstChildElement("electronics");
  xyz.push_back(Eigen::Vector3d(0, 0, 0));
  R.push_back(Eigen::Matrix3d::Identity());
  ParseElement<std::string>(subnode, "file", file_name);
  ParseElement<double>(subnode, "density", density);
  ParseLocation(subnode, xyz[0], R[0]);
  TriMesh electronic_mesh; electronic_mesh.Initialize(mesh_folder + file_name);
  TriRigidBody electronics; electronics.Initialize(density, electronic_mesh);
  reference_rigid_components_.push_back({ electronics, xyz[0], R[0],
    Eigen::Vector3d(1, 1, 1), kElectronics,
    static_cast<int>(reference_rigid_components_.size()) });
  rigid_component_offsets_[kElectronics + 1] =
    rigid_component_offsets_[kElectronics] + 1;
  file_name = "", density = 0.0, xyz.clear(); R.clear();

  // Tube.
  subnode = root->FirstChildElement("tube");
  std::vector<double> length(0);
  ParseTube(subnode, file_name, density, xyz, R, length);
  TriMesh tube_mesh; tube_mesh.Initialize(mesh_folder + file_name);
  TriRigidBody tube_instance; tube_instance.Initialize(density, tube_mesh);
  for (auto i = 0; i < xyz.size(); ++i) {
    reference_rigid_components_.push_back({ tube_instance, xyz[i], R[i],
      Eigen::Vector3d(length[i], 1, 1), kTube,
      static_cast<int>(reference_rigid_components_.size()) });
  }
  reference_tubes_ = std::vector<ParametrizedTube>(xyz.size());
  rigid_component_offsets_[kTube + 1] = rigid_component_offsets_[kTube]
    + static_cast<int>(xyz.size());
  file_name = "", density = 0.0, xyz.clear(); R.clear();

  // Round connectors.
  subnode = root->FirstChildElement("round_connector");
  ParseRoundConnector(subnode, file_name, density, xyz, R);
  TriMesh round_mesh; round_mesh.Initialize(mesh_folder + file_name);
  TriRigidBody round_instance; round_instance.Initialize(density, round_mesh);
  for (auto i = 0; i < xyz.size(); ++i) {
    reference_rigid_components_.push_back({ round_instance, xyz[i], R[i],
      Eigen::Vector3d(1, 1, 1), kRoundConnector,
      static_cast<int>(reference_rigid_components_.size()) });
  }
  reference_round_connectors_ = std::vector<ParametrizedConnector>(xyz.size());
  rigid_component_offsets_[kRoundConnector + 1] =
    rigid_component_offsets_[kRoundConnector] + static_cast<int>(xyz.size());
  file_name = "", density = 0.0, xyz.clear(); R.clear();

  // Motors.
  subnode = root->FirstChildElement("motor");
  std::string measurement_file = "";
  double prop_height = 0.0;
  std::vector<bool> is_ccw(0), flip_propeller(0);
  ParseMotor(subnode, file_name, density, measurement_file, prop_height, xyz,
    R, is_ccw, flip_propeller);
  TriMesh motor_mesh; motor_mesh.Initialize(mesh_folder + file_name);
  TriRigidBody motor_instance; motor_instance.Initialize(density, motor_mesh);
  for (auto i = 0; i < xyz.size(); ++i) {
    reference_rigid_components_.push_back({ motor_instance, xyz[i], R[i],
      Eigen::Vector3d(1, 1, 1), kMotor,
      static_cast<int>(reference_rigid_components_.size()) });
  }
  reference_rotors_ = std::vector<ParametrizedRotor>(xyz.size());
  rigid_component_offsets_[kMotor + 1] = rigid_component_offsets_[kMotor]
    + static_cast<int>(xyz.size());
  file_name = "", density = 0.0, xyz.clear(); R.clear();

  // Propellers.
  subnode = root->FirstChildElement("propeller");
  ParseElement<std::string>(subnode, "file", file_name);
  reference_propeller_.Initialize(mesh_folder + file_name);

  // Next, we build the connection between all parametrized parts.
  std::vector<int> connector_parents(0), motor_parents(0);
  InitializeParametrizedTubes(connector_parents);
  InitializeParametrizedRotors(measurement_file, prop_height, is_ccw,
    flip_propeller, motor_parents);
  InitializeParametrizedConnectors(connector_parents, motor_parents);

  // Figure out the design constraints.
  for (const auto& ref_tube : reference_tubes_) {
    ref_tube.AnalyzeConstraint();
  }
  for (const auto& ref_connector : reference_round_connectors_) {
    ref_connector.AnalyzeConstraint();
  }
  for (const auto& ref_rotor : reference_rotors_) {
    ref_rotor.AnalyzeConstraint();
  }
  design_constraints_ = ParametrizedShape::BuildConstraint();
  // Initially, reduced_design_parameters_ are all zero.
  reduced_design_parameters_ = Eigen::VectorXd::Zero(
    design_constraints_.NumOfReducedParameter());
  // The transform is identity.
  const int rigid_part_num = NumOfCopterRigidComponent();
  reference_to_design_transforms_ = std::vector<Eigen::Matrix4d>(
    rigid_part_num, Eigen::Matrix4d::Identity());
  // Initialize the copter body.
  std::vector<TriRigidBody> parts(rigid_part_num, TriRigidBody());
  for (int i = 0; i < rigid_part_num; ++i) {
    parts[i] = reference_rigid_components_[i].reference_rigid_body;
  }
  copter_body_.Initialize(parts);
  // Finally, initialize the rotors.
  rotors_.clear();
  for (const auto& ref_rotor : reference_rotors_) {
    Rotor new_rotor = ref_rotor.reference_rotor();
    new_rotor.set_position(copter_body_.WorldPointToBodyPoint(
      new_rotor.position()));
    new_rotor.set_direction(copter_body_.WorldVectorToBodyVector(
      new_rotor.direction()));
    rotors_.push_back(new_rotor);
  }
}

const Eigen::VectorXd CopterDesigner::GetCurrentFullDesignParameter() const {
  return design_constraints_.ToFullParameter(reduced_design_parameters_);
}

const bool CopterDesigner::SetReducedDesignParameter(
  const Eigen::VectorXd& parameter) {
  assert(parameter.size() == reduced_design_parameters_.size());
  // Determine if it is feasible.
  if (!design_constraints_.IsReducedParameterFeasible(parameter)) return false;
  // Lazy update.
  if ((reduced_design_parameters_ - parameter).cwiseAbs().maxCoeff() == 0.0)
    return true;

  // Now do the real job. Things need to update include:
  // - reduced_design_parameters_.
  // - reference_to_design_transforms_.
  // - copter_body_.
  // - rotors_.
  reduced_design_parameters_ = parameter;
  const Eigen::VectorXd full_parameters =
    design_constraints_.ToFullParameter(parameter);
  reference_to_design_transforms_ = std::vector<Eigen::Matrix4d>(
    NumOfCopterRigidComponent(), Eigen::Matrix4d::Identity());
  // Round connectors.
  for (auto i = 0; i < reference_round_connectors_.size(); ++i) {
    const int id = rigid_component_offsets_[kRoundConnector] + i;
    const ParametrizedConnector& c = reference_round_connectors_[i];
    const Eigen::VectorXd param = ParametrizedShape::ExtractParameter(
      c.name(), full_parameters);
    reference_to_design_transforms_[id] = c.Transform(param);
  }
  // Tubes.
  for (auto i = 0; i < reference_tubes_.size(); ++i) {
    const int id = rigid_component_offsets_[kTube] + i;
    const ParametrizedTube& t = reference_tubes_[i];
    const Eigen::VectorXd param = ParametrizedShape::ExtractParameter(
      t.name(), full_parameters);
    reference_to_design_transforms_[id] = t.Transform(param);
  }
  // Rotors.
  for (auto i = 0; i < reference_rotors_.size(); ++i) {
    const int id = rigid_component_offsets_[kMotor] + i;
    const ParametrizedRotor& r = reference_rotors_[i];
    const Eigen::VectorXd param = ParametrizedShape::ExtractParameter(
      r.name(), full_parameters);
    reference_to_design_transforms_[id] = r.Transform(param);
  }

  // Now build the new copter body.
  const int rigid_part_num = NumOfCopterRigidComponent();
  std::vector<TriRigidBody> new_rigid_parts(rigid_part_num, TriRigidBody());
  for (int i = 0; i < rigid_part_num; ++i) {
    new_rigid_parts[i] = reference_rigid_components_[i].reference_rigid_body;
    new_rigid_parts[i].Transform(reference_to_design_transforms_[i]);
  }
  copter_body_.Initialize(new_rigid_parts);

  // Update rotors.
  rotors_.clear();
  for (const auto& ref_rotor : reference_rotors_) {
    const Eigen::VectorXd param = ParametrizedShape::ExtractParameter(
      ref_rotor.name(), full_parameters);
    Rotor new_rotor = ref_rotor.NewRotor(param);
    new_rotor.set_position(copter_body_.WorldPointToBodyPoint(
      new_rotor.position()));
    new_rotor.set_direction(copter_body_.WorldVectorToBodyVector(
      new_rotor.direction()));
    rotors_.push_back(new_rotor);
  }

  return true;
}

const Eigen::Matrix4d CopterDesigner::TransformRigidToDesignSpace(
  const int i) const {
  assert(i >= 0 && i < NumOfCopterRigidComponent());
  return reference_to_design_transforms_[i] *
    reference_rigid_components_[i].reference_rigid_body.BodyToWorldTransform();
}

const Eigen::Matrix4d CopterDesigner::TransformPropellerToDesignSpace(
  const int i) const {
  assert(i >= 0 && i < NumOfPropellers());
  const Rotor& r = rotors_[i];
  const Eigen::Quaternionf q = Eigen::Quaternionf::FromTwoVectors(
    -Eigen::Vector3f::UnitZ(), r.direction().cast<float>());
  const Eigen::Vector3f v = (r.position() +
    r.propeller_height() * r.direction()).cast<float>();
  return copter_body_.BodyToWorldTransform() *
    (opengl_viewer::Translate(v) * opengl_viewer::Rotate(q)).cast<double>();
}

const Eigen::Matrix4d CopterDesigner::TransformRigidToDesignBodySpace(
  const int i) const {
  return copter_body_.WorldToBodyTransform() * TransformRigidToDesignSpace(i);
}

const Eigen::Matrix4d CopterDesigner::TransformPropellerToDesignBodySpace(
  const int i) const {
  assert(i >= 0 && i < NumOfPropellers());
  const Rotor& r = rotors_[i];
  const Eigen::Quaternionf q = Eigen::Quaternionf::FromTwoVectors(
    -Eigen::Vector3f::UnitZ(), r.direction().cast<float>());
  const Eigen::Vector3f v = (r.position() +
    r.propeller_height() * r.direction()).cast<float>();
  return (opengl_viewer::Translate(v)
    * opengl_viewer::Rotate(q)).cast<double>();
}

void CopterDesigner::GetRigidComponentLocationInfo(
  const CopterRigidComponentName name, int& start, int& length) const {
  start = rigid_component_offsets_[name];
  length = rigid_component_offsets_[name + 1] - rigid_component_offsets_[name];
}

const std::unordered_map<std::string, Eigen::VectorXd>&
CopterDesigner::AllParameterNameAndLength() const {
  return ParametrizedShape::all_reference_parameters();
}

void CopterDesigner::ParseLocation(const tinyxml2::XMLElement* node,
  Eigen::Vector3d& xyz, Eigen::Matrix3d& R) {
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  for (const tinyxml2::XMLElement* n = node->FirstChildElement();
    n; n = n->NextSiblingElement()) {
    if (n->Attribute("xyz")) {
      Eigen::Vector3d xyz(0, 0, 0);
      ParseDoubles<3>(n, "xyz", xyz);
      transform = opengl_viewer::Translate(xyz.cast<float>()) * transform;
    } else if (n->Attribute("rpy")) {
      Eigen::Vector3d rpy(0, 0, 0);
      ParseDoubles<3>(n, "rpy", rpy);
      rpy = DegreeToRadian(rpy);
      const Eigen::Matrix3d R = RollPitchYawToRotation(rpy);
      transform = opengl_viewer::Rotate(Eigen::Quaternionf(R.cast<float>())
        ) * transform;
    } else {
      // Ignore.
    }
  }
  xyz = transform.topRightCorner(3, 1).cast<double>();
  R = transform.topLeftCorner(3, 3).cast<double>();
}

void CopterDesigner::ParsePlate(const tinyxml2::XMLElement* node,
  std::string& file_name, double& density, std::vector<Eigen::Vector3d>& xyz,
  std::vector<Eigen::Matrix3d>& R, std::vector<Eigen::Vector2d>& size) {
  ParseElement<std::string>(node, "file", file_name);
  ParseElement<double>(node, "density", density);
  // Now loop over all instances.
  xyz.clear(); R.clear(); size.clear();
  for (const tinyxml2::XMLElement* n = node->FirstChildElement("instance");
    n; n = n->NextSiblingElement("instance")) {
    size.push_back(Eigen::Vector2d::Ones());
    ParseDoubles<2>(n, "size", size.back());
    // Parse location.
    xyz.push_back(Eigen::Vector3d::Zero());
    R.push_back(Eigen::Matrix3d::Zero());
    ParseLocation(n, xyz.back(), R.back());
  }
}

void CopterDesigner::ParseTube(const tinyxml2::XMLElement* node,
  std::string& file_name, double& density, std::vector<Eigen::Vector3d>& xyz,
  std::vector<Eigen::Matrix3d>& R, std::vector<double>& length) {
  ParseElement<std::string>(node, "file", file_name);
  ParseElement<double>(node, "density", density);
  // Now loop over all instances.
  xyz.clear(); R.clear(); length.clear();
  for (const tinyxml2::XMLElement* n = node->FirstChildElement("instance");
    n; n = n->NextSiblingElement("instance")) {
    length.push_back(1.0);
    ParseElement<double>(n, "length", length.back());
    // Parse location.
    xyz.push_back(Eigen::Vector3d::Zero());
    R.push_back(Eigen::Matrix3d::Zero());
    ParseLocation(n, xyz.back(), R.back());
  }
}

void CopterDesigner::ParseRoundConnector(const tinyxml2::XMLElement* node,
  std::string& file_name, double& density, std::vector<Eigen::Vector3d>& xyz,
  std::vector<Eigen::Matrix3d>& R) {
  ParseElement<std::string>(node, "file", file_name);
  ParseElement<double>(node, "density", density);
  // Now loop over all instances.
  xyz.clear(); R.clear();
  for (const tinyxml2::XMLElement* n = node->FirstChildElement("instance");
    n; n = n->NextSiblingElement("instance")) {
    xyz.push_back(Eigen::Vector3d::Zero());
    R.push_back(Eigen::Matrix3d::Zero());
    ParseLocation(n, xyz.back(), R.back());
  }
}

void CopterDesigner::ParseBattery(const tinyxml2::XMLElement* node,
  std::string& file_name, double& density, Eigen::Vector3d& xyz,
  Eigen::Matrix3d& R, double& capacity) {
  ParseElement<std::string>(node, "file", file_name);
  ParseElement<double>(node, "density", density);
  // Parse capacity.
  capacity = 0.0;
  ParseElement<double>(node, "capacity", capacity);
  // Parse location.
  ParseLocation(node, xyz, R);
}

void CopterDesigner::ParseMotor(const tinyxml2::XMLElement* node,
  std::string& file_name, double& density, std::string& measurement_file,
  double& propeller_height, std::vector<Eigen::Vector3d>& xyz,
  std::vector<Eigen::Matrix3d>& R, std::vector<bool>& is_ccw,
  std::vector<bool>& flip_propeller) {
  ParseElement<std::string>(node, "file", file_name);
  ParseElement<double>(node, "density", density);
  ParseElement<std::string>(node, "measurement", measurement_file);
  ParseElement<double>(node, "propeller_height", propeller_height);
  // Parse motor instances.
  xyz.clear(); R.clear(); is_ccw.clear(); flip_propeller.clear();
  for (const tinyxml2::XMLElement* n = node->FirstChildElement("instance");
    n; n = n->NextSiblingElement("instance")) {
    std::string spin_dir = "";
    ParseElement<std::string>(n, "spin_dir", spin_dir);
    is_ccw.push_back(spin_dir == "ccw");
    std::string flip_prop = "false";
    ParseElement<std::string>(n, "flip_prop", flip_prop);
    flip_propeller.push_back(flip_prop == "true");
    // Location.
    xyz.push_back(Eigen::Vector3d::Zero());
    R.push_back(Eigen::Matrix3d::Zero());
    ParseLocation(n, xyz.back(), R.back());
  }
}

void CopterDesigner::InitializeParametrizedTubes(
  std::vector<int>& connector_parents) {
  const int tube_num = static_cast<int>(reference_tubes_.size());
  const int round_num = static_cast<int>(reference_round_connectors_.size());
  connector_parents = std::vector<int>(round_num, -1);
  for (int i = 0; i < tube_num; ++i) {
    const int tube_id = rigid_component_offsets_[kTube] + i;
    const CopterReferenceRigidComponentInfo& info =
      reference_rigid_components_[tube_id];
    const Eigen::Vector3d tube_pos = info.model_to_reference_translation;
    const Eigen::Matrix3d tube_rotate = info.model_to_reference_rotation;
    const double tube_length = info.model_to_reference_scale.x();
    const Eigen::Vector3d tube_dir = tube_rotate.col(0);
    std::vector<std::pair<double, int>> adjacent_round_info(0);
    for (int j = 0; j < round_num; ++j) {
      const int round_id = rigid_component_offsets_[kRoundConnector] + j;
      const CopterReferenceRigidComponentInfo& info =
        reference_rigid_components_[round_id];
      const Eigen::Vector3d round_pos = info.model_to_reference_translation;
      // Determine if it is parallel.
      const Eigen::Vector3d tube_to_round = round_pos - tube_pos;
      if (tube_to_round.norm() > 1e-6 && tube_to_round.cross(tube_dir).norm() /
        tube_to_round.norm() > 1e-6) continue;
      // Determine if it is too far.
      const double t = tube_to_round.dot(tube_dir);
      if (t > tube_length / 2.0 || t < -tube_length / 2.0) continue;
      // It's safe to say this connector is attached to the tube now.
      adjacent_round_info.push_back(std::make_pair(t, j));
      assert(connector_parents[j] == -1);
      connector_parents[j] = i;
    }
    std::sort(adjacent_round_info.begin(), adjacent_round_info.end());
    // Now we are ready to initialize the tube.
    std::vector<ParametrizedConnector*> adjacent_round(0, NULL);
    for (const auto& info : adjacent_round_info) {
      adjacent_round.push_back(&reference_round_connectors_[info.second]);
    }
    reference_tubes_[i].Initialize(info.reference_rigid_body,
      (Eigen::Vector4d() << tube_pos, tube_length).finished(), tube_dir,
      adjacent_round);
  }
}

void CopterDesigner::InitializeParametrizedConnectors(
  const std::vector<int>& connector_parents,
  const std::vector<int>& motor_parents) {
  // Assume tubes and rotors have been initialized.
  const int round_num = static_cast<int>(reference_round_connectors_.size());
  const int round_offset = rigid_component_offsets_[kRoundConnector];
  const CopterReferenceRigidComponentInfo& round_info =
    reference_rigid_components_[round_offset];
  TriMesh round_mesh = round_info.reference_rigid_body.WorldSpaceMesh();
  round_mesh.Translate(-round_info.model_to_reference_translation);
  round_mesh.Rotate(round_info.model_to_reference_rotation.transpose());
  round_mesh.Scale(round_info.model_to_reference_scale.cwiseInverse());
  const double round_size = round_mesh.vertex().row(2).maxCoeff();

  const int plate_offset = rigid_component_offsets_[kPlate];
  const int plate_num = rigid_component_offsets_[kPlate + 1] - plate_offset;
  const CopterReferenceRigidComponentInfo& plate_info =
    reference_rigid_components_[plate_offset];
  TriMesh plate_mesh = round_info.reference_rigid_body.WorldSpaceMesh();
  plate_mesh.Translate(-round_info.model_to_reference_translation);
  plate_mesh.Rotate(round_info.model_to_reference_rotation.transpose());
  plate_mesh.Scale(round_info.model_to_reference_scale.cwiseInverse());
  const double plate_half_thickness = plate_mesh.vertex().row(2).maxCoeff();

  // First, let's build connector-to-connector connections and connector-to
  // -plate connections.
  std::vector<std::vector<ParametrizedConnectorNeighbor>> neighbors(round_num,
    { { kLeft, kFree, NULL }, { kRight, kFree, NULL },
      { kTop, kFree, NULL }, { kBottom, kFree, NULL } });
  for (int i = 0; i < round_num; ++i) {
    const int first_id = round_offset + i;
    const CopterReferenceRigidComponentInfo& first_info =
      reference_rigid_components_[first_id];
    const Eigen::Vector3d first_v = first_info.model_to_reference_translation;
    const Eigen::Matrix3d first_R = first_info.model_to_reference_rotation;
    for (int j = 0; j < round_num; ++j) {
      if (i == j) continue;
      const int second_id = round_offset + j;
      const CopterReferenceRigidComponentInfo& second_info =
        reference_rigid_components_[second_id];
      const Eigen::Vector3d second_v =
        second_info.model_to_reference_translation;
      const Eigen::Matrix3d second_R = second_info.model_to_reference_rotation;
      // Check if they are close enough.
      if ((first_v - second_v).norm() > 2.2 * round_size) continue;
      // Determine side.
      const Eigen::Vector3d first_to_second =
        first_R.transpose() * (second_v - first_v).normalized();
      NeighborLocation loc = kLeft;
      if (first_to_second.y() > 0.99) loc = kRight;
      else if (first_to_second.y() < -0.99) loc = kLeft;
      else if (first_to_second.z() > 0.99) loc = kBottom;
      else if (first_to_second.z() < -0.99) loc = kTop;
      else continue;
      assert(neighbors[i][loc].type == kFree);
      assert(!neighbors[i][loc].neighbor);
      neighbors[i][loc].type = kNeighborIsConnector;
      const ParametrizedConnector* attached = &reference_round_connectors_[j];
      neighbors[i][loc].neighbor = reinterpret_cast<const void*>(attached);
    }
    // Now link connectors to plates.
    for (int j = 0; j < plate_num; ++j) {
      const int plate_id = plate_offset + j;
      const CopterReferenceRigidComponentInfo& info =
        reference_rigid_components_[plate_id];
      const Eigen::Vector3d plate_v = info.model_to_reference_translation;
      const Eigen::Matrix3d plate_R = info.model_to_reference_rotation;
      const double size_x = info.model_to_reference_scale.x(),
        size_y = info.model_to_reference_scale.y();
      // Determine if the connector is on the plate.
      const Eigen::Vector3d plate_to_connector = (plate_R.transpose()
        * (first_v - plate_v)).cwiseAbs();
      if (plate_to_connector.z() > plate_half_thickness + round_size + 1e-3
        || plate_to_connector.x() >= size_x / 2.0
        || plate_to_connector.y() >= size_y / 2.0)
        continue;
      // Now we are certain with the connection. Figure out the face.
      const Eigen::Vector3d connector_to_plate = first_R.transpose()
        * (plate_v - first_v);
      const Eigen::Vector3d plate_normal = first_R.transpose()
        * plate_R.col(2);
      NeighborLocation loc = kLeft;
      if (plate_normal.y() > 0.9 || plate_normal.y() < -0.9) {
        loc = (connector_to_plate.y() > 0.0) ? kRight : kLeft;
      } else if (plate_normal.z() > 0.9 || plate_normal.z() < -0.9) {
        loc = (connector_to_plate.z() > 0.0) ? kBottom : kTop;
      } else {
        // Should not happen.
        assert(false);
      }
      neighbors[i][loc].type = kNeighborIsPlate;
    }
  }

  // Next, add motors to connectors.
  const int rotor_num = static_cast<int>(motor_parents.size());
  const int rotor_offset = rigid_component_offsets_[kMotor];
  for (int i = 0; i < rotor_num; ++i) {
    const int j = motor_parents[i];
    const CopterReferenceRigidComponentInfo& info =
      reference_rigid_components_[round_offset + j];
    const Eigen::Vector3d v = info.model_to_reference_translation;
    const Eigen::Matrix3d R = info.model_to_reference_rotation;
    const Eigen::Vector3d motor_v = reference_rigid_components_[
      rotor_offset + i].model_to_reference_translation;
    const Eigen::Vector3d connector_to_rotor = R.transpose() *
      (motor_v - v).normalized();
    NeighborLocation loc = kLeft;
    if (connector_to_rotor.y() > 0.9) loc = kRight;
    else if (connector_to_rotor.y() < -0.9) loc = kLeft;
    else if (connector_to_rotor.z() > 0.9) loc = kBottom;
    else if (connector_to_rotor.z() < -0.9) loc = kTop;
    else assert(false); // Should not happen.
    assert(neighbors[j][loc].type == kFree);
    assert(!neighbors[j][loc].neighbor);
    neighbors[j][loc].type = kNeighborIsRotor;
    const ParametrizedRotor* attached = &reference_rotors_[i];
    neighbors[j][loc].neighbor = reinterpret_cast<const void*>(attached);
  }

  // We have collected enough information to initialize all connectors.
  for (int i = 0; i < round_num; ++i) {
    const CopterReferenceRigidComponentInfo& info =
      reference_rigid_components_[round_offset + i];
    reference_round_connectors_[i].Initialize(info.reference_rigid_body,
      info.model_to_reference_translation,
      info.model_to_reference_rotation.col(0),
      &reference_tubes_[connector_parents[i]],
      neighbors[i]);
  }
}

void CopterDesigner::InitializeParametrizedRotors(
  const std::string& measure_file, const double prop_height,
  const std::vector<bool>& is_ccw, const std::vector<bool>& flip_propeller,
  std::vector<int>& motor_parents) {
  assert(reference_rotors_.size() == is_ccw.size());
  // Get the geometric measurement of the rotor.
  const int rotor_num = static_cast<int>(reference_rotors_.size());
  const int rotor_offset = rigid_component_offsets_[kMotor];
  const CopterReferenceRigidComponentInfo& rotor_info =
    reference_rigid_components_[rotor_offset];
  TriMesh rotor_mesh = rotor_info.reference_rigid_body.WorldSpaceMesh();
  rotor_mesh.Translate(-rotor_info.model_to_reference_translation);
  rotor_mesh.Rotate(rotor_info.model_to_reference_rotation.transpose());
  rotor_mesh.Scale(rotor_info.model_to_reference_scale.cwiseInverse());
  const double rotor_height = rotor_mesh.vertex().row(2).maxCoeff();

  const int round_num = static_cast<int>(reference_round_connectors_.size());
  const int round_offset = rigid_component_offsets_[kRoundConnector];
  const CopterReferenceRigidComponentInfo& round_info =
    reference_rigid_components_[round_offset];
  TriMesh round_mesh = round_info.reference_rigid_body.WorldSpaceMesh();
  round_mesh.Translate(-round_info.model_to_reference_translation);
  round_mesh.Rotate(round_info.model_to_reference_rotation.transpose());
  round_mesh.Scale(round_info.model_to_reference_scale.cwiseInverse());
  const double round_size = round_mesh.vertex().row(2).maxCoeff();

  // For each rotor, find the connector that it is attached.
  motor_parents = std::vector<int>(rotor_num, -1);
  const double max_dist = 1.1 * (round_size + rotor_height);
  for (int i = 0; i < rotor_num; ++i) {
    const int rotor_id = rotor_offset + i;
    const Eigen::Vector3d rotor_pos =
      reference_rigid_components_[rotor_id].model_to_reference_translation;
    const Eigen::Vector3d rotor_dir =
      reference_rigid_components_[rotor_id].model_to_reference_rotation *
      (-Eigen::Vector3d::UnitZ());
    Eigen::VectorXd rotor_param(6);
    rotor_param.head(3) = rotor_pos; rotor_param.tail(3) = rotor_dir;
    // Loop over all connectors to find the one that rigidly attached to it.
    for (int j = 0; j < round_num; ++j) {
      const Eigen::Vector3d round_pos = reference_rigid_components_[
        round_offset + j].model_to_reference_translation;
      if ((rotor_pos - round_pos).norm() < max_dist) {
        // Initialize the rotor and break.
        reference_rotors_[i].Initialize(
          reference_rigid_components_[rotor_id].reference_rigid_body,
          rotor_param, &reference_round_connectors_[j],
          measure_file, prop_height, is_ccw[i], flip_propeller[i]);
        assert(motor_parents[i] == -1);
        motor_parents[i] = j;
        break;
      }
    }
  }
}

} // copter_simulation